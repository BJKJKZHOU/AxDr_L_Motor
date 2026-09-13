#!/usr/bin/env python3
"""Run SENSORLESS_SPEED continuously at a mechanical RPM target.

The command remains active until Ctrl+C, SIGTERM, a configured duration, or a
guard trip. A normal exit ramps the mechanical-speed command back to the I/F
handover speed before STOP and DISABLE. A guard trip stops immediately.

This host-side guard cannot stop the motor after host power loss, USB loss, or
SIGKILL. Unattended operation still requires a hardware stop or a firmware
communication watchdog.

Example:
    python3 tools/sensorless_run.py \
        --port /dev/ttyACM0 --run --rpm 1000 --direction forward
"""

import argparse
from collections import deque
import math
import signal
import struct
import sys
import time

import sensorless_test as base


IF_WE_RAD_S = 120.0
FAST_CONFIG_ID = 11
NORMAL_CONFIG_ID = 12
FAST_BLOCK_SAMPLES = 20
MONITOR_HZ = 1000.0
NORMAL_HZ = 500.0
INV_SQRT3 = 1.0 / math.sqrt(3.0)
VOLT_MOD_MAX = 0.95

FAST_VARS = (
    ("Id", base.PARAM_RUN_ID, 0.001),
    ("Iq", base.PARAM_RUN_IQ, 0.001),
    ("Ud", base.PARAM_RUN_UD, 0.001),
    ("Uq", base.PARAM_RUN_UQ, 0.001),
    ("We_obs", base.PARAM_OBS_WE, 0.1),
)
FAST_INDEX = {name: index for index, (name, _, _) in enumerate(FAST_VARS)}
NORMAL_VARS = (
    ("Vbus", base.PARAM_ADC_VBUS),
    ("Theta_e", base.PARAM_RUN_THETA_E),
    ("Theta_obs", base.PARAM_OBS_THETA),
)


def mean(values):
    return sum(values) / len(values)


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class StopRequest:
    def __init__(self):
        self.count = 0

    @property
    def requested(self):
        return self.count > 0

    @property
    def force(self):
        return self.count > 1

    def handle(self, signum, frame):
        del signum, frame
        self.count += 1
        if self.count == 1:
            print("\nStop requested; ramping down. Press Ctrl+C again for immediate stop.")
        else:
            print("\nImmediate stop requested.")


class SensorlessRun(base.SensorlessTest):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args)
        self.stop_request = stop
        monitor_count = max(
            2000,
            int(math.ceil(args.observer_window * MONITOR_HZ)),
            int(math.ceil(args.speed_error_time * MONITOR_HZ)),
        )
        self.blocks = deque(maxlen=monitor_count)
        self.vbus = deque(maxlen=2000)
        self.theta_errors = deque(maxlen=2000)
        self.normal_last = None
        self.normal_lost = 0
        self.fast_saturation = [0] * len(FAST_VARS)
        self.block_sum = [0.0] * len(FAST_VARS)
        self.block_count = 0
        self.current_peak = 0.0
        self.current_trip = False
        self.last_fast_rx = None
        self.last_normal_rx = None
        self.monitor_started = None
        self.command_wm = 0.0
        self.target_we = 0.0
        self.motor_state = 0
        self.observer_ready = False

    def prepare(self):
        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (TimeoutError, RuntimeError):
            pass
        try:
            self.request(
                base.MSG_PLOT,
                base.PLOT_STOP,
                bytes([base.FAST_MASK | base.NORMAL_MASK]),
            )
        except (TimeoutError, RuntimeError):
            pass

    def configure_plot(self):
        fast_data = bytes([
            base.FAST_GROUP,
            FAST_CONFIG_ID,
            len(FAST_VARS),
        ])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([
            base.NORMAL_GROUP,
            NORMAL_CONFIG_ID,
            len(NORMAL_VARS),
        ])
        normal_data += b"".join(
            struct.pack("<H", var_id) for _, var_id in NORMAL_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(
            base.MSG_PLOT,
            base.PLOT_START,
            bytes([base.FAST_MASK | base.NORMAL_MASK]),
        )
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        if len(payload) != 4 + sample_count * count * 2:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected) & 0xFFFF
        self.fast_last = seq
        self.fast_frames += 1
        self.fast_samples += sample_count
        self.last_fast_rx = time.monotonic()

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for sample in range(sample_count):
            start = sample * count
            sample_raw = raw[start:start + count]
            values = [
                sample_raw[index] * FAST_VARS[index][2]
                for index in range(count)
            ]

            for index, value in enumerate(sample_raw):
                if value in (-32768, 32767):
                    self.fast_saturation[index] += 1

            i_mag = math.hypot(
                values[FAST_INDEX["Id"]],
                values[FAST_INDEX["Iq"]],
            )
            self.current_peak = max(self.current_peak, i_mag)
            if i_mag > self.args.current_limit:
                self.current_trip = True

            for index, value in enumerate(values):
                self.block_sum[index] += value
            self.block_count += 1

            if self.block_count == FAST_BLOCK_SAMPLES:
                self.blocks.append(tuple(
                    value / FAST_BLOCK_SAMPLES for value in self.block_sum
                ))
                self.block_sum = [0.0] * count
                self.block_count = 0

    def process_normal(self, payload):
        count = len(NORMAL_VARS)
        if (len(payload) != 4 + count * 4 or
                payload[2] != NORMAL_CONFIG_ID or payload[3] != count):
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        vbus, theta_e, theta_obs = struct.unpack_from("<fff", payload, 4)
        self.vbus.append(vbus)
        self.theta_errors.append(angle_diff(theta_e, theta_obs))
        self.last_normal_rx = time.monotonic()

    def read_motor_state(self):
        self.motor_state = self.parameter_read(
            base.PARAM_MOTOR_STATE,
            base.PARAM_U8,
        )
        if self.motor_state != 2:
            raise RuntimeError("Sensorless stopped while running")

    def speed_set(self, wm):
        self.command_wm = float(wm)
        super().speed_set(self.command_wm)

    def recent(self, seconds):
        count = max(1, int(math.ceil(seconds * MONITOR_HZ)))
        return list(self.blocks)[-count:]

    def snapshot(self):
        rows = self.recent(self.args.observer_window)
        if not rows or not self.vbus or not self.theta_errors:
            return None

        values = {}
        for name in FAST_INDEX:
            index = FAST_INDEX[name]
            values[name] = mean([row[index] for row in rows])
        vbus_count = max(1, int(math.ceil(
            self.args.observer_window * NORMAL_HZ
        )))
        values["Vbus"] = mean(list(self.vbus)[-vbus_count:])
        values["Theta_err_RMS"] = rms(
            list(self.theta_errors)[-vbus_count:]
        )
        values["U_mag"] = max(
            math.hypot(row[FAST_INDEX["Ud"]], row[FAST_INDEX["Uq"]])
            for row in rows
        )
        u_limit = values["Vbus"] * INV_SQRT3 * VOLT_MOD_MAX
        values["U_util"] = values["U_mag"] / u_limit if u_limit > 0.0 else 0.0
        return values

    def guard(self, require_run=True):
        now = time.monotonic()
        if self.current_trip:
            raise RuntimeError(
                f"current magnitude exceeded {self.args.current_limit:.3f} A"
            )
        if any(self.fast_saturation):
            raise RuntimeError(
                f"FAST quantizer saturated: {self.fast_saturation}"
            )
        if self.fast_lost > self.args.max_lost:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")
        if self.normal_lost > self.args.max_lost:
            raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")
        if require_run and not self.observer_ready:
            raise RuntimeError("observer takeover is not established")
        if (self.monitor_started is not None and self.last_fast_rx is None and
                now - self.monitor_started > 0.5):
            raise RuntimeError("no FAST monitor data")
        if self.last_fast_rx is not None and now - self.last_fast_rx > 0.5:
            raise RuntimeError("FAST monitor timeout")
        if (self.monitor_started is not None and self.last_normal_rx is None and
                now - self.monitor_started > 1.0):
            raise RuntimeError("no Vbus monitor data")
        if self.last_normal_rx is not None and now - self.last_normal_rx > 1.0:
            raise RuntimeError("Vbus monitor timeout")

        values = self.snapshot()
        if values is None:
            return
        if not self.args.vbus_min <= values["Vbus"] <= self.args.vbus_max:
            raise RuntimeError(
                f"Vbus {values['Vbus']:.3f} V outside "
                f"{self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
            )
        required = int(math.ceil(self.args.observer_window * MONITOR_HZ))
        if require_run and len(self.blocks) >= required:
            if values["U_util"] > self.args.voltage_util_limit:
                raise RuntimeError(
                    f"voltage utilization {values['U_util']:.3f} exceeded "
                    f"{self.args.voltage_util_limit:.3f}"
                )

    def pump(self, seconds, require_run=True, honor_stop=True):
        deadline = time.monotonic() + seconds
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            if honor_stop and self.stop_request.requested:
                return False

            now = time.monotonic()
            if now >= next_status:
                self.read_motor_state()
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(require_run)
        return True

    def wait_run(self):
        deadline = time.monotonic() + self.args.ready_timeout
        stable_since = None
        target_we = math.copysign(IF_WE_RAD_S, self.command_wm)
        tolerance = max(
            self.args.we_tolerance,
            IF_WE_RAD_S * self.args.we_relative_tolerance,
        )

        while time.monotonic() < deadline:
            if self.stop_request.requested:
                return False
            if not self.pump(0.05, require_run=False):
                return False

            values = self.snapshot()
            if values is None:
                continue
            speed_ok = abs(values["We_obs"] - target_we) <= tolerance
            angle_ok = (values["Theta_err_RMS"] <=
                        self.args.observer_angle_rms_limit)
            if speed_ok and angle_ok:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif (time.monotonic() - stable_since >=
                      self.args.observer_settle_seconds):
                    self.observer_ready = True
                    print(
                        "  Observer takeover inferred: "
                        f"We={values['We_obs']:.1f} rad/s, "
                        f"Theta RMS={values['Theta_err_RMS']:.4f} rad"
                    )
                    return True
            else:
                stable_since = None

        raise TimeoutError(
            "observer takeover was not inferred from speed/angle tracking"
        )

    def ramp_to(self, wm_target, honor_stop=True):
        while abs(self.command_wm - wm_target) > 1e-6:
            if honor_stop and self.stop_request.requested:
                return False
            if not honor_stop and self.stop_request.force:
                return False

            delta = wm_target - self.command_wm
            step = min(abs(delta), self.args.ramp_step)
            command = self.command_wm + (step if delta > 0.0 else -step)
            self.speed_set(command)
            if not self.pump(
                    self.args.ramp_interval,
                    require_run=True,
                    honor_stop=honor_stop):
                return False
        return True

    def wait_target(self):
        deadline = time.monotonic() + self.args.target_timeout
        stable_since = None
        tolerance = max(
            self.args.we_tolerance,
            abs(self.target_we) * self.args.we_relative_tolerance,
        )

        while time.monotonic() < deadline:
            if self.stop_request.requested:
                return False
            if not self.pump(0.05):
                return False

            values = self.snapshot()
            if values is None:
                continue
            if abs(values["We_obs"] - self.target_we) <= tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self.args.settle_seconds:
                    return True
            else:
                stable_since = None

        raise TimeoutError(
            f"speed target {self.target_we:.1f} electrical rad/s was not reached"
        )

    def print_status(self, elapsed):
        values = self.snapshot()
        if values is None:
            return
        rpm = values["We_obs"] / self.args.pole_pairs * 60.0 / (2.0 * math.pi)
        print(
            f"t={elapsed:8.1f} s RPM={rpm:8.1f} "
            f"We={values['We_obs']:8.1f} rad/s "
            f"Id={values['Id']:+.3f} A Iq={values['Iq']:+.3f} A "
            f"ThetaErr={values['Theta_err_RMS']:.4f} rad "
            f"U={100.0 * values['U_util']:.1f}% "
            f"Vbus={values['Vbus']:.2f} V"
        )

    def hold(self):
        start = time.monotonic()
        next_status = start
        speed_bad_since = None
        tolerance = max(
            self.args.we_tolerance,
            abs(self.target_we) * self.args.we_relative_tolerance,
        )

        while not self.stop_request.requested:
            now = time.monotonic()
            if self.args.duration > 0.0 and now - start >= self.args.duration:
                return
            if not self.pump(0.05):
                return

            values = self.snapshot()
            if values is None:
                continue
            if abs(values["We_obs"] - self.target_we) > tolerance:
                if speed_bad_since is None:
                    speed_bad_since = now
                elif now - speed_bad_since >= self.args.speed_error_time:
                    raise RuntimeError(
                        f"speed error persisted for "
                        f"{self.args.speed_error_time:.2f} s"
                    )
            else:
                speed_bad_since = None

            if now >= next_status:
                self.print_status(now - start)
                next_status = now + self.args.status_interval

    def run_motor(self, wm_target):
        sign = 1.0 if wm_target >= 0.0 else -1.0
        wm_if = sign * IF_WE_RAD_S / self.args.pole_pairs
        self.target_we = wm_target * self.args.pole_pairs

        self.parameter_write(
            base.PARAM_MOTOR_MODE,
            base.PARAM_U8,
            base.MODE_SENSORLESS_SPEED,
        )
        self.speed_set(wm_if)
        self.parameter_action(base.ACTION_MOTOR_ENABLE)
        self.parameter_action(base.ACTION_MOTOR_RUN)
        print(
            "ALIGN -> I/F 120 rad/s -> Observer takeover "
            "(inferred from public speed/angle signals)"
        )

        if not self.wait_run():
            return
        if not self.ramp_to(wm_target):
            return
        print(f"Command ramp complete: {wm_target:.3f} mechanical rad/s")
        if not self.wait_target():
            return
        print(f"Target stable: {self.args.rpm:.2f} mechanical RPM")
        self.hold()

    def stop_all(self, graceful):
        if graceful and self.observer_ready and not self.stop_request.force:
            try:
                sign = 1.0 if self.command_wm >= 0.0 else -1.0
                wm_if = sign * IF_WE_RAD_S / self.args.pole_pairs
                print(f"Ramping down to {wm_if:.3f} mechanical rad/s...")
                self.ramp_to(wm_if, honor_stop=False)
            except (TimeoutError, RuntimeError) as exc:
                print(f"Ramp-down warning: {exc}", file=sys.stderr)

        for name, action_id in (
            ("STOP", base.ACTION_MOTOR_STOP),
            ("DISABLE", base.ACTION_MOTOR_DISABLE),
        ):
            try:
                self.parameter_action(action_id)
                print(f"{name} OK")
            except (TimeoutError, RuntimeError) as exc:
                print(f"{name} warning: {exc}", file=sys.stderr)
        try:
            self.request(
                base.MSG_PLOT,
                base.PLOT_STOP,
                bytes([base.FAST_MASK | base.NORMAL_MASK]),
            )
            print("PLOT_STOP OK")
        except (TimeoutError, RuntimeError) as exc:
            print(f"PLOT_STOP warning: {exc}", file=sys.stderr)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Continuously run guarded sensorless speed at mechanical RPM"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--rpm", type=float, default=1000.0)
    parser.add_argument("--pole-pairs", type=int, default=16)
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="hold time in seconds; 0 runs until interrupted")
    parser.add_argument("--ramp-step", type=float, default=0.5,
                        help="mechanical rad/s per ramp step")
    parser.add_argument("--ramp-interval", type=float, default=0.02)
    parser.add_argument("--current-limit", type=float, default=2.6)
    parser.add_argument(
        "--observer-angle-rms-limit", type=float, default=0.03,
        help="maximum control/observer angle RMS used to infer takeover",
    )
    parser.add_argument(
        "--observer-window", type=float, default=0.2,
        help="speed/angle inference window in seconds",
    )
    parser.add_argument(
        "--observer-settle-seconds", type=float, default=0.5,
        help="continuous stable time required before the final speed ramp",
    )
    parser.add_argument("--we-tolerance", type=float, default=20.0)
    parser.add_argument("--we-relative-tolerance", type=float, default=0.03)
    parser.add_argument("--speed-error-time", type=float, default=1.0)
    parser.add_argument("--voltage-util-limit", type=float, default=0.985)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--target-timeout", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.rpm <= 0.0 or args.pole_pairs <= 0:
        parser.error("rpm and pole-pairs must be positive")
    if args.duration < 0.0:
        parser.error("duration must be non-negative")
    for name in (
            "ramp_step", "ramp_interval", "current_limit",
            "observer_angle_rms_limit", "observer_window",
            "observer_settle_seconds", "we_tolerance", "we_relative_tolerance",
            "speed_error_time", "voltage_util_limit", "vbus_min",
            "vbus_max", "vbus_seconds", "ready_timeout", "target_timeout",
            "settle_seconds", "status_interval", "timeout"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("vbus-min must be less than vbus-max")
    if args.voltage_util_limit > 1.0:
        parser.error("voltage-util-limit must not exceed 1")
    if args.max_lost < 0:
        parser.error("max-lost must be non-negative")

    wm_target = args.rpm * 2.0 * math.pi / 60.0
    if args.direction == "reverse":
        wm_target = -wm_target

    we_target = abs(wm_target) * args.pole_pairs
    if not IF_WE_RAD_S <= we_target <= 3000.0:
        rpm_min = IF_WE_RAD_S / args.pole_pairs * 60.0 / (2.0 * math.pi)
        rpm_max = 3000.0 / args.pole_pairs * 60.0 / (2.0 * math.pi)
        parser.error(
            f"rpm maps to {we_target:.1f} electrical rad/s; valid mechanical "
            f"range is {rpm_min:.2f} .. {rpm_max:.2f} RPM"
        )
    return args, wm_target


def main():
    args, wm_target = parse_args()
    stop = StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    print(
        f"Target: {wm_target:.3f} mechanical rad/s "
        f"({wm_target * args.pole_pairs:.1f} electrical rad/s), "
        f"direction={args.direction}"
    )

    test = None
    error = None
    graceful = False
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = SensorlessRun(ser, args, stop)
            try:
                test.prepare()
                test.configure_plot()
                test.check_vbus()
                test.run_motor(wm_target)
                graceful = True
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop_all(graceful)

            values = test.snapshot()
            if values is not None:
                test.print_status(0.0)
            print(
                f"Peak current magnitude={test.current_peak:.3f} A, "
                f"FAST lost={test.fast_lost}, NORMAL lost={test.normal_lost}"
            )

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, KeyboardInterrupt) as exc:
        error = exc

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
