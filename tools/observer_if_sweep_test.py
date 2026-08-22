#!/usr/bin/env python3
"""Run guarded multi-speed I/F Shadow tests in both directions."""

import argparse
import csv
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import time

import sensorless_test as base


CUR_TS = 50.0e-6
BLOCK_SAMPLES = 20
NORMAL_FS_HZ = 1000.0
FAST_CONFIG_ID = 7
NORMAL_CONFIG_ID = 8
SENSORLESS_TARGET_SET = 0x04

FAST_VARS = (
    ("Theta_IF", 0x0014, 0.0002),
    ("Theta_obs", 0x0020, 0.0002),
    ("We_obs", 0x0021, 0.1),
    ("PLL_Err", 0x0022, 0.0001),
    ("Id", 0x0010, 0.001),
    ("Iq", 0x0011, 0.001),
    ("Ud", 0x0012, 0.001),
    ("Uq", 0x0013, 0.001),
)

NORMAL_VARS = (
    ("Vbus", 0x0004),
    ("Flux_Err", 0x0023),
)

FAST_INDEX = {name: index for index, (name, _, _) in enumerate(FAST_VARS)}
NORMAL_INDEX = {name: index for index, (name, _) in enumerate(NORMAL_VARS)}


def angle_wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def mean(values):
    return sum(values) / len(values)


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def circular_mean(values):
    sin_mean = mean([math.sin(value) for value in values])
    cos_mean = mean([math.cos(value) for value in values])
    angle = math.atan2(sin_mean, cos_mean)
    length = math.hypot(sin_mean, cos_mean)
    if length <= 0.0:
        std = math.pi
    else:
        std = math.sqrt(max(-2.0 * math.log(min(length, 1.0)), 0.0))
    return angle, std


def git_info(repo):
    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip()
        dirty = bool(subprocess.check_output(
            ["git", "status", "--short"], cwd=repo, text=True
        ).strip())
        return head, dirty
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True


class IFSweepTest(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.fast_rows = []
        self.normal_rows = []
        self.status_rows = []
        self.fast_saturation = [0] * len(FAST_VARS)
        self.normal_frames = 0
        self.normal_lost = 0
        self.normal_last = None
        self.direction = 0
        self.target = 0
        self.segment_active = False
        self.segment_fast_samples = 0
        self.segment_normal_samples = 0
        self.fast_sum = [0.0] * len(FAST_VARS)
        self.fast_sin = [0.0, 0.0]
        self.fast_cos = [0.0, 0.0]
        self.fast_block_count = 0

    def configure_plot(self):
        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
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

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for sample in range(sample_count):
            start = sample * count
            sample_raw = raw[start:start + count]
            values = [sample_raw[index] * FAST_VARS[index][2]
                      for index in range(count)]

            for index, value in enumerate(sample_raw):
                if value in (-32768, 32767):
                    self.fast_saturation[index] += 1

            current_mag = math.hypot(
                values[FAST_INDEX["Id"]], values[FAST_INDEX["Iq"]]
            )
            self.phase_peak = max(self.phase_peak, current_mag)
            if current_mag > self.args.phase_limit:
                self.tripped = True

            if not self.segment_active:
                continue

            self.segment_fast_samples += 1
            for index in range(2, len(FAST_VARS)):
                self.fast_sum[index] += values[index]
            for angle_index in range(2):
                self.fast_sin[angle_index] += math.sin(values[angle_index])
                self.fast_cos[angle_index] += math.cos(values[angle_index])

            self.fast_block_count += 1
            if self.fast_block_count == BLOCK_SAMPLES:
                row = [0.0] * len(FAST_VARS)
                for index in range(2, len(FAST_VARS)):
                    row[index] = self.fast_sum[index] / BLOCK_SAMPLES
                for index in range(2):
                    row[index] = math.atan2(
                        self.fast_sin[index], self.fast_cos[index]
                    )

                elapsed = self.segment_fast_samples * CUR_TS
                self.fast_rows.append(
                    (self.direction, self.target, elapsed, *row)
                )
                self.fast_sum = [0.0] * len(FAST_VARS)
                self.fast_sin = [0.0, 0.0]
                self.fast_cos = [0.0, 0.0]
                self.fast_block_count = 0

    def process_normal(self, payload):
        expected_len = 4 + len(NORMAL_VARS) * 4
        if (len(payload) != expected_len or
                payload[2] != NORMAL_CONFIG_ID or
                payload[3] != len(NORMAL_VARS)):
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        self.normal_frames += 1

        values = struct.unpack_from(f"<{len(NORMAL_VARS)}f", payload, 4)
        self.vbus.append(values[NORMAL_INDEX["Vbus"]])
        if not self.segment_active:
            return

        self.segment_normal_samples += 1
        elapsed = self.segment_normal_samples / NORMAL_FS_HZ
        self.normal_rows.append(
            (self.direction, self.target, elapsed, *values)
        )

    def begin_segment(self, target):
        self.target = target
        self.segment_fast_samples = 0
        self.segment_normal_samples = 0
        self.fast_sum = [0.0] * len(FAST_VARS)
        self.fast_sin = [0.0, 0.0]
        self.fast_cos = [0.0, 0.0]
        self.fast_block_count = 0
        self.segment_active = True

    def end_segment(self):
        self.segment_active = False

    def read_status(self):
        data = self.request(base.MSG_SENSORLESS, base.SENSORLESS_STATUS)
        if len(data) != 8:
            raise RuntimeError(f"invalid Sensorless status length: {len(data)}")

        active, ready, stage, if_stage = data[:4]
        we, = struct.unpack_from("<f", data, 4)
        self.stage = stage
        self.if_stage = if_stage
        self.ready = bool(ready)
        self.we = we

        if self.segment_active:
            self.status_rows.append((self.direction, self.target, we))
        if not active:
            raise RuntimeError("Sensorless stopped during sweep")
        return stage, if_stage, bool(ready), we

    def target_set(self, target):
        self.request(
            base.MSG_SENSORLESS,
            SENSORLESS_TARGET_SET,
            struct.pack("<H", target),
        )

    def process_until(self, deadline, status_period=0.05):
        next_status = time.monotonic()
        status = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                status = self.read_status()
                next_status = now + status_period
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if self.tripped:
                raise RuntimeError(
                    f"current magnitude exceeded {self.args.phase_limit:.3f} A"
                )
        return status

    def idle(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))

    def wait_if(self):
        deadline = time.monotonic() + self.args.ready_timeout
        while time.monotonic() < deadline:
            stage, _, _, _ = self.read_status()
            if stage == 1:
                return
            self.process(self.parser.feed(self.ser.read(4096)))
            if self.tripped:
                raise RuntimeError("current limit exceeded during ALIGN")
        raise TimeoutError("Sensorless did not enter I/F")

    def wait_target(self, target):
        deadline = time.monotonic() + self.args.target_timeout
        expected = self.direction * target
        while time.monotonic() < deadline:
            stage, if_stage, ready, we = self.read_status()
            if (stage == 1 and if_stage == 1 and ready and
                    abs(we - expected) <= self.args.we_tolerance):
                return
            self.process(self.parser.feed(self.ser.read(4096)))
            if self.tripped:
                raise RuntimeError("current limit exceeded during speed change")
        raise TimeoutError(f"I/F target {expected} rad/s was not reached")

    def run_direction(self, direction, targets):
        direction_code = 1 if direction > 0 else 2
        self.direction = direction
        self.request(base.MSG_CONTROL, base.CTRL_MODE_SET,
                     bytes([base.MODE_SENSORLESS_SPEED]))
        self.request(base.MSG_SENSORLESS, base.SENSORLESS_DIR_SET,
                     bytes([direction_code]))
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)
        self.request(base.MSG_CONTROL, base.CTRL_RUN)
        print(f"Direction {'forward' if direction > 0 else 'reverse'}: ALIGN")

        self.wait_if()
        for target in targets:
            self.target_set(target)
            print(f"  target={direction * target:+d} rad/s: accelerating")
            self.wait_target(target)
            print(f"  target={direction * target:+d} rad/s: HOLD")
            self.begin_segment(target)
            self.process_until(time.monotonic() + self.args.hold_seconds)
            self.end_segment()

        self.motion_stop()

    def motion_stop(self):
        self.segment_active = False
        for msg_type, op in (
            (base.MSG_CONTROL, base.CTRL_STOP),
            (base.MSG_CONTROL, base.CTRL_DISABLE),
        ):
            try:
                self.request(msg_type, op)
            except (TimeoutError, RuntimeError) as exc:
                print(f"STOP warning: {exc}", file=sys.stderr)

    def stop_all(self):
        self.motion_stop()
        try:
            self.request(
                base.MSG_PLOT,
                base.PLOT_STOP,
                bytes([base.FAST_MASK | base.NORMAL_MASK]),
            )
        except (TimeoutError, RuntimeError) as exc:
            print(f"PLOT_STOP warning: {exc}", file=sys.stderr)

    def analyze_segment(self, direction, target):
        fast_all = [row for row in self.fast_rows
                    if row[0] == direction and row[1] == target]
        normal_all = [row for row in self.normal_rows
                      if row[0] == direction and row[1] == target]
        if not fast_all or not normal_all:
            raise RuntimeError(f"missing data for {direction * target} rad/s")

        fast_end = fast_all[-1][2]
        normal_end = normal_all[-1][2]
        fast = [row for row in fast_all
                if row[2] >= fast_end - self.args.analysis_seconds]
        normal = [row for row in normal_all
                  if row[2] >= normal_end - self.args.analysis_seconds]

        offset = 3
        theta_if = [row[offset + FAST_INDEX["Theta_IF"]] for row in fast]
        theta_obs = [row[offset + FAST_INDEX["Theta_obs"]] for row in fast]
        delta = [angle_wrap(obs - theta)
                 for theta, obs in zip(theta_if, theta_obs)]
        delta_mean, delta_std = circular_mean(delta)

        signals = {
            name: [row[offset + FAST_INDEX[name]] for row in fast]
            for name in ("We_obs", "PLL_Err", "Id", "Iq", "Ud", "Uq")
        }
        i_mag = [math.hypot(d_axis, q_axis)
                 for d_axis, q_axis in zip(signals["Id"], signals["Iq"])]
        u_mag = [math.hypot(d_axis, q_axis)
                 for d_axis, q_axis in zip(signals["Ud"], signals["Uq"])]
        id_real = []
        iq_real = []
        current_delta = []

        for d_axis, q_axis, angle in zip(
                signals["Id"], signals["Iq"], delta):
            id_real.append(d_axis * math.cos(angle) +
                           q_axis * math.sin(angle))
            iq_real.append(-d_axis * math.sin(angle) +
                           q_axis * math.cos(angle))
            current_delta.append(angle_wrap(math.atan2(q_axis, d_axis) -
                                            angle))

        current_delta_mean, current_delta_std = circular_mean(current_delta)
        normal_offset = 3
        flux_err = [row[normal_offset + NORMAL_INDEX["Flux_Err"]]
                    for row in normal]
        flux_mag = [math.sqrt(max(self.args.flux_wb ** 2 - error, 0.0))
                    for error in flux_err]
        we_if = [row[2] for row in self.status_rows
                 if row[0] == direction and row[1] == target]

        delta_load = angle_wrap(direction * 0.5 * math.pi - delta_mean)
        return {
            "direction": direction,
            "we_target_rad_s": direction * target,
            "we_if_mean_rad_s": mean(we_if),
            "we_obs_mean_rad_s": mean(signals["We_obs"]),
            "we_obs_error_rad_s": (
                mean(signals["We_obs"]) - direction * target
            ),
            "delta_theta_mean_deg": math.degrees(delta_mean),
            "delta_theta_circular_std_deg": math.degrees(delta_std),
            "delta_theta_abs_deg": abs(math.degrees(delta_mean)),
            "delta_load_formula_deg": math.degrees(delta_load),
            "current_flux_angle_mean_deg": math.degrees(current_delta_mean),
            "current_flux_angle_std_deg": math.degrees(current_delta_std),
            "pll_err_mean": mean(signals["PLL_Err"]),
            "pll_err_rms": rms(signals["PLL_Err"]),
            "flux_err_mean_wb2": mean(flux_err),
            "flux_err_rms_wb2": rms(flux_err),
            "flux_mag_mean_wb": mean(flux_mag),
            "id_if_mean_a": mean(signals["Id"]),
            "iq_if_mean_a": mean(signals["Iq"]),
            "i_mag_mean_a": mean(i_mag),
            "id_real_mean_a": mean(id_real),
            "iq_real_mean_a": mean(iq_real),
            "u_mag_mean_v": mean(u_mag),
            "fast_samples": len(fast),
            "normal_samples": len(normal),
        }

    def analyze(self, targets):
        return [self.analyze_segment(direction, target)
                for direction in (1, -1) for target in targets]

    def save(self, repo, head, dirty, results):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = f"observer_if_sweep_{stamp}"
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        fast_path = output_dir / f"{stem}_fast.csv"
        normal_path = output_dir / f"{stem}_normal.csv"
        json_path = output_dir / f"{stem}.json"

        with fast_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("direction", "target_rad_s", "time_s",
                             *(item[0] for item in FAST_VARS)))
            writer.writerows(self.fast_rows)
        with normal_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("direction", "target_rad_s", "time_s",
                             *(item[0] for item in NORMAL_VARS)))
            writer.writerows(self.normal_rows)

        record = {
            "head": head,
            "dirty": dirty,
            "targets_rad_s": self.args.speeds,
            "hold_seconds": self.args.hold_seconds,
            "analysis_seconds": self.args.analysis_seconds,
            "phase_peak_a": self.phase_peak,
            "fast_frames": self.fast_frames,
            "fast_samples": self.fast_samples,
            "fast_seq_lost": self.fast_lost,
            "normal_frames": self.normal_frames,
            "normal_seq_lost": self.normal_lost,
            "fast_saturation": {
                FAST_VARS[index][0]: value
                for index, value in enumerate(self.fast_saturation)
            },
            "results": results,
        }
        json_path.write_text(json.dumps(record, indent=2) + "\n",
                             encoding="utf-8")
        return fast_path, normal_path, json_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded forward/reverse multi-speed I/F Shadow sweep"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--speeds", nargs="+", type=int,
                        default=[60, 80, 100, 120, 160, 200])
    parser.add_argument("--hold-seconds", type=float, default=1.2)
    parser.add_argument("--analysis-seconds", type=float, default=1.0)
    parser.add_argument("--phase-limit", type=float, default=2.2)
    parser.add_argument("--flux-wb", type=float, default=0.0031835556)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=5.0)
    parser.add_argument("--target-timeout", type=float, default=8.0)
    parser.add_argument("--we-tolerance", type=float, default=0.2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output-dir", default="build/Release")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if (not args.speeds or any(speed <= 0 or speed > 400
                              for speed in args.speeds)):
        parser.error("speeds must be in 1..400 rad/s")
    if args.speeds != sorted(set(args.speeds)):
        parser.error("speeds must be unique and ascending")
    if args.hold_seconds < 1.0:
        parser.error("--hold-seconds must be at least 1 s")
    if not 0.0 < args.analysis_seconds <= args.hold_seconds:
        parser.error("analysis window must fit inside hold time")
    if args.phase_limit <= 0.0 or args.flux_wb <= 0.0:
        parser.error("phase limit and flux must be positive")
    if (args.vbus_seconds <= 0.0 or args.ready_timeout <= 0.0 or
            args.target_timeout <= 0.0 or args.we_tolerance <= 0.0):
        parser.error("timeouts and speed tolerance must be positive")
    return args


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    head, dirty = git_info(repo)
    error = None
    test = None

    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = IFSweepTest(ser, args)

            try:
                test.configure_plot()
                test.check_vbus()
                test.run_direction(1, args.speeds)
                test.idle(0.2)
                test.run_direction(-1, args.speeds)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop_all()

            if error is not None:
                raise error
            if test.fast_lost or test.normal_lost:
                raise RuntimeError(
                    f"plot loss FAST={test.fast_lost} NORMAL={test.normal_lost}"
                )
            if any(test.fast_saturation):
                raise RuntimeError(f"FAST saturation: {test.fast_saturation}")

            results = test.analyze(args.speeds)
            for result in results:
                print(
                    f"{result['we_target_rad_s']:+4d}: "
                    f"We_obs={result['we_obs_mean_rad_s']:+8.3f}, "
                    f"Delta={result['delta_theta_mean_deg']:+7.2f} deg, "
                    f"delta={result['delta_load_formula_deg']:+6.2f} deg, "
                    f"Iq(real)={result['iq_real_mean_a']:+.3f} A, "
                    f"PLL_RMS={result['pll_err_rms']:.5f}, "
                    f"Flux={result['flux_mag_mean_wb']:.7f} Wb, "
                    f"U={result['u_mag_mean_v']:.3f} V"
                )

            paths = test.save(repo, head, dirty, results)
            print(f"FAST:   {paths[0]}")
            print(f"NORMAL: {paths[1]}")
            print(f"JSON:   {paths[2]}")

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
