#!/usr/bin/env python3
"""Diagnose a sensorless I/F -> observer handover without changing firmware."""

from collections import deque
import math
import struct
import time

import sensorless_run
import sensorless_test as base


# Use the existing Plot registry only. Keep the normal sensorless FAST set
# unchanged and sample diagnostic-only float signals through NORMAL transport.
FLUX_ERR_ID = 0x0023
THETA_E_ID = 0x0014
THETA_OBS_ID = 0x0020
DIAG_NORMAL_CONFIG_ID = 15
OBS_WE_TAU_S = 0.020
MONITOR_TS = 1.0 / sensorless_run.MONITOR_HZ
OBS_WE_ALPHA = MONITOR_TS / (OBS_WE_TAU_S + MONITOR_TS)
WE_ERR_MAX = 5.0


def angle_diff(a, b):
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class HandoverDiag(sensorless_run.SensorlessRun):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args, stop)
        self.we_if = 0.0
        self.handover_started = False
        self.flux_err = deque(maxlen=2000)
        self.theta_err = deque(maxlen=2000)
        self.we_obs_f = None
        self.we_err_gate_pass = 0
        self.we_err_gate_total = 0

    def configure_plot(self):
        fast_data = bytes([
            base.FAST_GROUP,
            sensorless_run.FAST_CONFIG_ID,
            len(sensorless_run.FAST_VARS),
        ])
        fast_data += b"".join(
            struct.pack("<H", var_id)
            for _, var_id, _ in sensorless_run.FAST_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_ids = (
            sensorless_run.VBUS_ID,
            FLUX_ERR_ID,
            THETA_E_ID,
            THETA_OBS_ID,
        )
        normal_data = bytes([
            base.NORMAL_GROUP,
            DIAG_NORMAL_CONFIG_ID,
            len(normal_ids),
        ])
        normal_data += b"".join(struct.pack("<H", var_id) for var_id in normal_ids)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(
            base.MSG_PLOT,
            base.PLOT_START,
            bytes([base.FAST_MASK | base.NORMAL_MASK]),
        )
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        before = len(self.blocks)
        super().process_fast(payload)

        # SensorlessRun stores one 1 kHz mean block for every 20 FAST samples.
        # Re-run the firmware's 20 ms first-order speed filter at that 1 kHz
        # observation rate. This is intentionally diagnostic-only and avoids a
        # firmware telemetry change.
        if len(self.blocks) == before:
            return

        we_obs = self.blocks[-1][sensorless_run.FAST_INDEX["We_obs"]]
        if self.we_obs_f is None:
            self.we_obs_f = we_obs
        else:
            self.we_obs_f += OBS_WE_ALPHA * (we_obs - self.we_obs_f)

        if self.handover_started:
            self.we_err_gate_total += 1
            if abs(self.we_obs_f - self.we_if) <= WE_ERR_MAX:
                self.we_err_gate_pass += 1

    def process_normal(self, payload):
        if (len(payload) != 20 or payload[2] != DIAG_NORMAL_CONFIG_ID or
                payload[3] != 4):
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq

        vbus, flux_err, theta_e, theta_obs = struct.unpack_from("<ffff", payload, 4)
        self.vbus.append(vbus)
        self.flux_err.append(flux_err)
        self.theta_err.append(angle_diff(theta_obs, theta_e))
        self.last_normal_rx = time.monotonic()

    def read_stage(self):
        data = self.request(base.MSG_SENSORLESS, base.SENSORLESS_STATUS)
        if len(data) != 8:
            raise RuntimeError(f"invalid Sensorless status length: {len(data)}")

        active, ready, stage, if_stage = data[:4]
        self.we_if, = struct.unpack_from("<f", data, 4)
        self.stage = stage
        self.if_stage = if_stage
        self.ready = bool(ready)
        if not active:
            raise RuntimeError("Sensorless stopped while running")
        return stage

    def handover_diag(self):
        values = self.snapshot()
        if values is None or not self.flux_err:
            print("  Handover diag: monitor data unavailable")
            return

        we_obs = values["We_obs"]
        we_obs_f = self.we_obs_f if self.we_obs_f is not None else we_obs
        flux_err = sum(self.flux_err) / len(self.flux_err)
        theta_err = list(self.theta_err)
        theta_mean = sum(theta_err) / len(theta_err) if theta_err else 0.0
        theta_span = (max(theta_err) - min(theta_err)) if theta_err else 0.0
        gate_ratio = (
            self.we_err_gate_pass / self.we_err_gate_total
            if self.we_err_gate_total else 0.0
        )
        print(
            "  Handover diag: "
            f"We_IF={self.we_if:.3f} rad/s, "
            f"We_obs={we_obs:.3f} rad/s, "
            f"We_obs_f={we_obs_f:.3f} rad/s, "
            f"We_err_f={we_obs_f - self.we_if:+.3f} rad/s, "
            f"We_gate={100.0 * gate_ratio:.1f}%, "
            f"Theta_err={math.degrees(theta_mean):+.1f} deg, "
            f"Theta_span={math.degrees(theta_span):.1f} deg, "
            f"PLL_RMS={values['PLL_RMS']:.5f}, "
            f"Flux_Err={flux_err:+.8e} Wb^2"
        )

    def wait_run(self):
        deadline = time.monotonic() + self.args.ready_timeout
        next_status = time.monotonic()
        next_diag = None
        last_stage = None

        while time.monotonic() < deadline:
            if self.stop_request.requested:
                return False

            now = time.monotonic()
            if now >= next_status:
                stage = self.read_stage()
                if stage != last_stage:
                    name = sensorless_run.STAGE_NAME.get(stage, str(stage))
                    print(f"  Sensorless stage={name}")
                    last_stage = stage

                if (stage == 2) and not self.handover_started:
                    # Discard ALIGN/IF history and seed the diagnostic speed
                    # filter from the current observer speed at handover entry.
                    self.blocks.clear()
                    self.flux_err.clear()
                    self.theta_err.clear()
                    self.fast_saturation = [0] * len(sensorless_run.FAST_VARS)
                    self.we_obs_f = None
                    self.we_err_gate_pass = 0
                    self.we_err_gate_total = 0
                    self.handover_started = True
                    next_diag = now + 0.5

                if stage == sensorless_run.SENSORLESS_RUN:
                    self.handover_diag()
                    return True
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if self.handover_started:
                self.guard(require_run=False)
                if next_diag is not None and now >= next_diag:
                    self.handover_diag()
                    next_diag = now + 1.0
            else:
                if self.current_trip:
                    raise RuntimeError(
                        f"current magnitude exceeded {self.args.current_limit:.3f} A"
                    )
                if self.fast_lost > self.args.max_lost:
                    raise RuntimeError(f"FAST lost {self.fast_lost} frames")
                if self.normal_lost > self.args.max_lost:
                    raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")

        self.handover_diag()
        raise TimeoutError("Sensorless did not enter SENSORLESS_RUN")


def main():
    args, wm_target = sensorless_run.parse_args()
    stop = sensorless_run.StopRequest()

    print(
        f"Handover diagnostic target: {wm_target:.3f} mechanical rad/s "
        f"({wm_target * args.pole_pairs:.1f} electrical rad/s)"
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

            test = HandoverDiag(ser, args, stop)
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

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, KeyboardInterrupt) as exc:
        error = exc

    if error is not None:
        print(f"ERROR: {error}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
