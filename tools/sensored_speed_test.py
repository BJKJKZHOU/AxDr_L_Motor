#!/usr/bin/env python3
"""Guarded sensored SPEED regression test over Parameter/Action.

The test keeps enable/disable explicit inside this standalone regression tool.
It is not the NMIXX GUI workflow. It validates the sensed servo speed path and
records low-rate Host telemetry for repeatable comparisons.

Sequence:
    prepare -> enable -> run -> +target -> 0 -> -target -> 0 -> stop -> disable

Example:
    python3 tools/sensored_speed_test.py \
        --port /dev/ttyACM1 \
        --speed-rpm 200 \
        --hold-seconds 5 \
        --zero-seconds 3 \
        --current-limit 1.0 \
        --speed-bw 50 \
        --eso-bw 100 \
        --run
"""

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys
import time

import identification_test as base

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MODE_SPEED = 1
MOTOR_DISABLED = 0
MOTOR_ENABLED = 1
MOTOR_RUN = 2
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)


class SensoredSpeedTest(base.IdentificationClient):
    def protection_check(self):
        error = self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32)
        trip = self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32)
        if error != 0 or trip != 0:
            raise RuntimeError(
                "protection active: "
                f"error=0x{error:08X} ({base.protection_text(error)}), "
                f"trip=0x{trip:08X} ({base.protection_text(trip)})"
            )

    def encoder_check(self):
        cal_valid = self.parameter_read(base.PARAM_CAL_VALID, base.PARAM_U8)
        ready = self.parameter_read(base.PARAM_ENCODER_READY, base.PARAM_U8)
        valid = self.parameter_read(base.PARAM_ENCODER_VALID, base.PARAM_U8)
        fault = self.parameter_read(base.PARAM_ENCODER_FAULT, base.PARAM_U8)
        if cal_valid == 0:
            raise RuntimeError("Motor_Cal.Valid=0; run servo phase search first")
        if ready == 0 or valid == 0 or fault != 0:
            raise RuntimeError(
                "encoder not healthy: "
                f"ready={ready} valid={valid} fault={fault}"
            )

    def prepare(self):
        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError):
            pass

        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_DISABLED:
            raise RuntimeError(f"motor did not enter DISABLED state: state={state}")

        self.protection_check()
        self.encoder_check()

        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_SPEED)
        self.parameter_write(
            base.PARAM_LIMIT_I_MAX, base.PARAM_FLOAT, self.args.current_limit
        )
        self.parameter_write(
            base.PARAM_CTRL_SPEED_BW_HZ, base.PARAM_FLOAT, self.args.speed_bw
        )
        self.parameter_write(
            base.PARAM_CTRL_MECH_ESO_BW_HZ, base.PARAM_FLOAT, self.args.eso_bw
        )
        self.parameter_write(
            base.PARAM_MOTION_WM_ACC, base.PARAM_FLOAT, self.args.acceleration
        )
        self.parameter_write(
            base.PARAM_MOTION_WM_DEC, base.PARAM_FLOAT, self.args.deceleration
        )
        self.parameter_write(base.PARAM_TARGET_SPEED, base.PARAM_FLOAT, 0.0)

        speed_kp = self.parameter_read(base.PARAM_CTRL_SPEED_KP, base.PARAM_FLOAT)
        speed_ki = self.parameter_read(base.PARAM_CTRL_SPEED_KI, base.PARAM_FLOAT)

        print(
            f"Speed BW={self.args.speed_bw:.1f} Hz "
            f"(Kp={speed_kp:.6g}, Ki={speed_ki:.6g}); "
            f"ESO BW={self.args.eso_bw:.1f} Hz; "
            f"Imax={self.args.current_limit:.3f} A"
        )

    def start(self):
        self.parameter_action(base.ACTION_MOTOR_ENABLE)
        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_ENABLED:
            raise RuntimeError(f"motor enable failed: state={state}")

        self.parameter_action(base.ACTION_MOTOR_RUN)
        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_RUN:
            raise RuntimeError(f"motor run failed: state={state}")

    def sample_once(self, phase, start_time):
        self.protection_check()
        self.encoder_check()

        now = time.monotonic()
        return {
            "time_s": now - start_time,
            "phase": phase,
            "wm_ref": self.parameter_read(base.PARAM_REF_WM, base.PARAM_FLOAT),
            "wm_active": self.parameter_read(base.PARAM_RUN_WM, base.PARAM_FLOAT),
            "wm_eso": self.parameter_read(base.PARAM_MECH_ESO_WM, base.PARAM_FLOAT),
            "iq_ref": self.parameter_read(base.PARAM_REF_IQ, base.PARAM_FLOAT),
            "iq": self.parameter_read(base.PARAM_RUN_IQ, base.PARAM_FLOAT),
            "eso_error": self.parameter_read(base.PARAM_MECH_ESO_ERROR, base.PARAM_FLOAT),
            "td": self.parameter_read(base.PARAM_MECH_ESO_TD, base.PARAM_FLOAT),
        }

    def hold(self, phase, target, seconds, rows, start_time):
        self.parameter_write(base.PARAM_TARGET_SPEED, base.PARAM_FLOAT, target)
        print(
            f"{phase}: target={target:+.4f} rad/s "
            f"({target * RAD_S_TO_RPM:+.1f} RPM), hold={seconds:.1f} s"
        )

        deadline = time.monotonic() + seconds
        next_sample = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_sample:
                rows.append(self.sample_once(phase, start_time))
                next_sample += self.args.sample_interval
            time.sleep(min(0.005, max(0.0, next_sample - time.monotonic())))

    def stop_disable(self):
        try:
            state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
            if state == MOTOR_RUN:
                self.parameter_action(base.ACTION_MOTOR_STOP)
        except (serial.SerialException, TimeoutError, RuntimeError) as exc:
            print(f"STOP warning: {exc}", file=sys.stderr)

        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError) as exc:
            print(f"DISABLE warning: {exc}", file=sys.stderr)

    def print_summary(self, rows):
        print("\nSummary")
        for phase in ("forward", "reverse"):
            phase_rows = [row for row in rows if row["phase"] == phase]
            if not phase_rows:
                continue
            wm_eso = [row["wm_eso"] for row in phase_rows]
            iq_ref = [row["iq_ref"] for row in phase_rows]
            print(
                f"{phase:<7} "
                f"ESO={statistics.fmean(wm_eso) * RAD_S_TO_RPM:+.2f} RPM "
                f"std={statistics.pstdev(wm_eso) * RAD_S_TO_RPM:.2f} RPM; "
                f"IqRef std={statistics.pstdev(iq_ref):.4f} A"
            )


def parse_args():
    parser = argparse.ArgumentParser(description="Guarded sensored SPEED regression test")
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--speed-rpm", type=float, default=200.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument("--zero-seconds", type=float, default=3.0)
    parser.add_argument("--current-limit", type=float, default=1.0)
    parser.add_argument("--speed-bw", type=float, default=50.0)
    parser.add_argument("--eso-bw", type=float, default=100.0)
    parser.add_argument("--acceleration", type=float, default=100.0, help="rad/s^2")
    parser.add_argument("--deceleration", type=float, default=100.0, help="rad/s^2")
    parser.add_argument("--sample-interval", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--run",
        action="store_true",
        help="required confirmation that the motor may be energized",
    )
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    for name in (
        "speed_rpm", "hold_seconds", "zero_seconds", "current_limit",
        "speed_bw", "eso_bw", "acceleration", "deceleration", "sample_interval",
    ):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    return args


def main():
    args = parse_args()
    target = args.speed_rpm * RPM_TO_RAD_S
    rows = []

    if args.output is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output = Path("build/Release") / f"sensored_speed_{stamp}.csv"

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.003,
            write_timeout=1.0,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            test = SensoredSpeedTest(ser, args)
            try:
                test.prepare()
                test.start()
                t0 = time.monotonic()

                test.hold("forward", +target, args.hold_seconds, rows, t0)
                test.hold("zero_1", 0.0, args.zero_seconds, rows, t0)
                test.hold("reverse", -target, args.hold_seconds, rows, t0)
                test.hold("zero_2", 0.0, args.zero_seconds, rows, t0)

                test.print_summary(rows)
                print("\nSensored speed regression PASS")
            finally:
                test.stop_disable()

        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys() if rows else [
                "time_s", "phase", "wm_ref", "wm_active", "wm_eso",
                "iq_ref", "iq", "eso_error", "td",
            ])
            writer.writeheader()
            writer.writerows(rows)
        print(f"CSV: {args.output}")

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
