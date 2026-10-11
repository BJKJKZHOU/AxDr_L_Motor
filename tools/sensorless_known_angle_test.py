#!/usr/bin/env python3
"""Capture known-angle sensorless startup, zero-speed hold and restart over AxDr USB.

Only the initial encoder angle is used by the firmware control path. The
continuously sampled encoder angle here is diagnostic ground truth.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import sys
import time

import serial

import sensorless_test as axdr
import parameter_ids_generated as pid


MODE_KNOWN_ANGLE = 7
NORMAL_CONFIG_ID = 42
NORMAL_HZ = 1000.0
TWO_PI = 2.0 * math.pi

NORMAL_VARS = (
    ("theta_m_encoder", pid.PARAM_ENCODER_THETA_M),
    ("theta_e_observer", pid.PARAM_OBS_THETA),
    ("theta_e_foc", pid.PARAM_RUN_THETA_E),
    ("we_observer", pid.PARAM_OBS_WE),
    ("wm_eso", pid.PARAM_MECH_ESO_WM),
    ("wm_ref", pid.PARAM_REF_WM),
    ("iq_ref", pid.PARAM_REF_IQ),
    ("iq", pid.PARAM_RUN_IQ),
    ("ia", pid.PARAM_ADC_IA),
    ("ib", pid.PARAM_ADC_IB),
    ("ic", pid.PARAM_ADC_IC),
    ("vbus", pid.PARAM_ADC_VBUS),
)

STOP_REQUESTED = False


def request_stop(signum, frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


def angle_error(a, b):
    return (a - b + math.pi) % TWO_PI - math.pi


class KnownAngleTest(axdr.SensorlessTest):
    def __init__(self, ser, args, output):
        super().__init__(ser, args)
        self.output = output
        self.writer = csv.writer(output)
        self.writer.writerow(
            ("time_s", "sequence", *(name for name, _ in NORMAL_VARS),
             "theta_e_encoder", "angle_error_rad", "current_mag_a")
        )
        self.start_mono = time.monotonic()
        self.last_normal = None
        self.last_normal_mono = None
        self.normal_lost = 0
        self.samples = 0
        self.running = False
        self.trip_reason = None
        self.bad_angle_since = None
        self.pole_pairs = 0
        self.theta_offset = 0.0

    def process_fast(self, payload):
        # FAST Plot is deliberately unused during the first experiment.
        pass

    def process_normal(self, payload):
        import struct

        count = len(NORMAL_VARS)
        if len(payload) < 4 + 4 * count:
            return
        seq, config_id, received = struct.unpack_from("<HBB", payload)
        if config_id != NORMAL_CONFIG_ID or received != count:
            return
        values = struct.unpack_from("<" + "f" * count, payload, 4)
        if self.last_normal is not None:
            self.normal_lost += (seq - self.last_normal - 1) & 0xFFFF
        self.last_normal = seq
        now = time.monotonic()
        self.last_normal_mono = now
        self.samples += 1

        theta_m, theta_obs, theta_foc, we, wm_eso, wm_ref, iq_ref, iq, ia, ib, ic, vbus = values
        theta_encoder = (self.pole_pairs * theta_m + self.theta_offset) % TWO_PI
        error = angle_error(theta_obs, theta_encoder)
        current = math.sqrt(max(0.0, (2.0 / 3.0) * (ia * ia + ib * ib + ic * ic)))
        self.writer.writerow(
            (now - self.start_mono, seq, *values, theta_encoder, error, current)
        )

        if not self.running:
            return
        if not all(math.isfinite(v) for v in values):
            self.trip_reason = "non-finite telemetry"
        elif current > self.args.current_trip:
            self.trip_reason = f"measured current {current:.3f} A"
        elif not self.args.vbus_min <= vbus <= self.args.vbus_max:
            self.trip_reason = f"Vbus {vbus:.3f} V"

        if abs(error) > math.radians(self.args.angle_limit_deg):
            if self.bad_angle_since is None:
                self.bad_angle_since = now
            elif now - self.bad_angle_since >= self.args.angle_trip_seconds:
                self.trip_reason = f"angle error {math.degrees(error):+.1f} electrical deg"
        else:
            self.bad_angle_since = None

    def configure(self):
        import struct

        try:
            self.request(axdr.MSG_PLOT, axdr.PLOT_STOP, bytes([axdr.FAST_MASK | axdr.NORMAL_MASK]))
        except (RuntimeError, TimeoutError):
            pass
        ids = b"".join(struct.pack("<H", var_id) for _, var_id in NORMAL_VARS)
        self.request(
            axdr.MSG_PLOT, axdr.PLOT_CONFIG,
            bytes([axdr.NORMAL_GROUP, NORMAL_CONFIG_ID, len(NORMAL_VARS)]) + ids
        )
        self.request(axdr.MSG_PLOT, axdr.PLOT_START, bytes([axdr.NORMAL_MASK]))

    def pump_for(self, seconds):
        deadline = time.monotonic() + seconds
        next_state_check = time.monotonic()
        while time.monotonic() < deadline:
            if STOP_REQUESTED:
                raise RuntimeError("user requested stop")
            self.process(self.parser.feed(self.ser.read(4096)))
            if self.trip_reason:
                raise RuntimeError(self.trip_reason)
            now = time.monotonic()
            if self.last_normal_mono is None:
                if self.running and now - self.start_mono > 1.0:
                    raise RuntimeError("NORMAL Plot did not start")
            elif self.running and now - self.last_normal_mono > 1.0:
                raise RuntimeError("NORMAL Plot timeout")
            if self.running and now >= next_state_check:
                if self.parameter_read(pid.PARAM_MOTOR_STATE, axdr.PARAM_U8) != 2:
                    raise RuntimeError("motor left RUN")
                if self.parameter_read(pid.PARAM_ENCODER_FAULT, axdr.PARAM_U8):
                    raise RuntimeError("encoder ground-truth fault")
                next_state_check = now + 0.25


def args_parse():
    p = argparse.ArgumentParser(description="Known-angle sensorless speed experiment")
    p.add_argument("--port", required=True)
    p.add_argument("--run", action="store_true", help="required to energize the motor")
    p.add_argument("--rpm", type=float, action="append", dest="targets",
                   help="signed mechanical RPM; repeat for each target")
    p.add_argument("--hold", type=float, default=5.0)
    p.add_argument("--zero-hold", type=float, default=10.0)
    p.add_argument("--current-limit", type=float, default=0.8,
                   help="temporary RAM command current limit (A)")
    p.add_argument("--current-trip", type=float, default=1.5,
                   help="host-side measured current limit (A); not fast protection")
    p.add_argument("--angle-limit-deg", type=float, default=80.0)
    p.add_argument("--angle-trip-seconds", type=float, default=0.05)
    p.add_argument("--vbus-min", type=float, default=10.0)
    p.add_argument("--vbus-max", type=float, default=20.0)
    p.add_argument("--stop-timeout", type=float, default=10.0)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    if not args.run:
        p.error("--run is required")
    if args.targets is None:
        args.targets = [50.0, 150.0, 0.0, 50.0, 0.0]
    if any(not math.isfinite(v) for v in args.targets):
        p.error("RPM values must be finite")
    if (args.hold <= 0 or args.zero_hold <= 0 or
            args.current_limit <= 0 or args.current_trip <= 0 or
            args.angle_limit_deg <= 0 or args.angle_trip_seconds <= 0 or
            args.stop_timeout <= 0 or args.timeout <= 0 or
            args.vbus_min >= args.vbus_max):
        p.error("invalid experiment limits")
    if args.output is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output = Path("build/captures") / f"sensorless_known_angle_{stamp}.csv"
    return args


def main():
    args = args_parse()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    last_error = None

    with serial.Serial(args.port, args.baud, timeout=0.002,
                       write_timeout=1.0) as ser, args.output.open(
                           "w", newline="", encoding="utf-8") as out:
        test = KnownAngleTest(ser, args, out)
        old_current_limit = None
        stopped_normally = False
        try:
            test.parameter_action(pid.ACTION_MOTOR_DISABLE)
            if test.parameter_read(pid.PARAM_CAL_VALID, axdr.PARAM_U8) != 1:
                raise RuntimeError("servo phase calibration is invalid")
            if (test.parameter_read(pid.PARAM_ENCODER_READY, axdr.PARAM_U8) != 1 or
                    test.parameter_read(pid.PARAM_ENCODER_VALID, axdr.PARAM_U8) != 1 or
                    test.parameter_read(pid.PARAM_ENCODER_FAULT, axdr.PARAM_U8) != 0):
                raise RuntimeError("encoder not ready/valid")
            test.pole_pairs = test.parameter_read(pid.PARAM_MOTOR_PP, axdr.PARAM_U8)
            test.theta_offset = test.parameter_read(pid.PARAM_CAL_THETA_OFF, axdr.PARAM_FLOAT)
            vbus = test.parameter_read(pid.PARAM_ADC_VBUS, axdr.PARAM_FLOAT)
            if not args.vbus_min <= vbus <= args.vbus_max:
                raise RuntimeError(f"Vbus {vbus:.3f} V outside configured range")

            old_current_limit = test.parameter_read(pid.PARAM_LIMIT_I_MAX, axdr.PARAM_FLOAT)
            if old_current_limit > args.current_limit:
                test.parameter_write(pid.PARAM_LIMIT_I_MAX, axdr.PARAM_FLOAT, args.current_limit)

            test.parameter_write(pid.PARAM_MOTOR_MODE, axdr.PARAM_U8, MODE_KNOWN_ANGLE)
            test.parameter_write(pid.PARAM_TARGET_SPEED, axdr.PARAM_FLOAT, 0.0)
            test.configure()
            test.parameter_action(pid.ACTION_MOTOR_ENABLE)
            test.parameter_action(pid.ACTION_MOTOR_RUN)
            test.running = True
            print("RUN: known encoder angle used once; FOC uses NLOB + PLL")
            for index, rpm in enumerate(args.targets):
                target = rpm * TWO_PI / 60.0
                test.parameter_write(pid.PARAM_TARGET_SPEED, axdr.PARAM_FLOAT, target)
                duration = args.zero_hold if rpm == 0.0 else args.hold
                print(f"[{index + 1}/{len(args.targets)}] {rpm:+.1f} RPM for {duration:.1f} s")
                test.pump_for(duration)

            test.running = False
            test.parameter_action(pid.ACTION_MOTOR_STOP)
            deadline = time.monotonic() + args.stop_timeout
            while time.monotonic() < deadline:
                if test.parameter_read(pid.PARAM_MOTOR_STATE, axdr.PARAM_U8) != 2:
                    stopped_normally = True
                    break
                test.process(test.parser.feed(ser.read(4096)))
            if not stopped_normally:
                raise RuntimeError("STOP did not complete before timeout")
        except (RuntimeError, TimeoutError, KeyboardInterrupt, serial.SerialException) as exc:
            last_error = exc
            print(f"Experiment stopped: {exc}", file=sys.stderr)
        finally:
            test.running = False
            # Disable is immediate even when Stop is decelerating or fails.
            try:
                test.parameter_action(pid.ACTION_MOTOR_DISABLE)
            except (RuntimeError, TimeoutError) as exc:
                print(f"DISABLE failed: {exc}", file=sys.stderr)
                last_error = last_error or exc
            try:
                test.request(axdr.MSG_PLOT, axdr.PLOT_STOP, bytes([axdr.FAST_MASK | axdr.NORMAL_MASK]))
            except (RuntimeError, TimeoutError):
                pass
            if old_current_limit is not None:
                try:
                    test.parameter_write(
                        pid.PARAM_LIMIT_I_MAX, axdr.PARAM_FLOAT, old_current_limit
                    )
                except (RuntimeError, TimeoutError) as exc:
                    print(f"Current limit restore failed: {exc}", file=sys.stderr)
                    last_error = last_error or exc

        print(f"Capture: {args.output}")
        print(f"Samples={test.samples}, missing NORMAL sequences={test.normal_lost}")
        metadata = {
            "mode": "SENSORLESS_SPEED_KNOWN_ANGLE",
            "mode_value": MODE_KNOWN_ANGLE,
            "pole_pairs": test.pole_pairs,
            "theta_offset_rad": test.theta_offset,
            "targets_rpm": args.targets,
            "hold_s": args.hold,
            "zero_hold_s": args.zero_hold,
            "normal_rate_hz": NORMAL_HZ,
            "stopped_normally": stopped_normally,
            "samples": test.samples,
            "lost": test.normal_lost,
            "error": str(last_error) if last_error else None,
        }
        args.output.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    if last_error:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
