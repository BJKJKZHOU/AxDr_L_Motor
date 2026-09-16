#!/usr/bin/env python3
"""Run one or more J/B identifications with known electrical motor parameters.

The firmware performs I/F startup, observer handover, sinusoidal speed
excitation (default 3 Hz, 20%) and single-frequency DFT internally. This host tool only supplies
known Rs/Ls/Flux/Pp, starts the action, monitors current/protection, and reads
back the final J/B result.

Example:
    python3 tools/jb_identification_test.py \
        --port /dev/ttyACM0 --pole-pairs 7 --current-limit 2.0 \
        --rs 0.2772 --ls 76.36e-6 --flux 0.002275 \
        --wm-target 30 --count 3 --run
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import serial

import identification_test as base


IDENT_JB = 0x03
IDENT_FAIL_JB_INTERNAL = 4

FAIL_REASON_NAME = {
    0: "NONE",
    1: "PHASE_CURRENT",
    2: "FLUX_INTERNAL",
    3: "START_CONFIG",
    4: "JB_INTERNAL",
}


class JBClient(base.IdentificationClient):
    def electrical_parameters_set(self):
        self.parameter_write(base.PARAM_MOTOR_PP, base.PARAM_U8, self.args.pole_pairs)
        self.parameter_write(base.PARAM_MOTOR_RS, base.PARAM_FLOAT, self.args.rs)
        self.parameter_write(base.PARAM_MOTOR_LD, base.PARAM_FLOAT, self.args.ls)
        self.parameter_write(base.PARAM_MOTOR_LQ, base.PARAM_FLOAT, self.args.ls)
        self.parameter_write(base.PARAM_MOTOR_FLUX, base.PARAM_FLOAT, self.args.flux)

        pp = self.parameter_read(base.PARAM_MOTOR_PP, base.PARAM_U8)
        rs = self.parameter_read(base.PARAM_MOTOR_RS, base.PARAM_FLOAT)
        ld = self.parameter_read(base.PARAM_MOTOR_LD, base.PARAM_FLOAT)
        lq = self.parameter_read(base.PARAM_MOTOR_LQ, base.PARAM_FLOAT)
        flux = self.parameter_read(base.PARAM_MOTOR_FLUX, base.PARAM_FLOAT)

        if pp != self.args.pole_pairs:
            raise RuntimeError(f"pole-pairs readback {pp} != {self.args.pole_pairs}")
        for name, actual, expected in (
            ("Rs", rs, self.args.rs),
            ("Ld", ld, self.args.ls),
            ("Lq", lq, self.args.ls),
            ("Flux", flux, self.args.flux),
        ):
            if not math.isclose(actual, expected, rel_tol=1.0e-5, abs_tol=1.0e-9):
                raise RuntimeError(f"{name} readback {actual:.9g} != {expected:.9g}")

        print(
            f"Electrical parameters: Pp={pp} Rs={rs:.9g} ohm "
            f"Ls={ld * 1e6:.3f} uH Flux={flux:.9g} Wb"
        )

    def jb_result(self):
        valid = bool(self.parameter_read(base.PARAM_IDENT_JB_VALID, base.PARAM_U8))
        j = self.parameter_read(base.PARAM_IDENT_J_RESULT, base.PARAM_FLOAT)
        b = self.parameter_read(base.PARAM_IDENT_B_RESULT, base.PARAM_FLOAT)
        return {
            "valid": valid,
            "j_kgm2": j,
            "b_nms": b,
        }

    def fail_reason(self):
        reason = self.parameter_read(base.PARAM_IDENT_FAIL_REASON, base.PARAM_U8)
        return reason, FAIL_REASON_NAME.get(reason, f"UNKNOWN({reason})")

    def run_jb(self, run_number):
        self.parameter_write(base.PARAM_TARGET_SPEED, base.PARAM_FLOAT, self.args.wm_target)
        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, base.MODE_IDENT)
        self.parameter_action(base.ACTION_MOTOR_ENABLE)

        self.run_peak = 0.0
        self.run_trip = False
        self.run_trip_reason = None
        self.run_over_count = 0
        fast_lost_start = self.fast_lost
        normal_lost_start = self.normal_lost
        fast_samples_start = self.fast_samples
        normal_frames_start = self.normal_frames
        self.ident_active = True
        start = time.monotonic()
        result = None
        failure = None
        txn = None

        try:
            txn = self.parameter_action(base.ACTION_IDENT_JB_START)
            deadline = start + self.args.ident_timeout
            while time.monotonic() < deadline:
                self.process(self.parser.feed(self.ser.read(4096)))

                if self.run_trip:
                    failure = RuntimeError(self.run_trip_reason)
                    break

                status = self.action_complete_status(txn, base.ACTION_IDENT_JB_START)
                if status is None:
                    continue

                if status != 0:
                    reason, reason_name = self.fail_reason()
                    status_name = base.STATUS_NAME.get(status, str(status))
                    failure = RuntimeError(
                        f"J/B completion: {status_name}; reason={reason_name}"
                    )
                    result = {
                        "valid": False,
                        "fail_reason": reason,
                        "fail_reason_name": reason_name,
                    }
                else:
                    result = self.jb_result()
                    if not result["valid"]:
                        reason, reason_name = self.fail_reason()
                        result["fail_reason"] = reason
                        result["fail_reason_name"] = reason_name
                        failure = RuntimeError(
                            f"J/B completed without a valid result; reason={reason_name}"
                        )
                break
            else:
                failure = TimeoutError("J/B identification timeout")
        except (TimeoutError, RuntimeError) as exc:
            failure = exc
        finally:
            self.ident_active = False
            if failure is not None:
                try:
                    self.parameter_action(base.ACTION_IDENT_ABORT)
                except (TimeoutError, RuntimeError):
                    pass

        elapsed = time.monotonic() - start
        if result is None:
            result = {"valid": False}
        result.update(
            {
                "run": run_number,
                "time_s": elapsed,
                "phase_peak_a": self.run_peak,
                "fast_lost": self.fast_lost - fast_lost_start,
                "normal_lost": self.normal_lost - normal_lost_start,
                "fast_samples": self.fast_samples - fast_samples_start,
                "normal_frames": self.normal_frames - normal_frames_start,
            }
        )

        if failure is not None:
            result["error"] = str(failure)
            try:
                self.parameter_action(base.ACTION_MOTOR_DISABLE)
            except (TimeoutError, RuntimeError):
                pass
            raise base.IdentificationFailed(str(failure), result)

        if self.args.apply:
            self.parameter_action(base.ACTION_IDENT_APPLY)
            result["applied_j_kgm2"] = self.parameter_read(
                base.PARAM_MOTOR_J, base.PARAM_FLOAT
            )
            result["applied_b_nms"] = self.parameter_read(
                base.PARAM_MOTOR_B, base.PARAM_FLOAT
            )

        self.parameter_action(base.ACTION_MOTOR_DISABLE)
        return result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run J/B identification using known Rs/Ls/Flux parameters."
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--pole-pairs", type=int, required=True)
    parser.add_argument("--current-limit", type=float, required=True)
    parser.add_argument("--rs", type=float, required=True, help="phase resistance, ohm")
    parser.add_argument("--ls", type=float, required=True, help="Ld=Lq, H")
    parser.add_argument("--flux", type=float, required=True, help="PM flux, Wb")
    parser.add_argument(
        "--wm-target",
        type=float,
        required=True,
        help="signed mechanical bias speed, rad/s",
    )
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--ident-timeout", type=float, default=20.0)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output")
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if not 1 <= args.pole_pairs <= 255:
        parser.error("--pole-pairs must be between 1 and 255")
    if args.current_limit <= 0.0:
        parser.error("--current-limit must be positive")
    if args.rs <= 0.0 or args.ls <= 0.0 or args.flux <= 0.0:
        parser.error("--rs, --ls and --flux must be positive")
    if not math.isfinite(args.wm_target) or args.wm_target == 0.0:
        parser.error("--wm-target must be finite and non-zero")
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    if args.ident_timeout <= 0.0 or args.timeout <= 0.0 or args.vbus_seconds <= 0.0:
        parser.error("timeouts and --vbus-seconds must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")

    args.ident_current_limit = base.HOST_CURRENT_GUARD_RATIO * args.current_limit
    return args


def main():
    args = parse_args()
    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else Path(__file__).resolve().parents[1]
        / "build"
        / "Release"
        / f"jb_identification_{time.strftime('%Y%m%d_%H%M%S')}.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    args.capture_dir = output.parent / f"{output.stem}_plot"

    record = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "configuration": {
            "pole_pairs": args.pole_pairs,
            "current_limit_a": args.current_limit,
            "rs_ohm": args.rs,
            "ls_h": args.ls,
            "flux_wb": args.flux,
            "wm_target_rad_s": args.wm_target,
            "count": args.count,
            "apply": args.apply,
        },
        "runs": [],
        "status": "running",
    }

    client = None
    exit_code = 0
    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
            client = JBClient(ser, args)
            client.prepare()
            client.configure_plot(flux=False)
            client.check_vbus()
            client.current_limit_set()
            client.electrical_parameters_set()

            print(
                f"J/B identification: Wm_target={args.wm_target:.3f} rad/s, "
                f"runs={args.count}"
            )
            for index in range(1, args.count + 1):
                try:
                    result = client.run_jb(index)
                    record["runs"].append(result)
                    print(
                        f"J/B [{index}/{args.count}] PASS "
                        f"J={result['j_kgm2']:.9g} kg*m^2 "
                        f"B={result['b_nms']:.9g} N*m*s/rad "
                        f"time={result['time_s']:.3f} s "
                        f"I_peak={result['phase_peak_a']:.3f} A"
                    )
                except base.IdentificationFailed as exc:
                    record["runs"].append(exc.result)
                    print(
                        f"J/B [{index}/{args.count}] FAIL: {exc}",
                        file=sys.stderr,
                    )
                    exit_code = 1
                    break

                if index != args.count and args.interval > 0.0:
                    time.sleep(args.interval)

            if exit_code == 0:
                record["status"] = "pass"
            else:
                record["status"] = "fail"
    except (serial.SerialException, TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
        record["status"] = "error"
        record["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if client is not None:
            try:
                client.stop_all()
            except (serial.SerialException, TimeoutError, RuntimeError):
                pass
        record["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        output.write_text(json.dumps(record, indent=2), encoding="utf-8")
        print(f"Log: {output}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
