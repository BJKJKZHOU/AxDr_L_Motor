#!/usr/bin/env python3
"""Exercise only the initial fixed I/F part of Flux startup.

The tool reuses the production Flux action so the firmware path under test is
identical to normal identification. Each run is deliberately aborted before
FLUX_IF can reach the nominal 1 s ALIGN + 6 s I/F ramp boundary, preventing the
OBS_WAIT target-search stage from affecting the excitation.

This is a startup-repeatability diagnostic, not an observer test. Observer/PLL
signals are neither used for qualification nor used by this script to decide
whether the rotor followed the commanded field.

Example:
    python3 tools/if_start_test.py \
        --port /dev/ttyACM0 --pole-pairs 16 --current-limit 2.0 \
        --if-current 0.5 --count 10 --duration 6.8 --run
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

import identification_test as base


# Flux.c currently uses IF_ALIGN_TIME_S = 1.0 s and Open_Accel_S = 6.0 s.
# Keep a small host-side margin so the default run cannot enter OBS_WAIT.
IF_INITIAL_BOUNDARY_S = 7.0
IF_DURATION_MAX_S = 6.9
WE_EST_WINDOW_S = 0.20


class IFStartClient(base.IdentificationClient):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.theta_samples = []

    def process_normal(self, payload):
        super().process_normal(payload)

        if not self.ident_active or self.normal_vars != base.FLUX_NORMAL_VARS:
            return

        count = len(self.normal_vars)
        if len(payload) < 4 or payload[2] != base.NORMAL_CONFIG_ID or payload[3] != count:
            return
        expected_length = 4 + count * 4
        payload = base.canfd_payload(payload, expected_length)
        if payload is None:
            return

        values = struct.unpack_from(f"<{count}f", payload, 4)
        theta = values[base.NORMAL_INDEX["Theta_e"]]
        if math.isfinite(theta):
            self.theta_samples.append(theta)

    def theta_we_estimate(self):
        if len(self.theta_samples) < 3:
            return None

        window = max(3, int(round(WE_EST_WINDOW_S * base.NORMAL_RATE_HZ)))
        samples = self.theta_samples[-window:]
        if len(samples) < 3:
            return None

        diffs = []
        for previous, current in zip(samples, samples[1:]):
            diff = (current - previous + math.pi) % (2.0 * math.pi) - math.pi
            diffs.append(diff * base.NORMAL_RATE_HZ)
        return statistics.median(diffs) if diffs else None

    def run_if_start(self, run_number, direction):
        sign = 1.0 if direction == "forward" else -1.0
        wm_test = sign * base.IF_WE_RAD_S / self.args.pole_pairs
        self.parameter_write(base.PARAM_TARGET_SPEED, base.PARAM_FLOAT, wm_test)
        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, base.MODE_IDENT)
        self.parameter_action(base.ACTION_MOTOR_ENABLE)

        self.run_peak = 0.0
        self.run_trip = False
        self.run_trip_reason = None
        self.run_over_count = 0
        self.theta_samples = []

        fast_lost_start = self.fast_lost
        normal_lost_start = self.normal_lost
        fast_samples_start = self.fast_samples
        normal_frames_start = self.normal_frames

        current_path, state_path = self.flux_plot_start(run_number, direction)
        self.ident_active = True
        start = time.monotonic()
        action_txn = None
        error = None

        try:
            action_txn = self.parameter_action(base.ACTION_IDENT_FLUX_START)
            deadline = start + self.args.duration
            while time.monotonic() < deadline:
                self.process(self.parser.feed(self.ser.read(4096)))

                if self.run_trip:
                    raise RuntimeError(self.run_trip_reason)

                status = self.action_complete_status(
                    action_txn,
                    base.ACTION_IDENT_FLUX_START,
                )
                if status is not None:
                    status_name = base.STATUS_NAME.get(status, str(status))
                    raise RuntimeError(
                        "Flux action completed before the requested I/F-only "
                        f"window ended: {status_name}"
                    )
        except (TimeoutError, RuntimeError) as exc:
            error = str(exc)
        finally:
            self.ident_active = False
            try:
                self.parameter_action(base.ACTION_IDENT_ABORT)
            except (TimeoutError, RuntimeError):
                pass
            try:
                self.parameter_action(base.ACTION_MOTOR_DISABLE)
            except (TimeoutError, RuntimeError) as exc:
                if error is None:
                    error = f"DISABLE warning: {exc}"
                else:
                    print(f"DISABLE warning: {exc}", file=sys.stderr)
            self.flux_plot_stop()

        elapsed = time.monotonic() - start
        result = {
            "run": run_number,
            "direction": direction,
            "duration_s": elapsed,
            "phase_peak_a": self.run_peak,
            "theta_we_end_rad_s": self.theta_we_estimate(),
            "fast_lost": self.fast_lost - fast_lost_start,
            "normal_lost": self.normal_lost - normal_lost_start,
            "fast_samples": self.fast_samples - fast_samples_start,
            "normal_frames": self.normal_frames - normal_frames_start,
            "current_binary": str(current_path),
            "state_binary": str(state_path),
            "error": error,
        }
        return result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Repeat the initial fixed I/F startup interval only."
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--pole-pairs", type=int, required=True)
    parser.add_argument("--current-limit", type=float, required=True)
    parser.add_argument("--if-current", type=float, required=True)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument(
        "--duration",
        type=float,
        default=6.8,
        help=(
            "seconds from Flux action start before abort; default 6.8. "
            "Must stay below 6.9 s so OBS_WAIT target search cannot run"
        ),
    )
    parser.add_argument(
        "--direction",
        choices=("forward", "reverse"),
        default="forward",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="motor-off delay between runs, default 1 s",
    )
    parser.add_argument(
        "--prepare-rl",
        action="store_true",
        help="run one Rs/Ls identification and apply it to RAM before I/F tests",
    )
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ident-timeout", type=float, default=8.0)
    parser.add_argument("--max-lost", type=int, default=0)
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
    if args.if_current <= 0.0 or args.if_current > args.current_limit:
        parser.error("--if-current must be positive and <= --current-limit")
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.duration <= 0.0 or args.duration > IF_DURATION_MAX_S:
        parser.error(
            f"--duration must be > 0 and <= {IF_DURATION_MAX_S:.1f} s; "
            "longer runs may enter OBS_WAIT target search"
        )
    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")
    if args.vbus_seconds <= 0.0 or args.timeout <= 0.0 or args.ident_timeout <= 0.0:
        parser.error("timeouts and --vbus-seconds must be positive")
    if args.max_lost < 0:
        parser.error("--max-lost must be non-negative")

    args.ident_current_limit = base.HOST_CURRENT_GUARD_RATIO * args.current_limit
    return args


def main():
    args = parse_args()
    if args.output:
        log_path = Path(args.output).expanduser().resolve()
    else:
        log_path = (
            Path(__file__).resolve().parents[1]
            / "build"
            / "Release"
            / f"if_start_test_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    args.output = str(log_path)
    args.capture_dir = log_path.parent / f"{log_path.stem}_plot"

    record = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "configuration": {
            "pole_pairs": args.pole_pairs,
            "current_limit_a": args.current_limit,
            "if_current_a": args.if_current,
            "count": args.count,
            "duration_s": args.duration,
            "direction": args.direction,
            "interval_s": args.interval,
            "prepare_rl": args.prepare_rl,
            "initial_if_boundary_s": IF_INITIAL_BOUNDARY_S,
        },
        "runs": [],
        "status": "running",
    }

    client = None
    exit_code = 0
    error = None

    print(
        "I/F startup repeatability: "
        f"{args.count} x {args.direction}, {args.duration:.3f} s each"
    )
    print(
        "The run is aborted before the 7.0 s ALIGN+initial-I/F boundary; "
        "Observer/PLL quality is not used."
    )

    try:
        with base.serial.Serial(
            args.port,
            args.baud,
            timeout=0.003,
            write_timeout=1.0,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            client = IFStartClient(ser, args)
            try:
                client.prepare()
                client.configure_plot()
                client.check_vbus()
                client.pole_pairs_set()
                client.current_limit_set()
                client.if_current_set()

                if args.prepare_rl:
                    print("\nPreparing Rs/Ls once...")
                    rl = client.run_ident(base.IDENT_RS_LS, 1)
                    print(
                        f"Rs/Ls PASS Rs={rl['rs_ohm']:.9g} ohm "
                        f"Ls={rl['ls_h'] * 1.0e6:.4f} uH"
                    )
                    client.apply()
                    print("Rs/Ls applied to firmware RAM.")

                client.stop_plot()
                client.configure_plot(flux=True)

                for run in range(1, args.count + 1):
                    result = client.run_if_start(run, args.direction)
                    record["runs"].append(result)
                    we_text = (
                        "n/a"
                        if result["theta_we_end_rad_s"] is None
                        else f"{result['theta_we_end_rad_s']:.2f} rad/s"
                    )
                    status = "PASS" if result["error"] is None else "ERROR"
                    print(
                        f"[{run:02d}/{args.count:02d}] {status} "
                        f"time={result['duration_s']:.3f}s "
                        f"Ipeak={result['phase_peak_a']:.3f}A "
                        f"We_field_end~{we_text} "
                        f"lost={result['fast_lost']}/{result['normal_lost']}"
                    )
                    print(f"  current: {result['current_binary']}")
                    print(f"  state:   {result['state_binary']}")
                    if result["error"] is not None:
                        print(f"  error:   {result['error']}")

                    if args.interval > 0.0 and run < args.count:
                        time.sleep(args.interval)

                client.stop_plot()
                record["status"] = "completed"
            finally:
                if client is not None:
                    base.abort(client)
                    client.stop_all()

    except KeyboardInterrupt:
        record["status"] = "interrupted"
        error = "interrupted by user"
        exit_code = 130
    except (
        base.serial.SerialException,
        OSError,
        TimeoutError,
        RuntimeError,
    ) as exc:
        record["status"] = "failed"
        error = str(exc) or type(exc).__name__
        exit_code = 1
    finally:
        record["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        if error is not None:
            record["error"] = error
        log_path.write_text(
            json.dumps(record, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    print(f"Log: {log_path}")
    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
