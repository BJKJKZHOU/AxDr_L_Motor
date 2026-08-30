#!/usr/bin/env python3
"""Repeated Rs/Ls identification stress test."""

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import sys
import time

import new_motor_commission as comm
import sensorless_test as base


DEFAULT_COUNT = 500
DEFAULT_CURRENT_LIMIT_A = 2.0
DEFAULT_POLE_PAIRS = 11


def log_write(record, output):
    if output:
        path = Path(output).expanduser().resolve()
    else:
        path = (
            Path(__file__).resolve().parents[1]
            / "build"
            / "Release"
            / f"rs_ls_stress_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"Log: {path}")


def progress_print(index, count, successes, failures, results):
    text = f"[{index}/{count}] success={successes} failure={failures}"
    valid = [item for item in results if item.get("valid")]
    if valid:
        rs = [item["rs_ohm"] for item in valid]
        ls = [item["ls_h"] for item in valid]
        text += (
            f" Rs_med={statistics.median(rs):.6f} ohm"
            f" Ls_med={statistics.median(ls) * 1.0e6:.2f} uH"
        )
    print(text)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Repeat Rs/Ls identification and summarize results"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--count", type=int, default=DEFAULT_COUNT,
                        help=f"number of Rs/Ls runs, default {DEFAULT_COUNT}")
    parser.add_argument("--current-limit", type=float,
                        default=DEFAULT_CURRENT_LIMIT_A)
    parser.add_argument("--pole-pairs", type=int, default=DEFAULT_POLE_PAIRS)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ident-timeout", type=float, default=45.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--interval", type=float, default=0.0,
                        help="delay between runs in seconds")
    parser.add_argument("--progress-every", type=int, default=10)
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.count <= 0:
        parser.error("--count must be positive")
    if args.current_limit <= 0.0:
        parser.error("--current-limit must be positive")
    if not 1 <= args.pole_pairs <= 255:
        parser.error("--pole-pairs must be between 1 and 255")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")
    if args.ident_timeout <= 0.0 or args.poll_interval <= 0.0:
        parser.error("ident timeout and poll interval must be positive")
    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    if args.progress_every <= 0:
        parser.error("--progress-every must be positive")
    if args.max_lost < 0:
        parser.error("--max-lost must be non-negative")

    args.ident_current_limit = (
        comm.HOST_CURRENT_GUARD_RATIO * args.current_limit
    )
    return args


def main():
    args = parse_args()
    record = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "requested_count": args.count,
        "current_limit_a": args.current_limit,
        "host_current_guard_a": args.ident_current_limit,
        "pole_pairs": args.pole_pairs,
        "results": [],
        "status": "running",
    }

    successes = 0
    failures = 0
    stages = Counter()
    error = None

    print(
        f"Rs/Ls stress: count={args.count}, current_limit={args.current_limit:.3f} A, "
        f"pole_pairs={args.pole_pairs}"
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

            client = comm.Commission(ser, args)
            try:
                client.prepare()
                client.configure_plot()
                client.check_vbus()
                record["firmware_pole_pairs"] = client.pole_pairs_set()
                client.current_limit_set()

                for run in range(1, args.count + 1):
                    print(f"\nRs/Ls stress {run}/{args.count}")
                    try:
                        result = client.run_ident(comm.IDENT_RS_LS, run)
                        successes += 1
                    except comm.IdentificationFailed as exc:
                        result = exc.result
                        failures += 1
                        stage = result.get("stage", -1)
                        stages[str(stage)] += 1

                        record["results"].append(result)
                        progress_print(
                            run, args.count, successes, failures, record["results"]
                        )

                        if args.stop_on_error:
                            break
                        if args.interval > 0.0:
                            time.sleep(args.interval)
                        continue

                    record["results"].append(result)

                    if (
                        run == 1
                        or run % args.progress_every == 0
                        or run == args.count
                    ):
                        progress_print(
                            run, args.count, successes, failures, record["results"]
                        )

                    if args.interval > 0.0:
                        time.sleep(args.interval)

                record["status"] = "completed"
            finally:
                try:
                    client.stop_all()
                except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
                    print(f"cleanup warning: {exc}", file=sys.stderr)

    except KeyboardInterrupt:
        record["status"] = "interrupted"
    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        error = exc
        record["status"] = "failed"
        record["error"] = str(exc)
    finally:
        record["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        record["success_count"] = successes
        record["failure_count"] = failures
        record["failure_stage_counts"] = dict(stages)

        valid = [item for item in record["results"] if item.get("valid")]
        if valid:
            rs = [item["rs_ohm"] for item in valid]
            ls = [item["ls_h"] for item in valid]
            record["summary"] = {
                "rs_median_ohm": statistics.median(rs),
                "rs_min_ohm": min(rs),
                "rs_max_ohm": max(rs),
                "ls_median_h": statistics.median(ls),
                "ls_min_h": min(ls),
                "ls_max_h": max(ls),
            }

        try:
            log_write(record, args.output)
        except OSError as exc:
            print(f"Log warning: {exc}", file=sys.stderr)

    print(
        f"\nFinal: success={successes}, failure={failures}, "
        f"failure_stages={dict(stages)}"
    )

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
