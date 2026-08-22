#!/usr/bin/env python3
"""Find the guarded forward sensorless speed limit at the present Vbus."""

import argparse
import csv
import json
import math
from pathlib import Path
import struct
import sys
import time

import observer_if_sweep_test as sweep
import sensorless_test as base


SENSORLESS_SPEED_SET = 0x05
SENSORLESS_RUN = 6
INV_SQRT3 = 1.0 / math.sqrt(3.0)
VOLT_MOD_MAX = 0.95


class SpeedLimitTest(sweep.IFSweepTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.results = []

    def speed_set(self, target):
        self.request(
            base.MSG_SENSORLESS,
            SENSORLESS_SPEED_SET,
            struct.pack("<H", target),
        )

    def target_fast(self, target):
        return [row for row in self.fast_rows
                if row[0] == 1 and row[1] == target]

    def latest_signals(self, target, count=200):
        rows = self.target_fast(target)[-count:]
        if not rows:
            return None

        offset = 3
        result = {}
        for name in ("We_obs", "PLL_Err", "Id", "Iq", "Ud", "Uq"):
            index = offset + sweep.FAST_INDEX[name]
            result[name] = [row[index] for row in rows]
        return result

    def guard(self, target):
        if self.tripped:
            raise RuntimeError(
                f"current magnitude exceeded {self.args.phase_limit:.3f} A"
            )
        if any(self.fast_saturation):
            raise RuntimeError(f"FAST quantizer saturated: {self.fast_saturation}")

        signals = self.latest_signals(target, 100)
        if signals is None or len(signals["We_obs"]) < 20:
            return

        pll_rms = sweep.rms(signals["PLL_Err"])
        if pll_rms > self.args.pll_rms_limit:
            raise RuntimeError(
                f"PLL error RMS {pll_rms:.4f} exceeded "
                f"{self.args.pll_rms_limit:.4f}"
            )

        if sweep.mean(signals["We_obs"][-20:]) < 0.0:
            raise RuntimeError("observer direction became negative")

    def voltage_util(self, target, count=200):
        signals = self.latest_signals(target, count)
        if signals is None or not self.vbus:
            return 0.0

        u_mag = [math.hypot(ud, uq)
                 for ud, uq in zip(signals["Ud"], signals["Uq"])]
        u_lim = self.vbus[-1] * INV_SQRT3 * VOLT_MOD_MAX
        return max(u_mag) / u_lim if u_lim > 0.0 else 0.0

    def process_for(self, seconds, target):
        deadline = time.monotonic() + seconds
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                stage, _, _, _ = self.read_status()
                if stage != SENSORLESS_RUN:
                    raise RuntimeError(f"left SENSORLESS_RUN: stage={stage}")
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(target)

    def wait_run(self):
        deadline = time.monotonic() + self.args.ready_timeout
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                stage, _, _, _ = self.read_status()
                if stage == SENSORLESS_RUN:
                    return
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if self.tripped:
                raise RuntimeError("current limit exceeded before SENSORLESS_RUN")
        raise TimeoutError("Sensorless did not enter SENSORLESS_RUN")

    def wait_target(self, target):
        deadline = time.monotonic() + self.args.target_timeout
        stable_since = None
        voltage_since = None
        next_status = time.monotonic()
        tolerance = max(self.args.we_tolerance,
                        target * self.args.we_relative_tolerance)

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                stage, _, _, _ = self.read_status()
                if stage != SENSORLESS_RUN:
                    raise RuntimeError(f"left SENSORLESS_RUN: stage={stage}")
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            self.guard(target)
            signals = self.latest_signals(target)
            if signals is None or len(signals["We_obs"]) < 100:
                continue

            we_mean = sweep.mean(signals["We_obs"])
            if abs(we_mean - target) <= tolerance:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.args.settle_seconds:
                    return "reached"
            else:
                stable_since = None

            util = self.voltage_util(target)
            if util >= self.args.voltage_util_limit:
                if voltage_since is None:
                    voltage_since = now
                elif now - voltage_since >= self.args.settle_seconds:
                    return "voltage_limit"
            else:
                voltage_since = None

        return "timeout"

    def reset_target_data(self, target):
        self.end_segment()
        self.fast_rows = [row for row in self.fast_rows
                          if not (row[0] == 1 and row[1] == target)]
        self.normal_rows = [row for row in self.normal_rows
                            if not (row[0] == 1 and row[1] == target)]
        self.begin_segment(target)

    def analyze_target(self, target, status):
        fast_all = self.target_fast(target)
        normal_all = [row for row in self.normal_rows
                      if row[0] == 1 and row[1] == target]
        if not fast_all or not normal_all:
            raise RuntimeError(f"missing hold data at {target} rad/s")

        fast_end = fast_all[-1][2]
        normal_end = normal_all[-1][2]
        fast = [row for row in fast_all
                if row[2] >= fast_end - self.args.analysis_seconds]
        normal = [row for row in normal_all
                  if row[2] >= normal_end - self.args.analysis_seconds]
        offset = 3
        signals = {
            name: [row[offset + sweep.FAST_INDEX[name]] for row in fast]
            for name in ("We_obs", "PLL_Err", "Id", "Iq", "Ud", "Uq")
        }
        u_mag = [math.hypot(ud, uq)
                 for ud, uq in zip(signals["Ud"], signals["Uq"])]
        i_mag = [math.hypot(d_axis, q_axis)
                 for d_axis, q_axis in zip(signals["Id"], signals["Iq"])]
        vbus = [row[3 + sweep.NORMAL_INDEX["Vbus"]] for row in normal]
        u_lim = sweep.mean(vbus) * INV_SQRT3 * VOLT_MOD_MAX

        return {
            "target_we_rad_s": target,
            "target_rpm": target / 16.0 * 60.0 / (2.0 * math.pi),
            "status": status,
            "we_mean_rad_s": sweep.mean(signals["We_obs"]),
            "we_min_rad_s": min(signals["We_obs"]),
            "we_max_rad_s": max(signals["We_obs"]),
            "we_error_rad_s": sweep.mean(signals["We_obs"]) - target,
            "pll_err_rms": sweep.rms(signals["PLL_Err"]),
            "pll_err_max_abs": max(abs(value) for value in signals["PLL_Err"]),
            "id_mean_a": sweep.mean(signals["Id"]),
            "iq_mean_a": sweep.mean(signals["Iq"]),
            "iq_min_a": min(signals["Iq"]),
            "iq_max_a": max(signals["Iq"]),
            "i_mag_max_a": max(i_mag),
            "u_mag_mean_v": sweep.mean(u_mag),
            "u_mag_max_v": max(u_mag),
            "u_limit_v": u_lim,
            "voltage_util_mean": sweep.mean(u_mag) / u_lim,
            "voltage_util_max": max(u_mag) / u_lim,
            "vbus_mean_v": sweep.mean(vbus),
            "fast_samples": len(fast),
            "normal_samples": len(normal),
        }

    def hold_and_analyze(self, target, status="reached"):
        self.reset_target_data(target)
        self.process_for(self.args.hold_seconds, target)
        result = self.analyze_target(target, status)
        self.results.append(result)
        print(
            f"  {target:4d} rad/s "
            f"({result['target_rpm']:.0f} rpm): "
            f"We={result['we_mean_rad_s']:.1f}, "
            f"Iq={result['iq_mean_a']:.3f} A, "
            f"U={result['u_mag_mean_v']:.3f} V "
            f"({100.0 * result['voltage_util_mean']:.1f}%), "
            f"PLL_RMS={result['pll_err_rms']:.4f}"
        )
        return result

    def run_forward(self):
        self.direction = 1
        self.request(base.MSG_CONTROL, base.CTRL_MODE_SET,
                     bytes([base.MODE_SENSORLESS_SPEED]))
        self.request(base.MSG_SENSORLESS, base.SENSORLESS_DIR_SET, b"\x01")
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)
        self.begin_segment(120)
        self.request(base.MSG_CONTROL, base.CTRL_RUN)
        print("ALIGN -> I/F 120 rad/s -> Observer takeover")
        self.wait_run()
        print("SENSORLESS_RUN reached")
        self.hold_and_analyze(120)

        for target in self.args.targets:
            self.reset_target_data(target)
            self.speed_set(target)
            print(f"  target {target} rad/s: accelerating")
            status = self.wait_target(target)
            if status != "reached":
                print(f"  target {target} rad/s stopped by {status}")
                return status, target

            result = self.hold_and_analyze(target)
            if result["voltage_util_max"] >= self.args.voltage_util_limit:
                return "voltage_limit", target

        return "target_list_complete", self.args.targets[-1]

    def save(self, repo, head, dirty, stop_reason, stop_target):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = f"sensorless_speed_limit_{stamp}"
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        fast_path = output_dir / f"{stem}_fast.csv"
        normal_path = output_dir / f"{stem}_normal.csv"
        json_path = output_dir / f"{stem}.json"

        with fast_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("direction", "target_rad_s", "time_s",
                             *(item[0] for item in sweep.FAST_VARS)))
            writer.writerows(self.fast_rows)
        with normal_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("direction", "target_rad_s", "time_s",
                             *(item[0] for item in sweep.NORMAL_VARS)))
            writer.writerows(self.normal_rows)

        record = {
            "head": head,
            "dirty": dirty,
            "targets_rad_s": self.args.targets,
            "stop_reason": stop_reason,
            "stop_target_rad_s": stop_target,
            "phase_peak_a": self.phase_peak,
            "fast_seq_lost": self.fast_lost,
            "normal_seq_lost": self.normal_lost,
            "fast_saturation": self.fast_saturation,
            "results": self.results,
        }
        json_path.write_text(json.dumps(record, indent=2) + "\n",
                             encoding="utf-8")
        return fast_path, normal_path, json_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded forward sensorless voltage speed-limit test"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--targets", nargs="+", type=int, default=[
        200, 400, 600, 800, 1000, 1200, 1400, 1600,
        1800, 2000, 2200, 2350, 2450, 2550, 2650,
    ])
    parser.add_argument("--hold-seconds", type=float, default=0.7)
    parser.add_argument("--analysis-seconds", type=float, default=0.5)
    parser.add_argument("--settle-seconds", type=float, default=0.2)
    parser.add_argument("--target-timeout", type=float, default=4.0)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--we-tolerance", type=float, default=20.0)
    parser.add_argument("--we-relative-tolerance", type=float, default=0.03)
    parser.add_argument("--phase-limit", type=float, default=2.6)
    parser.add_argument("--pll-rms-limit", type=float, default=0.08)
    parser.add_argument("--voltage-util-limit", type=float, default=0.985)
    parser.add_argument("--flux-wb", type=float, default=0.0031835556)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output-dir", default="build/Release")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if (not args.targets or args.targets != sorted(set(args.targets)) or
            any(target < 120 or target > 3000 for target in args.targets)):
        parser.error("targets must be unique ascending values in 120..3000")
    if not 0.0 < args.analysis_seconds <= args.hold_seconds:
        parser.error("analysis window must fit inside hold time")
    if (args.phase_limit <= 0.0 or args.pll_rms_limit <= 0.0 or
            not 0.0 < args.voltage_util_limit <= 1.0):
        parser.error("protection limits must be positive and valid")
    return args


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    head, dirty = sweep.git_info(repo)
    test = None
    error = None
    stop_reason = "not_started"
    stop_target = 120

    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = SpeedLimitTest(ser, args)
            try:
                test.configure_plot()
                test.check_vbus()
                stop_reason, stop_target = test.run_forward()
            except (TimeoutError, RuntimeError) as exc:
                error = exc
                stop_reason = str(exc)
            finally:
                test.stop_all()

            paths = test.save(repo, head, dirty, stop_reason, stop_target)
            print(f"stop={stop_reason}, target={stop_target} rad/s")
            print(f"peak current magnitude={test.phase_peak:.3f} A")
            print(f"FAST lost={test.fast_lost}, NORMAL lost={test.normal_lost}")
            print(f"FAST:   {paths[0]}")
            print(f"NORMAL: {paths[1]}")
            print(f"JSON:   {paths[2]}")

            if error is not None:
                raise error

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
