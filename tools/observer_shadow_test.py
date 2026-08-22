#!/usr/bin/env python3
"""Capture SENSORLESS_SPEED observer Shadow signals over USB CDC.

The observer remains diagnostic-only: Motor_Run.Theta_e continues to come from
I/F startup. Samples are reduced from 20 kHz to 1 kHz rows for CSV output while
the raw phase-current peak is still checked at 20 kHz.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import struct
import sys
import time

import sensorless_test as base


CUR_TS = 50.0e-6
BLOCK_SAMPLES = 20
FAST_CONFIG_ID = 4
FAST_VARS = (
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
    ("Ic", 0x0003, 0.001),
    ("Theta_IF", 0x0014, 0.0002),
    ("Theta_obs", 0x0020, 0.0002),
    ("We_obs", 0x0021, 0.1),
    ("PLL_Err", 0x0022, 0.0001),
    ("Flux_Err", 0x0023, 1.0e-9),
)


def angle_wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ShadowStats:
    def __init__(self):
        self.count = 0
        self.diff_sin = 0.0
        self.diff_cos = 0.0
        self.we_sum = 0.0
        self.we_sq = 0.0
        self.we_min = math.inf
        self.we_max = -math.inf
        self.pll_sq = 0.0
        self.pll_max = 0.0
        self.flux_sum = 0.0
        self.flux_max = 0.0

    def add(self, row):
        diff = angle_wrap(row[4] - row[3])
        we = row[5]
        pll_err = row[6]
        flux_err = row[7]

        self.count += 1
        self.diff_sin += math.sin(diff)
        self.diff_cos += math.cos(diff)
        self.we_sum += we
        self.we_sq += we * we
        self.we_min = min(self.we_min, we)
        self.we_max = max(self.we_max, we)
        self.pll_sq += pll_err * pll_err
        self.pll_max = max(self.pll_max, abs(pll_err))
        self.flux_sum += flux_err
        self.flux_max = max(self.flux_max, abs(flux_err))

    def summary(self, flux_wb):
        mean_we = self.we_sum / self.count
        we_var = self.we_sq / self.count - mean_we * mean_we
        mean_diff = math.atan2(self.diff_sin, self.diff_cos)
        resultant = math.hypot(self.diff_sin, self.diff_cos) / self.count
        diff_rms = math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1.0e-12))))
        flux_sq = flux_wb * flux_wb

        return {
            "blocks": self.count,
            "we_mean_rad_s": mean_we,
            "we_ripple_rms_rad_s": math.sqrt(max(we_var, 0.0)),
            "we_min_rad_s": self.we_min,
            "we_max_rad_s": self.we_max,
            "angle_diff_mean_rad": mean_diff,
            "angle_diff_mean_deg": math.degrees(mean_diff),
            "angle_diff_circular_rms_rad": diff_rms,
            "angle_diff_circular_rms_deg": math.degrees(diff_rms),
            "pll_err_rms": math.sqrt(self.pll_sq / self.count),
            "pll_err_max": self.pll_max,
            "flux_err_mean_wb2": self.flux_sum / self.count,
            "flux_err_max_wb2": self.flux_max,
            "flux_err_mean_pu": (self.flux_sum / self.count) / flux_sq,
            "flux_err_max_pu": self.flux_max / flux_sq,
        }


class ObserverShadowTest(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.rows = []
        self.motion_samples = 0
        self.block_count = 0
        self.linear_sum = [0.0] * len(FAST_VARS)
        self.theta_sin = [0.0, 0.0]
        self.theta_cos = [0.0, 0.0]
        self.saturation = [0] * len(FAST_VARS)
        self.shadow_stats = {
            "ALIGN": ShadowStats(),
            "ACCEL": ShadowStats(),
            "HOLD": ShadowStats(),
        }

    def configure_plot(self):
        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, 2, 1])
        normal_data += struct.pack("<H", base.VBUS_ID)
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
        channel_count = len(FAST_VARS)
        expected = 4 + sample_count * channel_count * 2
        if len(payload) != expected:
            return

        if self.fast_last is not None:
            expected_seq = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected_seq) & 0xFFFF
        self.fast_last = seq
        self.fast_frames += 1
        self.fast_samples += sample_count

        raw = struct.unpack_from(
            f"<{sample_count * channel_count}h", payload, 4
        )

        for sample in range(sample_count):
            start = sample * channel_count
            sample_raw = raw[start:start + channel_count]
            values = [
                sample_raw[index] * FAST_VARS[index][2]
                for index in range(channel_count)
            ]

            for index, value in enumerate(sample_raw):
                if value in (-32768, 32767):
                    self.saturation[index] += 1

            if not self.motion_active:
                continue

            self.motion_samples += 1
            phase_abs = max(abs(values[0]), abs(values[1]), abs(values[2]))
            self.phase_peak = max(self.phase_peak, phase_abs)
            if phase_abs > self.args.phase_limit:
                self.tripped = True

            for index in (0, 1, 2, 5, 6, 7):
                self.linear_sum[index] += values[index]
            for theta_index, value_index in enumerate((3, 4)):
                self.theta_sin[theta_index] += math.sin(values[value_index])
                self.theta_cos[theta_index] += math.cos(values[value_index])

            self.block_count += 1
            if self.block_count == BLOCK_SAMPLES:
                self._finish_block()

    def _finish_block(self):
        row = [0.0] * len(FAST_VARS)
        for index in (0, 1, 2, 5, 6, 7):
            row[index] = self.linear_sum[index] / BLOCK_SAMPLES
        row[3] = math.atan2(self.theta_sin[0], self.theta_cos[0]) % (2.0 * math.pi)
        row[4] = math.atan2(self.theta_sin[1], self.theta_cos[1]) % (2.0 * math.pi)

        stage = self.stage_name()
        elapsed = self.motion_samples * CUR_TS
        self.rows.append((elapsed, stage, *row))
        self.shadow_stats[stage].add(row)

        self.block_count = 0
        self.linear_sum = [0.0] * len(FAST_VARS)
        self.theta_sin = [0.0, 0.0]
        self.theta_cos = [0.0, 0.0]

    def result(self, direction):
        stages = {}
        for name, stats in self.shadow_stats.items():
            if stats.count:
                stages[name] = stats.summary(self.args.flux_wb)

        hold = stages.get("HOLD")
        checks = {
            "fast_frames_complete": self.fast_lost == 0,
            "phase_current_safe": self.phase_peak <= self.args.phase_limit,
            "plot_not_saturated": not any(self.saturation),
            "hold_captured": hold is not None,
        }
        if hold is not None:
            target = 120.0 if direction == 1 else -120.0
            checks["observer_direction"] = hold["we_mean_rad_s"] * target > 0.0
            checks["observer_speed_within_15pct"] = (
                abs(hold["we_mean_rad_s"] - target) <= 0.15 * abs(target)
            )

        return {
            "head": self.args.head,
            "direction": self.args.direction,
            "flux_wb": self.args.flux_wb,
            "fast_frames": self.fast_frames,
            "fast_samples": self.fast_samples,
            "fast_seq_lost": self.fast_lost,
            "phase_peak_a": self.phase_peak,
            "phase_limit_a": self.args.phase_limit,
            "saturation_count": {
                FAST_VARS[index][0]: count
                for index, count in enumerate(self.saturation)
            },
            "stages": stages,
            "checks": checks,
        }

    def print_shadow_summary(self, result):
        print("\nObserver Shadow")
        print(
            f"FAST samples={self.fast_samples} frames={self.fast_frames} "
            f"seq_lost={self.fast_lost}"
        )
        print(
            f"phase peak={self.phase_peak:.3f} A "
            f"limit={self.args.phase_limit:.3f} A"
        )
        for name in ("ACCEL", "HOLD"):
            stats = result["stages"].get(name)
            if stats is None:
                continue
            print(
                f"{name}: We_obs={stats['we_mean_rad_s']:.3f} rad/s "
                f"ripple={stats['we_ripple_rms_rad_s']:.3f}, "
                f"angle diff={stats['angle_diff_mean_deg']:.2f} deg "
                f"circular rms={stats['angle_diff_circular_rms_deg']:.2f} deg, "
                f"PLL rms={stats['pll_err_rms']:.5f}, "
                f"FluxErr={stats['flux_err_mean_pu'] * 100.0:.3f}%"
            )
        for name, passed in result["checks"].items():
            print(f"{name}: {'PASS' if passed else 'WARN'}")

    def save(self, result):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = f"observer_shadow_{self.args.direction}_{stamp}"
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f"{stem}.csv"
        json_path = output_dir / f"{stem}.json"

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time_s", "stage", *(item[0] for item in FAST_VARS)))
            writer.writerows(self.rows)

        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
            handle.write("\n")

        print(f"CSV:  {csv_path}")
        print(f"JSON: {json_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded SENSORLESS_SPEED observer Shadow capture"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--phase-limit", type=float, default=2.2)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--flux-wb", type=float, default=0.00318646245)
    parser.add_argument("--head", default="3073c88735b67147eaf68cbadbf3e9b6b4d07eda")
    parser.add_argument("--output-dir", default="build/Release")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.hold_seconds < 0.0:
        parser.error("--hold-seconds must be non-negative")
    if args.phase_limit <= 0.0:
        parser.error("--phase-limit must be positive")
    if args.vbus_seconds <= 0.0 or args.ready_timeout <= 0.0:
        parser.error("Vbus and ready timeouts must be positive")
    if args.poll_interval <= 0.0 or args.flux_wb <= 0.0:
        parser.error("poll interval and flux must be positive")
    return args


def main():
    args = parse_args()
    direction = 1 if args.direction == "forward" else 2

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

            test = ObserverShadowTest(ser, args)
            error = None
            try:
                test.configure_plot()
                test.check_vbus()
                test.run(direction)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop()

            result = test.result(direction)
            test.print_shadow_summary(result)
            test.save(result)

            if error is not None:
                raise error
            if test.fast_lost:
                raise RuntimeError(f"FAST lost {test.fast_lost} frames")

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
