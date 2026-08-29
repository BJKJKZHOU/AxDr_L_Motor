#!/usr/bin/env python3
"""Diagnose phase-current gain asymmetry during fixed-speed sensorless shadow I/F.

The motor remains on the I/F path. Ia/Ib/Ic are captured from the existing FAST
plot stream at high rate. Candidate Ib gain corrections are applied offline only;
FOC and firmware current feedback are not modified by this tool.

For every candidate gain the tool reports:
- alpha/beta covariance ellipse axis ratio;
- equivalent small-signal negative-sequence / 2fe angle-ripple estimate;
- three-phase current-closure RMS for Ia + gain*Ib + Ic.

A clear minimum away from gain=1.0 is evidence of a fixed current-channel scale
mismatch. Results and raw samples are exported to JSON.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import statistics
import struct
import sys
import time

import observer_shadow_run as shadow
import sensorless_run
import sensorless_test as base


FAST_CONFIG_ID = 15
NORMAL_CONFIG_ID = sensorless_run.NORMAL_CONFIG_ID
CURRENT_SCALE_A = 0.001
CURRENT_VARS = (
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
)
VBUS_ID = 0x0004
INV_SQRT3 = 1.0 / math.sqrt(3.0)
STAGE_IF_TO_OBS = 2


def mean(values):
    return statistics.fmean(values) if values else math.nan


def rms(values):
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else math.nan


def parse_gain_list(text):
    try:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("gain list must be comma-separated numbers") from exc
    if not values or any((not math.isfinite(value)) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("gain values must be positive finite numbers")
    return values


def ellipse_metrics(samples, ib_gain):
    alpha = []
    beta = []
    closure = []
    phase_sq = []

    for row in samples:
        ia = row["Ia"]
        ib = ib_gain * row["Ib"]
        ic = row["Ic"]
        alpha.append(ia)
        beta.append((ia + 2.0 * ib) * INV_SQRT3)
        closure.append(ia + ib + ic)
        phase_sq.extend((ia * ia, ib * ib, ic * ic))

    ma = mean(alpha)
    mb = mean(beta)
    daa = [value - ma for value in alpha]
    dbb = [value - mb for value in beta]
    cxx = mean([value * value for value in daa])
    cyy = mean([value * value for value in dbb])
    cxy = mean([a * b for a, b in zip(daa, dbb)])

    half_trace = 0.5 * (cxx + cyy)
    root = math.sqrt(max(0.0, 0.25 * (cxx - cyy) ** 2 + cxy * cxy))
    lam_max = half_trace + root
    lam_min = max(half_trace - root, 1e-20)
    axis_ratio = math.sqrt(lam_max / lam_min)

    # For a nearly circular rotating vector, (r-1)/(r+1) is the magnitude of
    # the ellipse's negative-sequence component and is also the first-order
    # amplitude of the resulting 2*theta angle ripple in radians.
    neg_ratio = (axis_ratio - 1.0) / (axis_ratio + 1.0)
    phase_rms = math.sqrt(mean(phase_sq))
    closure_rms = rms(closure)

    return {
        "ib_gain": ib_gain,
        "alpha_mean_a": ma,
        "beta_mean_a": mb,
        "cov_aa_a2": cxx,
        "cov_bb_a2": cyy,
        "cov_ab_a2": cxy,
        "axis_ratio": axis_ratio,
        "negative_sequence_ratio_est": neg_ratio,
        "angle_2fe_est_rad": neg_ratio,
        "closure_rms_a": closure_rms,
        "closure_relative": closure_rms / phase_rms if phase_rms > 0.0 else math.nan,
    }


class EllipseRun(shadow.ShadowRun):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args, stop)
        self.current_samples = []
        self.last_fast_rx = None
        self.current_trip = False

    def configure_plot(self):
        self.fast_last = None
        self.fast_lost = 0
        self.normal_last = None
        self.normal_lost = 0
        self.current_samples = []

        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(CURRENT_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id) for _, var_id in CURRENT_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes([base.FAST_MASK | base.NORMAL_MASK]))
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return
        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(CURRENT_VARS)
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
            ia = raw[start] * CURRENT_SCALE_A
            ib = raw[start + 1] * CURRENT_SCALE_A
            ic = raw[start + 2] * CURRENT_SCALE_A
            self.current_peak = max(self.current_peak, abs(ia), abs(ib), abs(ic))
            if self.current_peak > self.args.current_limit:
                self.current_trip = True
            self.current_samples.append({"Ia": ia, "Ib": ib, "Ic": ic})

    def guard(self, require_run=False):
        del require_run
        now = time.monotonic()
        if self.current_trip:
            raise RuntimeError(f"phase current exceeded {self.args.current_limit:.3f} A")
        if self.fast_lost > self.args.max_lost:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")
        if self.normal_lost > self.args.max_lost:
            raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")
        if self.monitor_started is not None and self.last_fast_rx is None and now - self.monitor_started > 0.5:
            raise RuntimeError("no FAST current data")
        if self.last_fast_rx is not None and now - self.last_fast_rx > 0.5:
            raise RuntimeError("FAST current timeout")
        if self.last_normal_rx is not None and now - self.last_normal_rx > 1.0:
            raise RuntimeError("Vbus monitor timeout")
        if self.vbus:
            vbus = self.vbus[-1]
            if not self.args.vbus_min <= vbus <= self.args.vbus_max:
                raise RuntimeError(
                    f"Vbus {vbus:.3f} V outside {self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
                )

    def collect_currents(self):
        start_count = len(self.current_samples)
        deadline = time.monotonic() + self.args.duration
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                if self.read_stage() != STAGE_IF_TO_OBS:
                    raise RuntimeError("left sensorless shadow IF_TO_OBS")
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(False)
        rows = self.current_samples[start_count:]
        if len(rows) < 1000:
            raise RuntimeError(f"too few current samples: {len(rows)}")
        return rows


def parse_args():
    parser = argparse.ArgumentParser(description="Offline Ib-gain scan from high-rate current ellipse data")
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--rs", type=float, default=6.334559)
    parser.add_argument("--ld", type=float, default=0.001564084)
    parser.add_argument("--lq", type=float, default=0.001564084)
    parser.add_argument("--flux", type=float, default=0.01513128)
    parser.add_argument("--pole-pairs", type=int, default=11)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--we", type=float, default=120.0)
    parser.add_argument("--pll-bw-hz", type=float, default=20.0)
    parser.add_argument("--ib-gain", type=parse_gain_list,
                        default=[0.97, 0.98, 0.99, 1.00, 1.01, 1.02, 1.03],
                        help="comma-separated offline Ib gain candidates")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)

    # Fields consumed by SensorlessRun/ShadowRun construction.
    parser.add_argument("--pll-window", type=float, default=0.2)
    parser.add_argument("--speed-error-time", type=float, default=1.0)
    parser.add_argument("--pll-rms-limit", type=float, default=0.08)
    parser.add_argument("--we-tolerance", type=float, default=20.0)
    parser.add_argument("--we-relative-tolerance", type=float, default=0.03)
    parser.add_argument("--voltage-util-limit", type=float, default=0.985)
    parser.add_argument("--ramp-step", type=float, default=0.5)
    parser.add_argument("--ramp-interval", type=float, default=0.02)
    parser.add_argument("--target-timeout", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--diag-interval", type=float, default=0.02)
    parser.add_argument("--rpm", type=float, default=1.0, help=argparse.SUPPRESS)
    parser.add_argument("--direction", default="forward", help=argparse.SUPPRESS)

    args = parser.parse_args()
    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.pole_pairs <= 0 or args.current_limit <= 0.0 or args.we <= 0.0 or args.duration <= 0.0:
        parser.error("pole-pairs/current-limit/we/duration must be positive")
    if not 1.0 <= args.pll_bw_hz <= 200.0:
        parser.error("--pll-bw-hz must be within 1 .. 200 Hz")
    if any(value < 0.90 or value > 1.10 for value in args.ib_gain):
        parser.error("--ib-gain candidates must be within 0.90 .. 1.10")
    return args


def main():
    args = parse_args()
    stop = sensorless_run.StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    started = datetime.now().astimezone()
    log_dir = Path(__file__).resolve().parents[1] / "build" / "Release"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"current_ellipse_{started.strftime('%Y%m%d_%H%M%S')}.json"
    wm = args.we / args.pole_pairs
    log = {
        "test": "current_ellipse_gain_scan",
        "timestamp": started.isoformat(),
        "success": False,
        "error": None,
        "config": {
            "we_if_rad_s": args.we,
            "wm_rad_s": wm,
            "pll_bw_hz": args.pll_bw_hz,
            "duration_s": args.duration,
            "ib_gain_candidates": args.ib_gain,
            "motor": {
                "pole_pairs": args.pole_pairs,
                "rs_ohm": args.rs,
                "ld_h": args.ld,
                "lq_h": args.lq,
                "flux_wb": args.flux,
            },
        },
        "vbus": None,
        "results": [],
        "best_axis_ratio_gain": None,
        "best_closure_gain": None,
        "raw_current_samples": [],
        "guard": None,
    }

    test = None
    error = None
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = EllipseRun(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.motor_para_set()
                test.current_limit_set()
                test.pll_bw_set(args.pll_bw_hz)
                test.shadow_set(True, quiet=True)
                test.configure_plot()
                test.check_vbus()

                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([base.MODE_SENSORLESS_SPEED]))
                test.speed_set(wm)
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print(
                    f"Current ellipse test: We={args.we:.1f} rad/s, PLL BW={args.pll_bw_hz:.1f} Hz"
                )
                print("ALIGN -> I/F -> shadow; collecting high-rate Ia/Ib/Ic")
                test.wait_shadow()
                rows = test.collect_currents()
                log["raw_current_samples"] = rows

                results = [ellipse_metrics(rows, gain) for gain in args.ib_gain]
                log["results"] = results
                best_axis = min(results, key=lambda row: abs(row["axis_ratio"] - 1.0))
                best_close = min(results, key=lambda row: row["closure_rms_a"])
                log["best_axis_ratio_gain"] = best_axis["ib_gain"]
                log["best_closure_gain"] = best_close["ib_gain"]

                print("\nOffline Ib gain scan")
                print("  gain   axis ratio   neg/2fe est   closure RMS   closure rel")
                for row in results:
                    print(
                        f"  {row['ib_gain']:5.3f}    {row['axis_ratio']:8.5f}    "
                        f"{row['negative_sequence_ratio_est']:10.5f}    "
                        f"{row['closure_rms_a']:10.5f} A   {100.0 * row['closure_relative']:7.3f}%"
                    )
                print(
                    f"Best ellipse gain={best_axis['ib_gain']:.3f}; "
                    f"best three-phase closure gain={best_close['ib_gain']:.3f}"
                )
                log["success"] = True
            except (TimeoutError, RuntimeError) as exc:
                error = exc
                log["error"] = str(exc)
            finally:
                try:
                    test.stop_all(False)
                except (TimeoutError, RuntimeError):
                    pass
                try:
                    test.shadow_set(False, quiet=True)
                except (TimeoutError, RuntimeError):
                    pass

            if test.vbus:
                values = list(test.vbus)
                log["vbus"] = {"mean_v": mean(values), "min_v": min(values), "max_v": max(values)}
            log["guard"] = {
                "peak_current_a": test.current_peak,
                "fast_lost": test.fast_lost,
                "normal_lost": test.normal_lost,
            }
    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
        error = exc
        log["error"] = str(exc)
    finally:
        with log_path.open("w", encoding="utf-8") as stream:
            json.dump(log, stream, indent=2, ensure_ascii=False, allow_nan=False)
        print(f"Log: {log_path}")

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
