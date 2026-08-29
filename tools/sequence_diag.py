#!/usr/bin/env python3
"""Diagnose fixed alpha/beta asymmetry during sensorless shadow I/F.

The motor remains on the I/F path. One synchronized FAST stream captures:
Theta_e, Ia/Ib/Ic, Ualpha/Ubeta command, and observer PsiAlpha/PsiBeta.

The host then:
- fits Kb/Kc (Ka fixed at 1) to minimize Ia + Kb*Ib + Kc*Ic;
- extracts positive/negative sequence components of current, commanded voltage,
  and observer flux relative to Theta_e;
- exports raw samples and all metrics to JSON.

Optional Ia/Ib gains are written to firmware RAM before the run and read back.
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


CTRL_CURRENT_GAIN_SET = 0x0C
CTRL_CURRENT_GAIN_GET = 0x0D
FAST_CONFIG_ID = 16
NORMAL_CONFIG_ID = sensorless_run.NORMAL_CONFIG_ID
STAGE_IF_TO_OBS = 2
INV_SQRT3 = 1.0 / math.sqrt(3.0)

FAST_VARS = (
    ("Theta_e", 0x0014, 0.0002),
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
    ("Ic", 0x0003, 0.001),
    ("Ualpha", 0x0015, 0.001),
    ("Ubeta", 0x0016, 0.001),
    ("PsiAlpha", 0x0024, 1.0e-6),
    ("PsiBeta", 0x0025, 1.0e-6),
)
VBUS_ID = 0x0004


def mean(values):
    return statistics.fmean(values) if values else math.nan


def rms(values):
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else math.nan


def complex_mean(values):
    if not values:
        return complex(math.nan, math.nan)
    return sum(values, 0j) / len(values)


def sequence_metrics(rows, alpha_key, beta_key):
    positive = []
    negative = []
    magnitudes = []

    for row in rows:
        theta = row["Theta_e"]
        z = complex(row[alpha_key], row[beta_key])
        rot_pos = complex(math.cos(theta), -math.sin(theta))
        rot_neg = complex(math.cos(theta), math.sin(theta))
        positive.append(z * rot_pos)
        negative.append(z * rot_neg)
        magnitudes.append(abs(z))

    pos = complex_mean(positive)
    neg = complex_mean(negative)
    pos_mag = abs(pos)
    neg_mag = abs(neg)
    ratio = neg_mag / pos_mag if pos_mag > 1e-12 else math.nan

    return {
        "positive_real": pos.real,
        "positive_imag": pos.imag,
        "positive_mag": pos_mag,
        "negative_real": neg.real,
        "negative_imag": neg.imag,
        "negative_mag": neg_mag,
        "negative_ratio": ratio,
        "angle_2fe_est_rad": ratio,
        "vector_mag_mean": mean(magnitudes),
        "vector_mag_rms": rms(magnitudes),
    }


def current_sequence_metrics(rows):
    mapped = []
    for row in rows:
        ia = row["Ia"]
        ib = row["Ib"]
        mapped.append({
            "Theta_e": row["Theta_e"],
            "Ialpha": ia,
            "Ibeta": (ia + 2.0 * ib) * INV_SQRT3,
        })
    return sequence_metrics(mapped, "Ialpha", "Ibeta")


def closure_fit(rows):
    sii = sic = scc = sai = sac = 0.0
    raw_closure = []
    ia_values = []
    ib_values = []
    ic_values = []

    for row in rows:
        ia = row["Ia"]
        ib = row["Ib"]
        ic = row["Ic"]
        sii += ib * ib
        sic += ib * ic
        scc += ic * ic
        sai += ia * ib
        sac += ia * ic
        raw_closure.append(ia + ib + ic)
        ia_values.append(ia)
        ib_values.append(ib)
        ic_values.append(ic)

    det = sii * scc - sic * sic
    if abs(det) < 1e-18:
        raise RuntimeError("three-phase closure fit is singular")

    kb = (-sai * scc + sac * sic) / det
    kc = (-sac * sii + sai * sic) / det
    fitted = [row["Ia"] + kb * row["Ib"] + kc * row["Ic"] for row in rows]
    phase_rms = math.sqrt(mean([
        value * value
        for triplet in zip(ia_values, ib_values, ic_values)
        for value in triplet
    ]))

    return {
        "kb": kb,
        "kc": kc,
        "ia_rms_a": rms(ia_values),
        "ib_rms_a": rms(ib_values),
        "ic_rms_a": rms(ic_values),
        "raw_closure_rms_a": rms(raw_closure),
        "raw_closure_relative": rms(raw_closure) / phase_rms if phase_rms > 0.0 else math.nan,
        "fitted_closure_rms_a": rms(fitted),
        "fitted_closure_relative": rms(fitted) / phase_rms if phase_rms > 0.0 else math.nan,
    }


class SequenceRun(shadow.ShadowRun):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args, stop)
        self.samples = []
        self.last_fast_rx = None
        self.current_trip = False
        self.fast_saturation = [0] * len(FAST_VARS)
        self.current_gain = None

    def current_gain_set(self):
        self.request(
            base.MSG_CONTROL,
            CTRL_CURRENT_GAIN_SET,
            struct.pack("<ff", self.args.ia_gain, self.args.ib_gain),
        )
        data = self.request(base.MSG_CONTROL, CTRL_CURRENT_GAIN_GET)
        if len(data) != 8:
            raise RuntimeError(f"invalid current gain readback length: {len(data)}")
        ia_gain, ib_gain = struct.unpack("<ff", data)
        if abs(ia_gain - self.args.ia_gain) > 1e-5 or abs(ib_gain - self.args.ib_gain) > 1e-5:
            raise RuntimeError(
                f"current gain readback mismatch: Ia={ia_gain:.6f}, Ib={ib_gain:.6f}"
            )
        self.current_gain = {"ia": ia_gain, "ib": ib_gain}
        print(f"Current gain RAM: Ia={ia_gain:.5f}, Ib={ib_gain:.5f}")

    def configure_plot(self):
        self.fast_last = None
        self.fast_lost = 0
        self.normal_last = None
        self.normal_lost = 0
        self.samples = []
        self.fast_saturation = [0] * len(FAST_VARS)

        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes([base.FAST_MASK | base.NORMAL_MASK]))
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return
        seq_num, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        if len(payload) != 4 + sample_count * count * 2:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq_num - expected) & 0xFFFF
        self.fast_last = seq_num
        self.fast_frames += 1
        self.fast_samples += sample_count
        self.last_fast_rx = time.monotonic()

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for sample in range(sample_count):
            start = sample * count
            row = {}
            for index, (name, _, scale) in enumerate(FAST_VARS):
                code = raw[start + index]
                if code in (-32768, 32767):
                    self.fast_saturation[index] += 1
                row[name] = code * scale
            self.current_peak = max(
                self.current_peak,
                abs(row["Ia"]), abs(row["Ib"]), abs(row["Ic"]),
            )
            if self.current_peak > self.args.current_limit:
                self.current_trip = True
            self.samples.append(row)

    def guard(self, require_run=False):
        del require_run
        now = time.monotonic()
        if self.current_trip:
            raise RuntimeError(f"phase current exceeded {self.args.current_limit:.3f} A")
        if any(self.fast_saturation):
            names = [FAST_VARS[i][0] for i, count in enumerate(self.fast_saturation) if count]
            raise RuntimeError(f"FAST quantizer saturated: {names}")
        if self.fast_lost > self.args.max_lost:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")
        if self.normal_lost > self.args.max_lost:
            raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")
        if self.monitor_started is not None and self.last_fast_rx is None and now - self.monitor_started > 0.5:
            raise RuntimeError("no FAST diagnostic data")
        if self.last_fast_rx is not None and now - self.last_fast_rx > 0.5:
            raise RuntimeError("FAST diagnostic timeout")
        if self.last_normal_rx is not None and now - self.last_normal_rx > 1.0:
            raise RuntimeError("Vbus monitor timeout")
        if self.vbus:
            vbus = self.vbus[-1]
            if not self.args.vbus_min <= vbus <= self.args.vbus_max:
                raise RuntimeError(
                    f"Vbus {vbus:.3f} V outside {self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
                )

    def collect(self):
        start_count = len(self.samples)
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
        rows = self.samples[start_count:]
        if len(rows) < 1000:
            raise RuntimeError(f"too few synchronized samples: {len(rows)}")
        return rows


def parse_args():
    parser = argparse.ArgumentParser(description="Synchronized I/Ucmd/Psi sequence diagnostics")
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--rs", type=float, default=6.334559)
    parser.add_argument("--ld", type=float, default=0.001564084)
    parser.add_argument("--lq", type=float, default=0.001564084)
    parser.add_argument("--flux", type=float, default=0.01513128)
    parser.add_argument("--pole-pairs", type=int, default=11)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--ia-gain", type=float, default=1.0)
    parser.add_argument("--ib-gain", type=float, default=1.0)
    parser.add_argument("--we", type=float, default=120.0, help="signed electrical speed target in rad/s")
    parser.add_argument("--pll-bw-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)

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
    if args.pole_pairs <= 0 or args.current_limit <= 0.0 or args.we == 0.0 or args.duration <= 0.0:
        parser.error("pole-pairs/current-limit/duration must be positive and we must be non-zero")
    if not 1.0 <= args.pll_bw_hz <= 200.0:
        parser.error("--pll-bw-hz must be within 1 .. 200 Hz")
    if not 0.8 <= args.ia_gain <= 1.2 or not 0.8 <= args.ib_gain <= 1.2:
        parser.error("--ia-gain/--ib-gain must be within 0.8 .. 1.2")
    return args


def main():
    args = parse_args()
    stop = sensorless_run.StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    started = datetime.now().astimezone()
    log_dir = Path(__file__).resolve().parents[1] / "build" / "Release"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"sequence_diag_{started.strftime('%Y%m%d_%H%M%S')}.json"
    wm = args.we / args.pole_pairs
    log = {
        "test": "alpha_beta_sequence_diag",
        "timestamp": started.isoformat(),
        "success": False,
        "error": None,
        "config": {
            "we_if_rad_s": args.we,
            "wm_rad_s": wm,
            "direction": "forward" if args.we > 0.0 else "reverse",
            "pll_bw_hz": args.pll_bw_hz,
            "duration_s": args.duration,
            "current_gain_requested": {"ia": args.ia_gain, "ib": args.ib_gain},
            "motor": {
                "pole_pairs": args.pole_pairs,
                "rs_ohm": args.rs,
                "ld_h": args.ld,
                "lq_h": args.lq,
                "flux_wb": args.flux,
            },
        },
        "current_gain_readback": None,
        "vbus": None,
        "closure_fit": None,
        "sequence": None,
        "raw_samples": [],
        "guard": None,
    }

    test = None
    error = None
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = SequenceRun(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.motor_para_set()
                test.current_limit_set()
                test.current_gain_set()
                log["current_gain_readback"] = test.current_gain
                test.pll_bw_set(args.pll_bw_hz)
                test.shadow_set(True, quiet=True)
                test.configure_plot()
                test.check_vbus()

                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([base.MODE_SENSORLESS_SPEED]))
                test.speed_set(wm)
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print(
                    f"Sequence diagnostic: We={args.we:.1f} rad/s, PLL BW={args.pll_bw_hz:.1f} Hz"
                )
                print("ALIGN -> I/F -> shadow; collecting Theta/I/Ucmd/Psi at FAST rate")
                test.wait_shadow()
                rows = test.collect()
                log["raw_samples"] = rows

                fit = closure_fit(rows)
                current = current_sequence_metrics(rows)
                voltage = sequence_metrics(rows, "Ualpha", "Ubeta")
                flux = sequence_metrics(rows, "PsiAlpha", "PsiBeta")
                log["closure_fit"] = fit
                log["sequence"] = {
                    "current_alpha_beta": current,
                    "voltage_command_alpha_beta": voltage,
                    "observer_flux_alpha_beta": flux,
                }

                print("\nThree-phase closure fit (Ka fixed at 1)")
                print(
                    f"  phase RMS: Ia={fit['ia_rms_a']:.4f} A "
                    f"Ib={fit['ib_rms_a']:.4f} A Ic={fit['ic_rms_a']:.4f} A"
                )
                print(
                    f"  raw closure RMS={fit['raw_closure_rms_a']:.5f} A "
                    f"({100.0 * fit['raw_closure_relative']:.2f}%)"
                )
                print(f"  fitted Kb={fit['kb']:.5f}, Kc={fit['kc']:.5f}")
                print(
                    f"  fitted closure RMS={fit['fitted_closure_rms_a']:.5f} A "
                    f"({100.0 * fit['fitted_closure_relative']:.2f}%)"
                )

                print("\nSynchronous positive / negative sequence")
                print("  vector       positive mag    negative mag    neg/pos    2fe est")
                for name, metrics in (
                    ("I alpha-beta", current),
                    ("Ucmd a-b", voltage),
                    ("Psi a-b", flux),
                ):
                    print(
                        f"  {name:11s} {metrics['positive_mag']:12.6f} "
                        f"{metrics['negative_mag']:14.6f} "
                        f"{metrics['negative_ratio']:10.5f} "
                        f"{metrics['angle_2fe_est_rad']:10.5f} rad"
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
                time.sleep(0.05)
                try:
                    test.shadow_set(False, quiet=True)
                except (TimeoutError, RuntimeError):
                    pass

            if test.vbus:
                values = list(test.vbus)
                log["vbus"] = {
                    "mean_v": mean(values),
                    "min_v": min(values),
                    "max_v": max(values),
                }
            log["guard"] = {
                "peak_current_a": test.current_peak,
                "fast_lost": test.fast_lost,
                "normal_lost": test.normal_lost,
                "fast_saturation": test.fast_saturation,
            }

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
        error = exc
        log["error"] = str(exc)
    finally:
        with log_path.open("w", encoding="utf-8") as handle:
            json.dump(log, handle, indent=2, ensure_ascii=False, allow_nan=False)
        print(f"Log: {log_path}")

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
