#!/usr/bin/env python3
"""Single-window current-loop 2fe diagnostics during sensorless shadow I/F.

Captures one synchronized 5 kHz FAST window:
Theta_e, IdErr/IqErr (50 uA/LSB), Id/Iq PI integrators, Ud/Uq, IqRef.
Ualpha/Ubeta are reconstructed from Ud/Uq/Theta_e so the complete
Error -> P/Integrator -> dq output -> alpha/beta negative-sequence chain is
analyzed from the same samples.
"""

import json
import math
from datetime import datetime
from pathlib import Path
import signal
import statistics
import struct
import sys
import time

import sequence_diag as seq
import sensorless_run
import sensorless_test as base

META_MASK = 0x80
DECIMATE4_MASK = 0x40
CONFIG_ID = META_MASK | DECIMATE4_MASK | 0x13
BLOCK_SAMPLES = 20
CUR_BW_HZ = 1000.0
EXPECTED_DECIMATION = 4

FAST_VARS = (
    ("Theta_e", 0x0014, 0.0002),
    ("IdErr", 0x001B, 0.00005),
    ("IqErr", 0x001C, 0.00005),
    ("IdInt", 0x0019, 0.001),
    ("IqInt", 0x001A, 0.001),
    ("Ud", 0x0012, 0.001),
    ("Uq", 0x0013, 0.001),
    ("IqRef", 0x0018, 0.001),
)


def mean(values):
    return statistics.fmean(values) if values else math.nan


def rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values)) if values else math.nan


def wrap_deg(value):
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def scalar_2fe(rows, key):
    values = [row[key] for row in rows]
    dc = mean(values)
    coeff = sum(
        (row[key] - dc)
        * complex(math.cos(2.0 * row["Theta_e"]), -math.sin(2.0 * row["Theta_e"]))
        for row in rows
    ) / len(rows)
    return {
        "mean": dc,
        "rms": rms(values),
        "coeff_real": coeff.real,
        "coeff_imag": coeff.imag,
        "peak": 2.0 * abs(coeff),
        "phase_deg": math.degrees(math.atan2(coeff.imag, coeff.real)),
    }


def continuity(rows):
    decimation = rows[0]["decimation"]
    gaps = []
    for prev, cur in zip(rows, rows[1:]):
        expected = prev["control_tick"] + decimation
        if cur["control_tick"] != expected:
            delta = cur["control_tick"] - expected
            gaps.append({
                "from": prev["control_tick"],
                "to": cur["control_tick"],
                "missing_control_ticks": delta,
                "missing_plot_samples": delta // decimation if delta > 0 else 0,
            })
    return {
        "first_tick": rows[0]["control_tick"],
        "last_tick": rows[-1]["control_tick"],
        "decimation": decimation,
        "gap_count": len(gaps),
        "missing_control_ticks": sum(g["missing_control_ticks"] for g in gaps),
        "missing_plot_samples": sum(g["missing_plot_samples"] for g in gaps),
        "fast_drop_first": rows[0]["fast_drop"],
        "fast_drop_last": rows[-1]["fast_drop"],
        "fast_drop_delta": rows[-1]["fast_drop"] - rows[0]["fast_drop"],
        "gaps": gaps,
    }


class CurrentLoopRun(seq.SequenceRun):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args, stop)
        self.samples = []
        self.last_fast_rx = None
        self.current_trip = False
        self.fast_saturation = [0] * len(FAST_VARS)

    def configure_plot(self):
        self.fast_last = None
        self.fast_lost = 0
        self.normal_last = None
        self.normal_lost = 0
        self.samples = []
        self.fast_saturation = [0] * len(FAST_VARS)

        fast_data = bytes([base.FAST_GROUP, CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, sensorless_run.NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", seq.VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes([base.FAST_MASK | base.NORMAL_MASK]))
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        if len(payload) < 14 or payload[2] != CONFIG_ID:
            return

        (frame_seq,) = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        first_tick, fast_drop = struct.unpack_from("<II", payload, 4)
        first_block_pos = payload[12]
        decimation = payload[13]
        if decimation != EXPECTED_DECIMATION:
            raise RuntimeError(f"unexpected FAST decimation: {decimation}")
        count = len(FAST_VARS)
        if len(payload) != 14 + sample_count * count * 2:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (frame_seq - expected) & 0xFFFF
        self.fast_last = frame_seq
        self.fast_frames += 1
        self.fast_samples += sample_count
        self.last_fast_rx = time.monotonic()

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 14)
        for sample in range(sample_count):
            start = sample * count
            row = {
                "control_tick": first_tick + sample * decimation,
                "block_pos": (first_block_pos + sample) % BLOCK_SAMPLES,
                "fast_drop": fast_drop,
                "decimation": decimation,
            }
            for index, (name, _, scale) in enumerate(FAST_VARS):
                code = raw[start + index]
                if code in (-32768, 32767):
                    self.fast_saturation[index] += 1
                row[name] = code * scale
            self.samples.append(row)

    def collect(self):
        start_count = len(self.samples)
        deadline = time.monotonic() + self.args.duration
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                if self.read_stage() != seq.STAGE_IF_TO_OBS:
                    raise RuntimeError("left sensorless shadow IF_TO_OBS")
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(False)
        rows = self.samples[start_count:]
        if len(rows) < 1000:
            raise RuntimeError(f"too few synchronized samples: {len(rows)}")
        return rows


def print_scalar(name, metrics, unit):
    print(
        f"  {name:8s} mean={metrics['mean']:+.6f} {unit:<2s} "
        f"2fe_peak={metrics['peak']:.6f} {unit:<2s} phase={metrics['phase_deg']:+7.2f} deg"
    )


def main():
    args = seq.parse_args()
    stop = sensorless_run.StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    started = datetime.now().astimezone()
    log_dir = Path(__file__).resolve().parents[1] / "build" / "Release"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"current_loop_diag_{started.strftime('%Y%m%d_%H%M%S')}.json"
    log = {"timestamp": started.isoformat(), "success": False}
    error = None
    test = None

    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = CurrentLoopRun(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.motor_para_set()
                test.current_limit_set()
                test.current_gain_set()
                test.pll_bw_set(args.pll_bw_hz)
                test.shadow_set(True, quiet=True)
                test.configure_plot()
                test.check_vbus()

                wm = args.we / args.pole_pairs
                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([base.MODE_SENSORLESS_SPEED]))
                test.speed_set(wm)
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print(f"Current-loop diagnostic: We={args.we:.1f} rad/s, FAST=5 kHz (decimation 4)")
                print("ALIGN -> I/F -> shadow; one synchronized FAST window")
                test.wait_shadow()
                print(f"Collecting Error/Integrator/Udq for {args.duration:.2f} s")
                rows = test.collect()
                cont = continuity(rows)

                wc = 2.0 * math.pi * CUR_BW_HZ
                kp_d = args.ld * wc
                kp_q = args.lq * wc
                ki = args.rs * wc

                mapped_u = []
                for row in rows:
                    row["IdP"] = kp_d * row["IdErr"]
                    row["IqP"] = kp_q * row["IqErr"]
                    theta = row["Theta_e"]
                    c = math.cos(theta)
                    s = math.sin(theta)
                    row["Ualpha"] = row["Ud"] * c - row["Uq"] * s
                    row["Ubeta"] = row["Ud"] * s + row["Uq"] * c
                    mapped_u.append(row)

                metrics = {
                    key: scalar_2fe(rows, key)
                    for key in ("IdErr", "IqErr", "IdP", "IqP", "IdInt", "IqInt", "Ud", "Uq", "IqRef")
                }
                u_sequence = seq.sequence_metrics(mapped_u, "Ualpha", "Ubeta")

                int_theory = {}
                for axis in ("Id", "Iq"):
                    err = metrics[f"{axis}Err"]
                    integ = metrics[f"{axis}Int"]
                    pred_peak = ki * err["peak"] / (2.0 * args.we)
                    phase_delta = wrap_deg(integ["phase_deg"] - err["phase_deg"])
                    int_theory[axis] = {
                        "predicted_peak_v": pred_peak,
                        "measured_peak_v": integ["peak"],
                        "peak_ratio_measured_over_predicted": integ["peak"] / pred_peak if pred_peak > 1e-12 else math.nan,
                        "phase_delta_deg": phase_delta,
                        "ideal_phase_delta_deg": -90.0,
                    }

                print("\nCurrent error / PI / final dq output")
                for key, unit in (
                    ("IdErr", "A"), ("IqErr", "A"), ("IdP", "V"), ("IqP", "V"),
                    ("IdInt", "V"), ("IqInt", "V"), ("Ud", "V"), ("Uq", "V"), ("IqRef", "A"),
                ):
                    print_scalar(key, metrics[key], unit)

                print("\nIntegrator consistency at 2fe")
                for axis in ("Id", "Iq"):
                    item = int_theory[axis]
                    print(
                        f"  {axis}: predicted={item['predicted_peak_v']:.6f} V "
                        f"measured={item['measured_peak_v']:.6f} V "
                        f"ratio={item['peak_ratio_measured_over_predicted']:.3f} "
                        f"phase_delta={item['phase_delta_deg']:+.2f} deg (ideal -90 deg)"
                    )

                print("\nReconstructed Ucmd alpha-beta")
                print(
                    f"  positive={u_sequence['positive_mag']:.6f} V "
                    f"negative={u_sequence['negative_mag']:.6f} V "
                    f"neg/pos={u_sequence['negative_ratio']:.5f}"
                )

                print("\nFAST continuity")
                print(
                    f"  tick={cont['first_tick']}..{cont['last_tick']} decim={cont['decimation']} "
                    f"gaps={cont['gap_count']} missing_plot={cont['missing_plot_samples']} "
                    f"fast_drop_delta={cont['fast_drop_delta']}"
                )

                log = {
                    "timestamp": started.isoformat(),
                    "success": True,
                    "config": {
                        "we_rad_s": args.we,
                        "rs_ohm": args.rs,
                        "ld_h": args.ld,
                        "lq_h": args.lq,
                        "current_bw_hz": CUR_BW_HZ,
                        "kp_d": kp_d,
                        "kp_q": kp_q,
                        "ki": ki,
                        "fast_decimation": EXPECTED_DECIMATION,
                        "error_scale_a": 0.00005,
                    },
                    "continuity": cont,
                    "metrics": metrics,
                    "integrator_theory": int_theory,
                    "u_sequence": u_sequence,
                    "raw_samples": rows,
                }

            except (TimeoutError, RuntimeError) as exc:
                error = exc
                log = {"timestamp": started.isoformat(), "success": False, "error": str(exc)}
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

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
        error = exc
        log = {"timestamp": started.isoformat(), "success": False, "error": str(exc)}
    finally:
        with log_path.open("w", encoding="utf-8") as handle:
            json.dump(log, handle, indent=2, ensure_ascii=False, allow_nan=False)
        print(f"Log: {log_path}")

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
