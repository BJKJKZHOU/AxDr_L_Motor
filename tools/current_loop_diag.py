#!/usr/bin/env python3
"""Diagnose current-loop 2fe sources during sensorless shadow I/F.

One motor run is kept alive while two FAST windows are captured:
  A: Theta, Id/Iq, Id/Iq references, Id/Iq PI integrators.
  B: Theta, Id/Iq, Ud/Uq, Ualpha/Ubeta.

FAST Config_ID bit7 enables the extended firmware header carrying the first
control tick and Plot_Fast_Drop count. Bit6 requests 4:1 decimation, reducing
20 kHz FAST transport to 5 kHz while preserving ample resolution for 2fe.
Raw rows keep global control_tick and true FAST block position so offline
analysis can reject non-contiguous sample pairs exactly.
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
CONFIG_A = META_MASK | DECIMATE4_MASK | 0x11
CONFIG_B = META_MASK | DECIMATE4_MASK | 0x12
BLOCK_SAMPLES = 20
CUR_BW_HZ = 1000.0
EXPECTED_DECIMATION = 4

FAST_A = (
    ("Theta_e", 0x0014, 0.0002),
    ("Id", 0x0010, 0.001),
    ("Iq", 0x0011, 0.001),
    ("IdRef", 0x0017, 0.001),
    ("IqRef", 0x0018, 0.001),
    ("IdInt", 0x0019, 0.001),
    ("IqInt", 0x001A, 0.001),
)

FAST_B = (
    ("Theta_e", 0x0014, 0.0002),
    ("Id", 0x0010, 0.001),
    ("Iq", 0x0011, 0.001),
    ("Ud", 0x0012, 0.001),
    ("Uq", 0x0013, 0.001),
    ("Ualpha", 0x0015, 0.001),
    ("Ubeta", 0x0016, 0.001),
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
    coeff = 0j
    for row in rows:
        angle = 2.0 * row["Theta_e"]
        coeff += (row[key] - dc) * complex(math.cos(angle), -math.sin(angle))
    coeff /= len(rows)
    peak = 2.0 * abs(coeff)
    return {
        "mean": dc,
        "rms": rms(values),
        "coeff_real": coeff.real,
        "coeff_imag": coeff.imag,
        "peak": peak,
        "phase_deg": math.degrees(math.atan2(coeff.imag, coeff.real)),
    }


def continuity(rows):
    expected_step = rows[0]["decimation"]
    gaps = []
    for prev, cur in zip(rows, rows[1:]):
        expected = prev["control_tick"] + expected_step
        if cur["control_tick"] != expected:
            delta = cur["control_tick"] - expected
            gaps.append(
                {
                    "from": prev["control_tick"],
                    "to": cur["control_tick"],
                    "missing_control_ticks": delta,
                    "missing_plot_samples": delta // expected_step if delta > 0 else 0,
                }
            )
    return {
        "first_tick": rows[0]["control_tick"],
        "last_tick": rows[-1]["control_tick"],
        "decimation": expected_step,
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
        self.active_vars = FAST_A
        self.active_config = CONFIG_A
        self.samples = []
        self.last_fast_rx = None
        self.current_trip = False
        self.fast_saturation = [0] * len(self.active_vars)

    def configure_plot(self):
        self._configure_fast(FAST_A, CONFIG_A, start_normal=True)

    def _configure_fast(self, variables, config_id, start_normal=False):
        self.active_vars = variables
        self.active_config = config_id
        self.fast_last = None
        self.fast_lost = 0
        self.samples = []
        self.fast_saturation = [0] * len(variables)
        self.last_fast_rx = None

        fast_data = bytes([base.FAST_GROUP, config_id, len(variables)])
        fast_data += b"".join(struct.pack("<H", var_id) for _, var_id, _ in variables)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        if start_normal:
            self.normal_last = None
            self.normal_lost = 0
            normal_data = bytes([base.NORMAL_GROUP, sensorless_run.NORMAL_CONFIG_ID, 1])
            normal_data += struct.pack("<H", seq.VBUS_ID)
            self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
            mask = base.FAST_MASK | base.NORMAL_MASK
        else:
            mask = base.FAST_MASK

        self.request(base.MSG_PLOT, base.PLOT_START, bytes([mask]))
        self.monitor_started = time.monotonic()

    def switch_fast(self, variables, config_id):
        self.request(base.MSG_PLOT, base.PLOT_STOP, bytes([base.FAST_MASK]))
        self._configure_fast(variables, config_id, start_normal=False)

    def process_fast(self, payload):
        if len(payload) < 14 or payload[2] != self.active_config:
            return

        (frame_seq,) = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        first_tick, fast_drop = struct.unpack_from("<II", payload, 4)
        first_block_pos = payload[12]
        decimation = payload[13]
        count = len(self.active_vars)
        if decimation != EXPECTED_DECIMATION:
            raise RuntimeError(f"unexpected FAST decimation: {decimation}")
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
            tick = first_tick + sample * decimation
            row = {
                "control_tick": tick,
                "block_pos": (first_block_pos + sample) % BLOCK_SAMPLES,
                "fast_drop": fast_drop,
                "decimation": decimation,
            }
            for index, (name, _, scale) in enumerate(self.active_vars):
                code = raw[start + index]
                if code in (-32768, 32767):
                    self.fast_saturation[index] += 1
                row[name] = code * scale
            if "Id" in row and "Iq" in row:
                self.current_peak = max(self.current_peak, abs(row["Id"]), abs(row["Iq"]))
            self.samples.append(row)

    def collect_window(self, seconds):
        start_count = len(self.samples)
        deadline = time.monotonic() + seconds
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
                print("ALIGN -> I/F -> shadow; two FAST windows in the same motor run")
                test.wait_shadow()

                print(f"Window A: Ref/Fbk/PI integrators for {args.duration:.2f} s")
                rows_a = test.collect_window(args.duration)
                cont_a = continuity(rows_a)

                test.switch_fast(FAST_B, CONFIG_B)
                time.sleep(0.05)
                print(f"Window B: dq/alpha-beta outputs for {args.duration:.2f} s")
                rows_b = test.collect_window(args.duration)
                cont_b = continuity(rows_b)

                wc = 2.0 * math.pi * CUR_BW_HZ
                kp_d = args.ld * wc
                kp_q = args.lq * wc
                ki = args.rs * wc
                for row in rows_a:
                    row["IdErr"] = row["IdRef"] - row["Id"]
                    row["IqErr"] = row["IqRef"] - row["Iq"]
                    row["IdP"] = kp_d * row["IdErr"]
                    row["IqP"] = kp_q * row["IqErr"]

                a_metrics = {
                    key: scalar_2fe(rows_a, key)
                    for key in ("Id", "Iq", "IdErr", "IqErr", "IdP", "IqP", "IdInt", "IqInt")
                }
                b_metrics = {key: scalar_2fe(rows_b, key) for key in ("Id", "Iq", "Ud", "Uq")}
                u_sequence = seq.sequence_metrics(rows_b, "Ualpha", "Ubeta")

                int_theory = {}
                for axis in ("Id", "Iq"):
                    err = a_metrics[f"{axis}Err"]
                    integ = a_metrics[f"{axis}Int"]
                    pred_peak = ki * err["peak"] / (2.0 * args.we)
                    phase_delta = wrap_deg(integ["phase_deg"] - err["phase_deg"])
                    int_theory[axis] = {
                        "predicted_peak_v": pred_peak,
                        "measured_peak_v": integ["peak"],
                        "peak_ratio_measured_over_predicted": integ["peak"] / pred_peak if pred_peak > 1e-12 else math.nan,
                        "phase_delta_deg": phase_delta,
                        "ideal_phase_delta_deg": -90.0,
                    }

                print("\nWindow A: current error / PI decomposition")
                for key, unit in (
                    ("Id", "A"), ("Iq", "A"), ("IdErr", "A"), ("IqErr", "A"),
                    ("IdP", "V"), ("IqP", "V"), ("IdInt", "V"), ("IqInt", "V"),
                ):
                    print_scalar(key, a_metrics[key], unit)

                print("\nIntegrator consistency at 2fe")
                for axis in ("Id", "Iq"):
                    item = int_theory[axis]
                    print(
                        f"  {axis}: predicted={item['predicted_peak_v']:.6f} V "
                        f"measured={item['measured_peak_v']:.6f} V "
                        f"ratio={item['peak_ratio_measured_over_predicted']:.3f} "
                        f"phase_delta={item['phase_delta_deg']:+.2f} deg (ideal -90 deg)"
                    )

                print("\nWindow B: final dq output")
                for key, unit in (("Id", "A"), ("Iq", "A"), ("Ud", "V"), ("Uq", "V")):
                    print_scalar(key, b_metrics[key], unit)
                print(
                    f"  Ucmd alpha-beta: positive={u_sequence['positive_mag']:.6f} V "
                    f"negative={u_sequence['negative_mag']:.6f} V "
                    f"neg/pos={u_sequence['negative_ratio']:.5f}"
                )

                print("\nFAST continuity")
                for name, cont in (("A", cont_a), ("B", cont_b)):
                    print(
                        f"  window {name}: tick={cont['first_tick']}..{cont['last_tick']} "
                        f"decim={cont['decimation']} gaps={cont['gap_count']} "
                        f"missing_plot={cont['missing_plot_samples']} "
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
                    },
                    "window_a": {
                        "continuity": cont_a,
                        "metrics": a_metrics,
                        "integrator_theory": int_theory,
                        "raw_samples": rows_a,
                    },
                    "window_b": {
                        "continuity": cont_b,
                        "metrics": b_metrics,
                        "u_sequence": u_sequence,
                        "raw_samples": rows_b,
                    },
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
