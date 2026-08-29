#!/usr/bin/env python3
"""Diagnose PWM-synchronous current-sampling errors by electrical sector.

Runs the existing pure-voltage identification diagnostic so the commanded alpha/beta
voltage is an almost ideal positive-sequence rotating vector and no current PI is
present. FAST samples of Theta_e, Ia/Ib/Ic and Ualpha/Ubeta are split into six
60-degree electrical sectors. The host reports per-sector phase RMS and three-phase
closure error, plus whole-run alpha/beta positive/negative sequence.

No control behavior is modified by this tool. Optional RAM-only Ia/Ib gain factors
are written before ENABLE and read back for controlled comparisons.
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

import sensorless_test as base


MSG_IDENTIFICATION = 0x05
MODE_IDENT = 4
CTRL_I_LIMIT_SET = 0x07
CTRL_MOTOR_PARA_SET = 0x0A
CTRL_CURRENT_GAIN_SET = 0x0C
CTRL_CURRENT_GAIN_GET = 0x0D
IDENT_MODE_SET = 0x01
IDENT_STATUS = 0x02
IDENT_VOLTAGE_DIAG = 0x03
IDENT_RUNNING = 1
VOLT_STAGE_HOLD = 2
FAST_CONFIG_ID = 19
NORMAL_CONFIG_ID = 20
FAST_VARS = (
    ("Theta", 0x0014, 0.0002),
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
    ("Ic", 0x0003, 0.001),
    ("Ualpha", 0x0015, 0.001),
    ("Ubeta", 0x0016, 0.001),
)
VBUS_ID = 0x0004
INV_SQRT3 = 1.0 / math.sqrt(3.0)
TWO_PI = 2.0 * math.pi
SECTOR_RAD = math.pi / 3.0


def mean(values):
    return statistics.fmean(values) if values else math.nan


def rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values)) if values else math.nan


def sequence(rows, x_name, y_name):
    pos = 0j
    neg = 0j
    for row in rows:
        theta = row["Theta"]
        z = complex(row[x_name], row[y_name])
        rot = complex(math.cos(theta), math.sin(theta))
        pos += z / rot
        neg += z * rot
    pos /= len(rows)
    neg /= len(rows)
    p = abs(pos)
    n = abs(neg)
    return {
        "positive_mag": p,
        "negative_mag": n,
        "negative_positive_ratio": n / p if p > 1e-12 else math.nan,
    }


def vector_sector(row):
    # Use the actual commanded voltage vector as the sector reference. This
    # remains valid if Theta_e has a fixed offset relative to the phase axes.
    theta = math.atan2(row["Ubeta"], row["Ualpha"])
    if theta < 0.0:
        theta += TWO_PI
    sector = int(theta / SECTOR_RAD)
    return min(sector, 5)


def sector_metrics(rows):
    groups = [[] for _ in range(6)]
    for row in rows:
        sector = vector_sector(row)
        row["sector"] = sector + 1
        groups[sector].append(row)

    result = []
    for index, group in enumerate(groups):
        if not group:
            result.append({"sector": index + 1, "samples": 0})
            continue
        ia = [r["Ia"] for r in group]
        ib = [r["Ib"] for r in group]
        ic = [r["Ic"] for r in group]
        closure = [a + b + c for a, b, c in zip(ia, ib, ic)]
        phase_sq = [v * v for triplet in zip(ia, ib, ic) for v in triplet]
        phase_rms = math.sqrt(mean(phase_sq))
        result.append({
            "sector": index + 1,
            "angle_start_deg": index * 60,
            "angle_end_deg": (index + 1) * 60,
            "samples": len(group),
            "ia_rms_a": rms(ia),
            "ib_rms_a": rms(ib),
            "ic_rms_a": rms(ic),
            "closure_mean_a": mean(closure),
            "closure_rms_a": rms(closure),
            "closure_relative": rms(closure) / phase_rms if phase_rms > 1e-12 else math.nan,
        })
    return result


class StopRequest:
    def __init__(self):
        self.requested = False

    def handle(self, signum, frame):
        del signum, frame
        self.requested = True
        print("\nStop requested.")


class SectorDiag(base.SensorlessTest):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args)
        self.stop_request = stop
        self.samples = []
        self.vbus = []
        self.normal_last = None
        self.normal_lost = 0
        self.current_peak = 0.0
        self.capture = False

    def prepare(self):
        for msg_type, op, data in (
            (base.MSG_CONTROL, base.CTRL_STOP, b""),
            (base.MSG_CONTROL, base.CTRL_DISABLE, b""),
            (base.MSG_PLOT, base.PLOT_STOP, bytes([base.FAST_MASK | base.NORMAL_MASK])),
        ):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError):
                pass

    def set_parameters(self):
        payload = bytes([self.args.pole_pairs]) + struct.pack(
            "<ffff", self.args.rs, self.args.ld, self.args.lq, self.args.flux
        )
        self.request(base.MSG_CONTROL, CTRL_MOTOR_PARA_SET, payload)
        self.request(base.MSG_CONTROL, CTRL_I_LIMIT_SET, struct.pack("<f", self.args.current_limit))
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
            raise RuntimeError("current gain readback mismatch")
        print(f"Current gain RAM: Ia={ia_gain:.5f}, Ib={ib_gain:.5f}")

    def configure_plot(self):
        fast = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast += b"".join(struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast)
        normal = bytes([base.NORMAL_GROUP, NORMAL_CONFIG_ID, 1]) + struct.pack("<H", VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes([base.FAST_MASK | base.NORMAL_MASK]))

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return
        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        if len(payload) != 4 + sample_count * count * 2:
            return
        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected) & 0xFFFF
        self.fast_last = seq
        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for s in range(sample_count):
            start = s * count
            row = {
                name: raw[start + i] * scale
                for i, (name, _, scale) in enumerate(FAST_VARS)
            }
            self.current_peak = max(
                self.current_peak, abs(row["Ia"]), abs(row["Ib"]), abs(row["Ic"])
            )
            if self.current_peak > self.args.host_guard:
                raise RuntimeError(
                    f"phase current exceeded host guard {self.args.host_guard:.3f} A"
                )
            if self.capture:
                self.samples.append(row)

    def process_normal(self, payload):
        if len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or payload[3] != 1:
            return
        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        value, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(value)

    def ident_status(self):
        data = self.request(MSG_IDENTIFICATION, IDENT_STATUS)
        if len(data) < 4:
            raise RuntimeError(f"invalid diagnostic status length: {len(data)}")
        return data[0], data[1], data[2], data[3]

    def check_stream(self):
        if self.fast_lost > self.args.max_lost:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")
        if self.normal_lost > self.args.max_lost:
            raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")
        if self.vbus and not self.args.vbus_min <= self.vbus[-1] <= self.args.vbus_max:
            raise RuntimeError(f"Vbus {self.vbus[-1]:.3f} V outside allowed range")

    def wait_hold(self):
        deadline = time.monotonic() + self.args.ready_timeout
        last_stage = None
        while time.monotonic() < deadline:
            if self.stop_request.requested:
                raise RuntimeError("user stop")
            self.process(self.parser.feed(self.ser.read(4096)))
            self.check_stream()
            mode, state, stage, active = self.ident_status()
            if mode != IDENT_VOLTAGE_DIAG or state != IDENT_RUNNING or not active:
                raise RuntimeError(f"diagnostic stopped: mode={mode} state={state}")
            if stage != last_stage:
                print(f"  Voltage diagnostic stage={'HOLD' if stage == 2 else 'RAMP' if stage == 1 else stage}")
                last_stage = stage
            if stage == VOLT_STAGE_HOLD:
                return
            time.sleep(0.02)
        raise TimeoutError("voltage diagnostic did not reach HOLD")

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.stop_request.requested:
                raise RuntimeError("user stop")
            self.process(self.parser.feed(self.ser.read(4096)))
            self.check_stream()

    def collect(self):
        self.samples = []
        self.capture = True
        self.pump(self.args.duration)
        self.capture = False
        if len(self.samples) < 1000:
            raise RuntimeError(f"too few FAST samples: {len(self.samples)}")
        return self.samples

    def stop_all(self):
        self.capture = False
        for msg_type, op, data, label in (
            (base.MSG_CONTROL, base.CTRL_STOP, b"", "STOP"),
            (base.MSG_CONTROL, base.CTRL_DISABLE, b"", "DISABLE"),
            (base.MSG_PLOT, base.PLOT_STOP, bytes([base.FAST_MASK | base.NORMAL_MASK]), "PLOT_STOP"),
        ):
            try:
                self.request(msg_type, op, data)
                print(f"{label} OK")
            except (TimeoutError, RuntimeError):
                pass


def parse_args():
    p = argparse.ArgumentParser(description="Current-sampling diagnostics by 60-degree electrical sector")
    p.add_argument("--port", required=True)
    p.add_argument("--run", action="store_true")
    p.add_argument("--rs", type=float, default=6.334559)
    p.add_argument("--ld", type=float, default=0.001564084)
    p.add_argument("--lq", type=float, default=0.001564084)
    p.add_argument("--flux", type=float, default=0.01513128)
    p.add_argument("--pole-pairs", type=int, default=11)
    p.add_argument("--current-limit", type=float, default=2.0)
    p.add_argument("--ia-gain", type=float, default=1.0)
    p.add_argument("--ib-gain", type=float, default=1.0)
    p.add_argument("--we", type=float, default=120.0)
    p.add_argument("--settle-seconds", type=float, default=0.5)
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--vbus-min", type=float, default=10.0)
    p.add_argument("--vbus-max", type=float, default=20.0)
    p.add_argument("--vbus-seconds", type=float, default=0.2)
    p.add_argument("--ready-timeout", type=float, default=20.0)
    p.add_argument("--max-lost", type=int, default=0)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=1.0)
    args = p.parse_args()
    if not args.run:
        p.error("--run is required")
    if not (0.8 <= args.ia_gain <= 1.2 and 0.8 <= args.ib_gain <= 1.2):
        p.error("--ia-gain/--ib-gain must be within 0.8 .. 1.2")
    args.host_guard = 1.10 * args.current_limit
    return args


def main():
    args = parse_args()
    stop = StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)
    started = datetime.now().astimezone()
    out_dir = Path(__file__).resolve().parents[1] / "build" / "Release"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / f"current_sector_{started.strftime('%Y%m%d_%H%M%S')}.json"
    wm = args.we / args.pole_pairs
    log = {
        "test": "current_sampling_sector_diag",
        "timestamp": started.isoformat(),
        "success": False,
        "error": None,
        "config": vars(args),
        "vbus": None,
        "whole_run": None,
        "sectors": [],
        "raw_samples": [],
        "guard": None,
    }
    error = None
    test = None
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = SectorDiag(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.set_parameters()
                test.configure_plot()
                test.pump(args.vbus_seconds)
                if not test.vbus:
                    raise RuntimeError("no Vbus samples")
                vmean = mean(test.vbus)
                print(f"Vbus={vmean:.3f} V ({min(test.vbus):.3f} .. {max(test.vbus):.3f} V)")
                if not args.vbus_min <= vmean <= args.vbus_max:
                    raise RuntimeError("Vbus outside allowed range")

                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([MODE_IDENT]))
                test.request(MSG_IDENTIFICATION, IDENT_MODE_SET, bytes([IDENT_VOLTAGE_DIAG]))
                test.request(base.MSG_CONTROL, base.CTRL_SPEED_SET, struct.pack("<f", wm))
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print(f"Current sector diagnostic: We={args.we:.1f} rad/s")
                test.wait_hold()
                print(f"Settling for {args.settle_seconds:.2f} s...")
                test.pump(args.settle_seconds)
                print(f"Collecting for {args.duration:.2f} s...")
                rows = test.collect()

                for row in rows:
                    row["Ialpha"] = row["Ia"]
                    row["Ibeta"] = (row["Ia"] + 2.0 * row["Ib"]) * INV_SQRT3
                i_seq = sequence(rows, "Ialpha", "Ibeta")
                u_seq = sequence(rows, "Ualpha", "Ubeta")
                sectors = sector_metrics(rows)
                log["whole_run"] = {
                    "current_alpha_beta": i_seq,
                    "ucmd_alpha_beta": u_seq,
                    "samples": len(rows),
                }
                log["sectors"] = sectors
                log["raw_samples"] = rows
                log["success"] = True

                print("\nWhole-run sequence")
                print(
                    f"  I neg/pos={i_seq['negative_positive_ratio']:.5f}; "
                    f"Ucmd neg/pos={u_seq['negative_positive_ratio']:.5f}"
                )
                print("\nPer-sector current sampling")
                print("  sec   deg      samples   Ia RMS   Ib RMS   Ic RMS   closure RMS   closure rel")
                for s in sectors:
                    print(
                        f"   {s['sector']}   {s['angle_start_deg']:3d}-{s['angle_end_deg']:3d}  "
                        f"{s['samples']:7d}   {s['ia_rms_a']:6.4f}   {s['ib_rms_a']:6.4f}   "
                        f"{s['ic_rms_a']:6.4f}     {s['closure_rms_a']:8.5f} A   "
                        f"{100.0 * s['closure_relative']:7.2f}%"
                    )
            except Exception as exc:
                error = exc
            finally:
                test.stop_all()
                if test.vbus:
                    log["vbus"] = {
                        "mean_v": mean(test.vbus),
                        "min_v": min(test.vbus),
                        "max_v": max(test.vbus),
                    }
                log["guard"] = {
                    "peak_current_a": test.current_peak,
                    "fast_lost": test.fast_lost,
                    "normal_lost": test.normal_lost,
                }
    except Exception as exc:
        error = error or exc
    finally:
        if error is not None:
            log["error"] = str(error)
        with log_path.open("w", encoding="utf-8") as f:
            json.dump(log, f, indent=2, ensure_ascii=False, allow_nan=False)
        print(f"Log: {log_path}")
    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
