#!/usr/bin/env python3
"""Characterize current sequence under the firmware voltage diagnostic mode."""

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
IDENT_MODE_SET = 0x01
IDENT_STATUS = 0x02
IDENT_VOLTAGE_DIAG = 0x03
IDENT_RUNNING = 1
VOLT_STAGE_HOLD = 2
FAST_CONFIG_ID = 17
NORMAL_CONFIG_ID = 18
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
    ratio = n / p if p > 0.0 else math.nan
    return {
        "positive_mag": p,
        "negative_mag": n,
        "negative_positive_ratio": ratio,
        "angle_2fe_est_rad": ratio,
    }


class StopRequest:
    def __init__(self):
        self.requested = False

    def handle(self, signum, frame):
        del signum, frame
        self.requested = True
        print("\nStop requested.")


class VoltageDiag(base.SensorlessTest):
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
        count = len(FAST_VARS)
        sample_count = payload[3]
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

    def collect(self):
        self.samples = []
        self.capture = True
        deadline = time.monotonic() + self.args.duration
        while time.monotonic() < deadline:
            if self.stop_request.requested:
                raise RuntimeError("user stop")
            self.process(self.parser.feed(self.ser.read(4096)))
            self.check_stream()
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
    p = argparse.ArgumentParser(description="Voltage-vector sequence diagnostic")
    p.add_argument("--port", required=True)
    p.add_argument("--run", action="store_true")
    p.add_argument("--rs", type=float, default=6.334559)
    p.add_argument("--ld", type=float, default=0.001564084)
    p.add_argument("--lq", type=float, default=0.001564084)
    p.add_argument("--flux", type=float, default=0.01513128)
    p.add_argument("--pole-pairs", type=int, default=11)
    p.add_argument("--current-limit", type=float, default=2.0)
    p.add_argument("--we", type=float, default=120.0)
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
    log_path = out_dir / f"voltage_sequence_{started.strftime('%Y%m%d_%H%M%S')}.json"
    wm = args.we / args.pole_pairs
    expected_u = abs(args.we) * args.flux + 0.15 * args.current_limit * args.rs
    log = {
        "test": "voltage_sequence_diag",
        "timestamp": started.isoformat(),
        "success": False,
        "error": None,
        "config": vars(args),
        "expected_u_target_v_unclamped": expected_u,
        "vbus": None,
        "sequence": None,
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
            test = VoltageDiag(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.set_parameters()
                test.configure_plot()
                deadline = time.monotonic() + args.vbus_seconds
                while time.monotonic() < deadline:
                    test.process(test.parser.feed(ser.read(4096)))
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
                print(
                    f"Voltage sequence diagnostic: We={args.we:.1f} rad/s, "
                    f"firmware Utarget estimate≈{expected_u:.3f} V before clamp"
                )
                test.wait_hold()
                print(f"Holding for {args.duration:.2f} s...")
                rows = test.collect()
                for row in rows:
                    row["Ialpha"] = row["Ia"]
                    row["Ibeta"] = (row["Ia"] + 2.0 * row["Ib"]) * INV_SQRT3
                i_seq = sequence(rows, "Ialpha", "Ibeta")
                u_seq = sequence(rows, "Ualpha", "Ubeta")
                phase_rms = {name: rms([r[name] for r in rows]) for name in ("Ia", "Ib", "Ic")}
                log["sequence"] = {
                    "current_alpha_beta": i_seq,
                    "ucmd_alpha_beta": u_seq,
                    "phase_rms_a": phase_rms,
                    "samples": len(rows),
                }
                log["raw_samples"] = rows
                log["success"] = True
                print("\nVoltage sequence result")
                print(f"  phase RMS: Ia={phase_rms['Ia']:.4f} A Ib={phase_rms['Ib']:.4f} A Ic={phase_rms['Ic']:.4f} A")
                print("  vector       positive mag    negative mag    neg/pos    2fe est")
                for label, seq in (("I alpha-beta", i_seq), ("Ucmd a-b", u_seq)):
                    print(
                        f"  {label:12s} {seq['positive_mag']:12.6f} {seq['negative_mag']:14.6f} "
                        f"{seq['negative_positive_ratio']:10.5f} {seq['angle_2fe_est_rad']:10.5f} rad"
                    )
            except Exception as exc:
                error = exc
            finally:
                test.stop_all()
                if test.vbus:
                    log["vbus"] = {"mean_v": mean(test.vbus), "min_v": min(test.vbus), "max_v": max(test.vbus)}
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
