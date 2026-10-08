#!/usr/bin/env python3
"""Measure sensored d-axis current-loop FRF using existing AxDr Parameter and FAST Plot.

Requirements: pip install pyserial numpy
The motor must be manually set to TORQUE + ENABLED before running. This tool
never sends Enable, Run, or a persistent Save. Each frequency starts a finite
sine experiment, captures raw FAST samples, then measures Id/IdRef and UdPI/IdRef.
"""

import argparse
import json
import math
from pathlib import Path
import struct
import time

import numpy as np

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial numpy") from exc

from parameter_ids_generated import (
    PARAM_MOTOR_MODE, PARAM_MOTOR_STATE,
    PARAM_RUN_ID, PARAM_SIGNAL_ID_REF, PARAM_SIGNAL_ID_PI_OUT,
    PARAM_SIGNAL_OUT, PARAM_SIGNAL_FREQ_HZ, PARAM_SIGNAL_AMP_A,
    PARAM_SIGNAL_TIME_S, ACTION_SIGNAL_START, ACTION_SIGNAL_ABORT,
)

MAGIC = b"AXDR"
NODE_ID = 1
MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_PARAMETER = 0x07
MSG_FAST_DATA = 0x18
PARAM_READ = 1
PARAM_WRITE = 2
PARAM_FLOAT = 2
PARAM_ACTION = 6
PLOT_CONFIG = 1
PLOT_START = 2
PLOT_STOP = 3
FAST_RATE_HZ = 20000.0
FAST_CONFIG_ID = 21
CHANNELS = (
    ("Injection", PARAM_SIGNAL_OUT, 0.001),
    ("IdRef", PARAM_SIGNAL_ID_REF, 0.001),
    ("Id", PARAM_RUN_ID, 0.001),
    ("UdPI", PARAM_SIGNAL_ID_PI_OUT, 0.001),
)
CANFD_LEN = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)


def wire(msg_type, payload):
    length = next(n for n in CANFD_LEN if n >= len(payload))
    return MAGIC + struct.pack("<HB", (msg_type << 6) | NODE_ID, length) + payload.ljust(length, b"\x00")


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 7:
                break
            msg_id, size = struct.unpack_from("<HB", self.buffer, 4)
            if msg_id > 0x07FF or size not in CANFD_LEN:
                del self.buffer[0]
                continue
            end = 7 + size
            if len(self.buffer) < end:
                break
            frames.append((msg_id >> 6, bytes(self.buffer[7:end])))
            del self.buffer[:end]
        return frames


class Client:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=1.0)
        self.parser = Parser()
        self.txn = 0
        self.fast = bytearray()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = False

    def close(self):
        self.ser.close()

    def poll(self):
        responses = []
        for msg_type, payload in self.parser.feed(self.ser.read(4096)):
            if msg_type == MSG_FAST_DATA and self.collect:
                if len(payload) < 4:
                    self.malformed += 1
                    continue
                seq, = struct.unpack_from("<H", payload)
                n, config_id = payload[3], payload[2]
                needed = 4 + n * len(CHANNELS) * 2
                if config_id != FAST_CONFIG_ID or n == 0 or len(payload) < needed:
                    self.malformed += 1
                    continue
                if self.last_seq is not None:
                    self.dropped += (seq - self.last_seq - 1) & 0xFFFF
                self.last_seq = seq
                self.fast.extend(payload[4:needed])
            elif msg_type == MSG_RESPONSE:
                responses.append(payload)
        return responses

    def request(self, msg_type, operation, body=b"", timeout=2.0):
        self.txn = (self.txn % 255) + 1
        txn = self.txn
        self.ser.write(wire(msg_type, bytes([txn, operation]) + body))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for payload in self.poll():
                if len(payload) < 4:
                    continue
                if payload[0] != txn or payload[1] != msg_type or payload[2] != operation:
                    continue
                if payload[3] != 0:
                    raise RuntimeError(f"request {msg_type:02X}/{operation:02X} status {payload[3]}")
                return payload[4:]
        raise TimeoutError(f"request {msg_type:02X}/{operation:02X} timed out")

    def read_u8(self, pid):
        payload = self.request(MSG_PARAMETER, PARAM_READ, struct.pack("<H", pid))
        if len(payload) < 4 or struct.unpack_from("<H", payload)[0] != pid or payload[2] != 0:
            raise RuntimeError(f"unexpected value reply for 0x{pid:04X}")
        return payload[3]

    def set_float(self, pid, value):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HBf", pid, PARAM_FLOAT, value))

    def action(self, pid):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HB", pid, PARAM_ACTION))

    def plot_start(self):
        vars_payload = bytes([0, FAST_CONFIG_ID, len(CHANNELS)])
        vars_payload += b"".join(struct.pack("<H", pid) for _, pid, _ in CHANNELS)
        self.request(MSG_PLOT, PLOT_CONFIG, vars_payload)
        self.fast.clear()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = True
        self.request(MSG_PLOT, PLOT_START, b"\x01")

    def plot_stop(self):
        self.request(MSG_PLOT, PLOT_STOP, b"\x01")
        self.collect = False


def sinusoid_fit(signal, freq, times):
    # Include DC as a separate fit parameter to avoid leakage from offsets.
    w = 2.0 * math.pi * freq
    basis = np.column_stack((np.sin(w * times), np.cos(w * times), np.ones(len(times))))
    coef = np.linalg.lstsq(basis, signal, rcond=None)[0]
    residue = signal - basis @ coef
    # Phasor in cosine convention; ratios do not depend on convention.
    return complex(coef[1], -coef[0]), float(np.sqrt(np.mean(residue ** 2)))


def analyze(raw, freq, amp, settle_cycles):
    data = np.frombuffer(raw, dtype="<i2")
    if len(data) % len(CHANNELS) != 0:
        raise RuntimeError("FAST sample buffer is not a whole number of records")
    raw_matrix = data.reshape(-1, len(CHANNELS))
    samples = raw_matrix.astype(np.float64) * np.array([c[2] for c in CHANNELS])
    inj = samples[:, 0]
    active = np.flatnonzero(np.abs(inj) > max(0.001, amp * 0.02))
    if len(active) < 50:
        raise RuntimeError("sine excitation missing from FAST Plot")
    start = int(active[0])
    end = int(active[-1]) + 1
    discard = int(round(settle_cycles * FAST_RATE_HZ / freq))
    start += discard
    if end - start < max(100, int(round(2.0 * FAST_RATE_HZ / freq))):
        raise RuntimeError("too few settled cycles; increase --duration")
    interval = samples[start:end]
    t = np.arange(len(interval), dtype=np.float64) / FAST_RATE_HZ
    ref, ref_err = sinusoid_fit(interval[:, 1], freq, t)
    current, cur_err = sinusoid_fit(interval[:, 2], freq, t)
    pi_out, out_err = sinusoid_fit(interval[:, 3], freq, t)
    if abs(ref) < max(0.005, amp * 0.2):
        raise RuntimeError("injected IdRef amplitude too small")
    loop = current / ref
    control = pi_out / ref

    def response(value):
        return {
            "gain_db": float(20.0 * np.log10(max(abs(value), 1e-20))),
            "phase_deg": float(np.angle(value, deg=True)),
        }

    return raw_matrix, {
        "frequency_hz": freq,
        "amplitude_requested_a": amp,
        "input_amplitude_measured_a": float(abs(ref)),
        "closed_loop_Id_over_IdRef": response(loop),
        "controller_UdPI_over_IdRef": response(control),
        "fit_residual_rms": {
            "id_ref_a": ref_err, "id_a": cur_err, "ud_pi_v": out_err,
        },
        "used_samples": len(interval),
        "captured_samples": len(samples),
        "sample_rate_hz": FAST_RATE_HZ,
        "settle_cycles": settle_cycles,
    }


def main():
    p = argparse.ArgumentParser(description="AxDr sensored d-axis current-loop sine FRF")
    p.add_argument("--port", required=True, help="USB CDC port, e.g. /dev/ttyACM0 or COM5")
    p.add_argument("--baud", type=int, default=115200,
                   help="USB CDC line coding; not a UART bandwidth limit")
    p.add_argument("--freq", type=float, nargs="+", required=True, help="Frequency points in Hz")
    p.add_argument("--amp", type=float, default=0.1, help="Injected peak current, A")
    p.add_argument("--duration", type=float, default=2.0, help="Seconds per point, max 60")
    p.add_argument("--settle-cycles", type=float, default=2.0, help="Initial periods excluded")
    p.add_argument("--out", type=Path, default=Path("current_frf_capture"))
    p.add_argument("--plot", action="store_true", help="write bode.png (requires matplotlib)")
    args = p.parse_args()
    if (not args.freq or any(not math.isfinite(f) or f <= 0 or f > 5000 for f in args.freq) or
        not math.isfinite(args.amp) or args.amp <= 0 or
        not math.isfinite(args.duration) or not 0 < args.duration <= 60 or
        not math.isfinite(args.settle_cycles) or args.settle_cycles < 0):
        p.error("invalid frequency, amplitude, duration or settle cycles")

    args.out.mkdir(parents=True, exist_ok=True)
    client = Client(args.port, args.baud)
    results = []
    plotting = False
    try:
        if client.read_u8(PARAM_MOTOR_MODE) != 0 or client.read_u8(PARAM_MOTOR_STATE) != 1:
            raise RuntimeError("set TORQUE mode and ENABLED state before testing; script will not Enable/Run")
        for i, freq in enumerate(args.freq):
            if client.read_u8(PARAM_MOTOR_STATE) != 1:
                raise RuntimeError("motor left ENABLED state")
            client.set_float(PARAM_SIGNAL_FREQ_HZ, freq)
            client.set_float(PARAM_SIGNAL_AMP_A, args.amp)
            client.set_float(PARAM_SIGNAL_TIME_S, args.duration)
            client.plot_start()
            plotting = True
            try:
                client.action(ACTION_SIGNAL_START)
                end = time.monotonic() + args.duration + 0.25
                while time.monotonic() < end:
                    client.poll()
                client.action(ACTION_SIGNAL_ABORT)
                client.plot_stop()
                plotting = False
            finally:
                # Abort is idempotent and does not alter Motor State.
                client.action(ACTION_SIGNAL_ABORT)

            if client.dropped or client.malformed:
                raise RuntimeError(f"FAST data incomplete: seq_lost={client.dropped}, malformed={client.malformed}")
            raw, result = analyze(client.fast, freq, args.amp, args.settle_cycles)
            result["channels"] = [
                {"name": name, "parameter_id": pid, "scale": scale}
                for name, pid, scale in CHANNELS
            ]
            result["fast_frames_lost"] = client.dropped
            raw_path = args.out / f"point_{i:03d}.raw"
            raw_path.write_bytes(client.fast)
            result["raw_file"] = raw_path.name
            results.append(result)
            print(f"{freq:g} Hz: Id/IdRef={result['closed_loop_Id_over_IdRef']['gain_db']:+.2f} dB, "
                  f"phase={result['closed_loop_Id_over_IdRef']['phase_deg']:+.1f} deg, "
                  f"samples={result['used_samples']}")

        (args.out / "bode.json").write_text(
            json.dumps({"measure": "sensored_d_current_closed_loop",
                        "sample_rate_hz": FAST_RATE_HZ, "points": results},
                       ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        if args.plot:
            try:
                import matplotlib.pyplot as plt
            except ImportError as exc:
                raise RuntimeError("matplotlib is required for --plot") from exc
            xs = np.array([point["frequency_hz"] for point in results])
            order = np.argsort(xs)
            ys = [results[i]["closed_loop_Id_over_IdRef"] for i in order]
            gain = [v["gain_db"] for v in ys]
            phase = np.rad2deg(np.unwrap(np.deg2rad([v["phase_deg"] for v in ys])))
            fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
            axes[0].semilogx(xs[order], gain, "o-")
            axes[0].set_ylabel("Id / IdRef (dB)")
            axes[1].semilogx(xs[order], phase, "o-")
            axes[1].set_ylabel("Phase (deg)")
            axes[1].set_xlabel("Frequency (Hz)")
            for axis in axes:
                axis.grid(True, which="both")
            fig.tight_layout()
            fig.savefig(args.out / "bode.png", dpi=150)
            plt.close(fig)
        print("Results:", args.out / "bode.json")
    finally:
        try:
            client.action(ACTION_SIGNAL_ABORT)
        except (OSError, RuntimeError, TimeoutError):
            pass
        if plotting:
            try:
                client.plot_stop()
            except (OSError, RuntimeError, TimeoutError):
                pass
        client.close()


if __name__ == "__main__":
    main()
