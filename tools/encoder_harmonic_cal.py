#!/usr/bin/env python3
"""Fit encoder mechanical 1x/2x angle error during unloaded VF rotation.

No firmware calibration state machine. By default only prints coefficients;
--apply writes them, runs existing phase search and saves NVS. Requires --run.
USB loss cannot guarantee motor shutdown: supervise the test physically.
"""

import argparse
import math
import struct
import time

import numpy as np
import serial

import sensorless_test as base

MODE_OPEN_LOOP = 3
MODE_PHASE_SEARCH = 6
METHOD_VF = 1
FAST_CONFIG = 31
CHANNELS = (
    base.PARAM_ENCODER_THETA_NATIVE,
    base.PARAM_ADC_IA,
    base.PARAM_ADC_IB,
    base.PARAM_ADC_IC,
)
COEFFICIENTS = (
    base.PARAM_ENCODER_HARMONIC_C1,
    base.PARAM_ENCODER_HARMONIC_S1,
    base.PARAM_ENCODER_HARMONIC_C2,
    base.PARAM_ENCODER_HARMONIC_S2,
)


class Client(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.raw = bytearray()
        self.collect = False
        self.seq = None
        self.frames = 0
        self.last_rx = time.monotonic()
        self.peak = 0.0

    def request(self, msg_type, op, data=b""):
        # AxDr uses CAN FD payload lengths even on USB CDC.
        size = 2 + len(data)
        frame_len = next((n for n in (*range(9), 12, 16, 20, 24, 32, 48, 64)
                          if n >= size), None)
        if frame_len is None:
            raise ValueError("AxDr payload exceeds 64 bytes")
        return super().request(msg_type, op, data.ljust(frame_len - 2, b"\0"))

    def parameter_read(self, param_id, param_type):
        reply = self.request(base.MSG_PARAMETER, base.PARAM_READ, struct.pack("<H", param_id))
        expected = 3 + struct.calcsize(base.PARAM_FORMAT[param_type])
        if len(reply) < expected or any(reply[expected:]):
            raise RuntimeError("invalid Parameter response padding")
        rid, kind = struct.unpack_from("<HB", reply)
        if rid != param_id or kind != param_type:
            raise RuntimeError("Parameter response ID/type mismatch")
        return struct.unpack_from(base.PARAM_FORMAT[param_type], reply, 3)[0]

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG:
            return
        seq, = struct.unpack_from("<H", payload)
        count = payload[3]
        required = 4 + count * len(CHANNELS) * 2
        if count == 0 or len(payload) < required or any(payload[required:]):
            raise RuntimeError("malformed FAST frame")
        if self.seq is not None and ((seq - self.seq) & 0xFFFF) != 1:
            raise RuntimeError("FAST sequence gap")
        self.seq = seq
        self.frames += 1
        self.last_rx = time.monotonic()
        samples = np.frombuffer(payload[4:required], dtype="<i2").reshape(-1, len(CHANNELS))
        self.peak = max(self.peak, float(np.max(np.abs(samples[:, 1:])) * 0.001))
        if self.peak > self.args.current_limit:
            raise RuntimeError(f"measured phase current exceeded {self.args.current_limit:g} A")
        if self.collect:
            self.raw.extend(payload[4:required])

    def poll_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.process(self.parser.feed(self.ser.read(4096)))
            if time.monotonic() - self.last_rx > 0.5:
                raise RuntimeError("FAST Plot stopped")

    def plot_start(self):
        self.request(base.MSG_PLOT, base.PLOT_STOP, bytes((base.FAST_MASK | base.NORMAL_MASK,)))
        data = bytes((base.FAST_GROUP, FAST_CONFIG, len(CHANNELS)))
        data += b"".join(struct.pack("<H", item) for item in CHANNELS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, data)
        before = self.parameter_read(base.PARAM_PLOT_FAST_DROP, base.PARAM_U32)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes((base.FAST_MASK,)))
        self.last_rx = time.monotonic()
        return before


def fit(raw):
    values = np.frombuffer(raw, dtype="<i2").reshape(-1, len(CHANNELS))
    angle = np.unwrap(values[:, 0].astype(np.float64) * 0.0002)
    turns = abs(angle[-1] - angle[0]) / (2 * np.pi)
    if turns < 10:
        raise RuntimeError(f"too few encoder turns: {turns:.1f}")
    t = np.arange(len(angle), dtype=np.float64) / 20000.0
    phase = values[:, 0].astype(np.float64) * 0.0002
    matrix = np.column_stack((
        np.ones(len(angle)), t - t.mean(),
        np.cos(phase), np.sin(phase),
        np.cos(2 * phase), np.sin(2 * phase),
    ))
    result, _, rank, _ = np.linalg.lstsq(matrix, angle, rcond=None)
    if rank != 6 or not np.all(np.isfinite(result)):
        raise RuntimeError("harmonic fit failed")
    coeff = result[2:]
    if np.any(np.abs(coeff) > 0.05):
        raise RuntimeError("harmonic coefficient outside firmware range")
    residual = angle - matrix @ result
    print(f"Turns={turns:.1f}; mean={result[1] * 60 / (2 * np.pi):.2f} rpm; "
          f"fit residual={np.std(residual) * 180 / np.pi:.4f} deg (mechanical)")
    print("C1/S1/C2/S2 (mechanical rad):", " ".join(f"{v:+.8f}" for v in coeff))
    return coeff


def apply(client, coeff):
    for param, value in zip(COEFFICIENTS, coeff):
        client.parameter_write(param, base.PARAM_FLOAT, float(value))
    client.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_PHASE_SEARCH)
    client.parameter_action(base.ACTION_MOTOR_ENABLE)
    txn = client.parameter_action(base.ACTION_MOTOR_RUN)
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        client.process(client.parser.feed(client.ser.read(4096)))
        status = client.action_complete_status(txn, base.ACTION_MOTOR_RUN)
        if status is not None:
            if status != 0 or client.parameter_read(base.PARAM_CAL_VALID, base.PARAM_U8) != 1:
                raise RuntimeError(f"servo phase search failed: {status}")
            client.parameter_action(base.ACTION_PARAMETER_SAVE)
            print("Harmonic coefficients and phase alignment saved.")
            return
    raise TimeoutError("servo phase search timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rpm", type=float, default=1000)
    parser.add_argument("--seconds", type=float, default=6)
    parser.add_argument("--settle", type=float, default=2)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--run", action="store_true", help="authorize actual motor motion")
    parser.add_argument("--apply", action="store_true", help="write coefficients, phase search, save")
    args = parser.parse_args()
    if (not math.isfinite(args.rpm) or not 100 <= abs(args.rpm) <= 3000 or
            not 2 <= args.seconds <= 30 or not 0.5 <= args.settle <= 10 or
            not 0.1 <= args.current_limit <= 5):
        parser.error("invalid rpm, duration, settle time or host current limit")
    if not args.run:
        print("Dry run only; use --run to rotate, --apply to also save.")
        return
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1) as ser:
        client = Client(ser, args)
        previous = {}
        try:
            client.parameter_action(base.ACTION_MOTOR_DISABLE)
            for key, kind in (("mode", base.PARAM_MOTOR_MODE),
                              ("method", base.PARAM_OPEN_LOOP_METHOD)):
                previous[key] = client.parameter_read(kind, base.PARAM_U8)
            previous["speed"] = client.parameter_read(base.PARAM_TARGET_SPEED, base.PARAM_FLOAT)
            if (client.parameter_read(base.PARAM_ENCODER_READY, base.PARAM_U8) != 1 or
                    client.parameter_read(base.PARAM_ENCODER_FAULT, base.PARAM_U8)):
                raise RuntimeError("encoder not ready")
            if client.parameter_read(base.PARAM_MOTOR_FLUX, base.PARAM_FLOAT) <= 0:
                raise RuntimeError("motor flux not identified")
            acceleration = client.parameter_read(base.PARAM_MOTION_WM_ACC, base.PARAM_FLOAT)
            if acceleration <= 0:
                raise RuntimeError("invalid acceleration")
            before = None
            try:
                client.parameter_write(base.PARAM_OPEN_LOOP_METHOD, base.PARAM_U8, METHOD_VF)
                client.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_OPEN_LOOP)
                client.speed_set(args.rpm * 2 * math.pi / 60)
                before = client.plot_start()
                client.parameter_action(base.ACTION_MOTOR_ENABLE)
                client.parameter_action(base.ACTION_MOTOR_RUN)
                client.poll_for(abs(args.rpm) * 2 * math.pi / (60 * acceleration) + args.settle)
                speeds = []
                for _ in range(8):
                    speeds.append(client.parameter_read(base.PARAM_MECH_ESO_WM, base.PARAM_FLOAT))
                    client.poll_for(0.05)
                measured_rpm = abs(sum(speeds) / len(speeds)) * 60 / (2 * math.pi)
                if abs(measured_rpm - abs(args.rpm)) > 0.1 * abs(args.rpm):
                    raise RuntimeError(f"rotor not synchronized: {measured_rpm:.1f} rpm")
                client.raw.clear()
                client.collect = True
                client.poll_for(args.seconds)
                client.collect = False
                if client.parameter_read(base.PARAM_PLOT_FAST_DROP, base.PARAM_U32) != before:
                    raise RuntimeError("firmware FAST Plot dropped blocks")
                if client.parameter_read(base.PARAM_ENCODER_FAULT, base.PARAM_U8):
                    raise RuntimeError("encoder fault during rotation")
            finally:
                client.collect = False
                try:
                    client.parameter_action(base.ACTION_MOTOR_STOP)
                    client.parameter_action(base.ACTION_MOTOR_DISABLE)
                finally:
                    if before is not None:
                        client.request(base.MSG_PLOT, base.PLOT_STOP, bytes((base.FAST_MASK,)))
            result = fit(bytes(client.raw))
            print(f"Measured phase current peak: {client.peak:.3f} A")
            if args.apply:
                apply(client, result)
        finally:
            try:
                client.parameter_action(base.ACTION_MOTOR_DISABLE)
                if "method" in previous:
                    client.parameter_write(base.PARAM_OPEN_LOOP_METHOD, base.PARAM_U8, previous["method"])
                    client.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, previous["mode"])
                    client.speed_set(previous["speed"])
            except (RuntimeError, TimeoutError, serial.SerialException) as exc:
                print(f"Warning: could not restore commands: {exc}")


if __name__ == "__main__":
    main()
