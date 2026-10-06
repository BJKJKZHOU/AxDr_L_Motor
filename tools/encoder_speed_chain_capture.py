#!/usr/bin/env python3
"""Atomic SENSORLESS_SPEED capture for encoder angle -> speed-chain diagnosis.

No repository Python modules are imported. This file owns the AXDR USB framing,
Parameter access, motor actions, Plot control and AXDRCAP1 binary output.

Only pyserial is required.

Default run (80 -> 120 -> 150 -> 120 -> 80 mechanical rad/s):
    python3 tools/encoder_speed_chain_capture.py --port /dev/ttyACM0 --run

Negative run:
    python3 tools/encoder_speed_chain_capture.py --port /dev/ttyACM0 --run \
        --target -5 --target -10 --target -20

The .axdr data section is a sequence of raw 1 kHz NORMAL Plot payloads:
    <HBB9f
    seq, config_id, count,
    wm_ref, wm_sensorless, wm_eso, we_obs,
    theta_e_foc, theta_e_obs, position_turns, iq_ref, iq
"""

from __future__ import annotations

import argparse
import json
import signal
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pyserial is required: python3 -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"AXDR"
CAPTURE_MAGIC = b"AXDRCAP1"
NODE_ID = 1

MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_PARAMETER = 0x07
MSG_NORMAL_DATA = 0x10

PARAM_READ = 0x01
PARAM_WRITE = 0x02

PARAM_U8 = 0
PARAM_I8 = 1
PARAM_FLOAT = 2
PARAM_ACTION = 6

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03
NORMAL_GROUP = 1
NORMAL_MASK = 1 << NORMAL_GROUP
NORMAL_CONFIG_ID = 21
NORMAL_RATE_HZ = 1000.0

MODE_SENSORLESS_SPEED = 5
MOTOR_RUN = 2

PARAM_RUN_IQ = 0x0011
PARAM_RUN_THETA_E = 0x0014
PARAM_OBS_THETA = 0x0020
PARAM_OBS_WE = 0x0021
PARAM_MOTOR_PP = 0x0101
PARAM_MOTOR_DIR = 0x0116
PARAM_ENCODER_PROTOCOL = 0x0301
PARAM_ENCODER_READY = 0x0302
PARAM_ENCODER_FAULT = 0x0304
PARAM_CAL_VALID = 0x0402
PARAM_CAL_ENC_DIR = 0x0403
PARAM_CAL_THETA_OFF = 0x0404
PARAM_RUN_POSITION = 0x0501
PARAM_RUN_WM = 0x0502
PARAM_MECH_ESO_WM = 0x0023
PARAM_REF_IQ = 0x0510
PARAM_REF_WM = 0x0511
PARAM_MOTOR_MODE = 0x0701
PARAM_TARGET_SPEED = 0x0703
PARAM_MOTOR_STATE = 0x0710

ACTION_MOTOR_ENABLE = 0x1001
ACTION_MOTOR_RUN = 0x1002
ACTION_MOTOR_STOP = 0x1003
ACTION_MOTOR_DISABLE = 0x1004

CANFD_LENGTHS = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)

STATUS_NAME = {
    0: "OK",
    1: "ERR_OP",
    2: "ERR_LENGTH",
    3: "ERR_VAR_ID",
    4: "ERR_READ_ONLY",
    5: "ERR_VALUE",
    6: "ERR_STATE",
    7: "ERR_CONFIG",
    8: "ERR_BANDWIDTH",
    9: "ERR_NOT_SUPPORTED",
}

CHANNELS = (
    ("wm_ref", PARAM_REF_WM, "rad/s", "user_mechanical"),
    ("wm_sensorless", PARAM_RUN_WM, "rad/s", "user_mechanical"),
    ("wm_eso", PARAM_MECH_ESO_WM, "rad/s", "internal_mechanical"),
    ("we_obs", PARAM_OBS_WE, "rad/s", "internal_electrical"),
    ("theta_e_foc", PARAM_RUN_THETA_E, "rad", "internal_electrical"),
    ("theta_e_obs", PARAM_OBS_THETA, "rad", "internal_electrical"),
    ("position_turns", PARAM_RUN_POSITION, "turn", "user_mechanical"),
    ("iq_ref", PARAM_REF_IQ, "A", "internal_dq"),
    ("iq", PARAM_RUN_IQ, "A", "internal_dq"),
)

RECORD_FORMAT = "<HBB" + "f" * len(CHANNELS)
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

stop_requested = False
force_disable = False


def on_signal(signum, frame):
    del signum, frame
    global stop_requested, force_disable
    if stop_requested:
        force_disable = True
        print("\nImmediate disable requested.")
    else:
        stop_requested = True
        print("\nStop requested.")


def can_id(msg_type):
    return (msg_type << 6) | NODE_ID


def canfd_length(length):
    for candidate in CANFD_LENGTHS:
        if length <= candidate:
            return candidate
    raise ValueError(f"invalid CAN-FD payload length {length}")


def usb_frame(msg_type, payload):
    length = canfd_length(len(payload))
    payload += bytes(length - len(payload))
    return MAGIC + struct.pack("<HB", can_id(msg_type), length) + payload


def strip_padding(payload, used):
    if len(payload) < used or any(payload[used:]):
        raise RuntimeError("invalid AXDR response length/padding")
    return payload[:used]


class Parser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []

        while True:
            pos = self.buf.find(MAGIC)
            if pos < 0:
                if len(self.buf) > 3:
                    del self.buf[:-3]
                break
            if pos:
                del self.buf[:pos]
            if len(self.buf) < 7:
                break

            msg_id, length = struct.unpack_from("<HB", self.buf, 4)
            if msg_id > 0x07FF or length not in CANFD_LENGTHS:
                del self.buf[0]
                continue

            total = 7 + length
            if len(self.buf) < total:
                break

            frames.append((msg_id, bytes(self.buf[7:total])))
            del self.buf[:total]

        return frames


class Capture:
    def __init__(self, path, metadata):
        self.path = path
        self.records = 0
        self.last_flush = time.monotonic()

        path.parent.mkdir(parents=True, exist_ok=True)
        header = json.dumps(
            metadata,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")

        self.file = path.open("wb")
        self.file.write(CAPTURE_MAGIC)
        self.file.write(struct.pack("<I", len(header)))
        self.file.write(header)

    def write(self, payload):
        self.file.write(payload)
        self.records += 1
        now = time.monotonic()
        if now - self.last_flush >= 0.25:
            self.file.flush()
            self.last_flush = now

    def close(self):
        if self.file is None:
            return
        self.file.flush()
        self.file.close()
        self.file = None


class Axdr:
    def __init__(self, ser, timeout):
        self.ser = ser
        self.timeout = timeout
        self.parser = Parser()
        self.txn = 1
        self.capture = None
        self.last_seq = None
        self.frames = 0
        self.lost = 0

    def process(self, msg_id, payload):
        msg_type = (msg_id >> 6) & 0x1F
        if msg_type != MSG_NORMAL_DATA:
            return msg_type, payload

        if len(payload) != canfd_length(RECORD_SIZE):
            return None
        if any(payload[RECORD_SIZE:]):
            return None
        payload = payload[:RECORD_SIZE]

        seq, config_id, count = struct.unpack_from("<HBB", payload)
        if config_id != NORMAL_CONFIG_ID or count != len(CHANNELS):
            return None

        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFFFF
            self.lost += (seq - expected) & 0xFFFF
        self.last_seq = seq
        self.frames += 1

        if self.capture is not None:
            self.capture.write(payload)
        return None

    def pump(self):
        out = []
        data = self.ser.read(4096)
        if not data:
            return out

        for msg_id, payload in self.parser.feed(data):
            item = self.process(msg_id, payload)
            if item is not None:
                out.append(item)
        return out

    def request(self, msg_type, op, data=b""):
        txn = self.txn
        self.txn = (self.txn % 255) + 1
        self.ser.write(usb_frame(msg_type, bytes([txn, op]) + data))

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            for rx_type, payload in self.pump():
                if rx_type != MSG_RESPONSE or len(payload) < 4:
                    continue
                rx_txn, req_msg, req_op, status = payload[:4]
                if rx_txn != txn or req_msg != msg_type or req_op != op:
                    continue
                if status != 0:
                    raise RuntimeError(
                        f"AXDR {msg_type}/{op}: "
                        f"{STATUS_NAME.get(status, status)}"
                    )
                return payload[4:]

        raise TimeoutError(f"AXDR {msg_type}/{op} response timeout")

    def read(self, param_id, param_type):
        fmt = {
            PARAM_U8: "<B",
            PARAM_I8: "<b",
            PARAM_FLOAT: "<f",
        }[param_type]
        size = struct.calcsize(fmt)

        data = self.request(
            MSG_PARAMETER,
            PARAM_READ,
            struct.pack("<H", param_id),
        )
        data = strip_padding(data, 3 + size)
        response_id, response_type = struct.unpack_from("<HB", data)
        if response_id != param_id or response_type != param_type:
            raise RuntimeError(f"Parameter read mismatch 0x{param_id:04X}")
        return struct.unpack_from(fmt, data, 3)[0]

    def write(self, param_id, param_type, value):
        fmt = {PARAM_U8: "<B", PARAM_FLOAT: "<f"}[param_type]
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", param_id, param_type) +
            struct.pack(fmt, value),
        )
        data = strip_padding(data, 2)
        response_id, = struct.unpack("<H", data)
        if response_id != param_id:
            raise RuntimeError(f"Parameter write mismatch 0x{param_id:04X}")

    def action(self, action_id):
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", action_id, PARAM_ACTION),
        )
        data = strip_padding(data, 2)
        response_id, = struct.unpack("<H", data)
        if response_id != action_id:
            raise RuntimeError(f"Action response mismatch 0x{action_id:04X}")

    def plot_stop(self):
        self.request(MSG_PLOT, PLOT_STOP, bytes([NORMAL_MASK]))

    def plot_config(self):
        data = bytes([NORMAL_GROUP, NORMAL_CONFIG_ID, len(CHANNELS)])
        data += b"".join(struct.pack("<H", item[1]) for item in CHANNELS)

        response = self.request(MSG_PLOT, PLOT_CONFIG, data)
        used = 3 + 2 * len(CHANNELS)
        response = strip_padding(response, used)
        group, config_id, count = response[:3]
        ids = struct.unpack_from(f"<{count}H", response, 3)

        if (
            group != NORMAL_GROUP or
            config_id != NORMAL_CONFIG_ID or
            count != len(CHANNELS) or
            tuple(ids) != tuple(item[1] for item in CHANNELS)
        ):
            raise RuntimeError("Plot config echo mismatch")

    def plot_start(self):
        self.request(MSG_PLOT, PLOT_START, bytes([NORMAL_MASK]))

    def state(self):
        return self.read(PARAM_MOTOR_STATE, PARAM_U8)

    def pump_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not stop_requested:
            self.pump()


def best_effort(client, action_id, name):
    try:
        client.action(action_id)
        print(f"{name} OK")
        return True
    except (TimeoutError, RuntimeError) as exc:
        print(f"{name} warning: {exc}", file=sys.stderr)
        return False


def stop_motor(client, timeout):
    if force_disable:
        best_effort(client, ACTION_MOTOR_DISABLE, "DISABLE")
        return

    if not best_effort(client, ACTION_MOTOR_STOP, "STOP"):
        best_effort(client, ACTION_MOTOR_DISABLE, "DISABLE")
        return

    deadline = time.monotonic() + timeout
    next_check = 0.0
    while time.monotonic() < deadline and not force_disable:
        client.pump()
        now = time.monotonic()
        if now >= next_check:
            try:
                if client.state() != MOTOR_RUN:
                    break
            except (TimeoutError, RuntimeError):
                pass
            next_check = now + 0.05

    best_effort(client, ACTION_MOTOR_DISABLE, "DISABLE")


def make_metadata(client, targets, first_hold, hold):
    pp = client.read(PARAM_MOTOR_PP, PARAM_U8)
    motor_dir = client.read(PARAM_MOTOR_DIR, PARAM_I8)
    enc_dir = client.read(PARAM_CAL_ENC_DIR, PARAM_I8)
    theta_off = client.read(PARAM_CAL_THETA_OFF, PARAM_FLOAT)
    cal_valid = client.read(PARAM_CAL_VALID, PARAM_U8)
    encoder_protocol = client.read(PARAM_ENCODER_PROTOCOL, PARAM_U8)
    encoder_ready = client.read(PARAM_ENCODER_READY, PARAM_U8)
    encoder_fault = client.read(PARAM_ENCODER_FAULT, PARAM_U8)

    return {
        "capture": "encoder_speed_chain",
        "format_version": 1,
        "created_unix_ns": time.time_ns(),
        "normal_rate_hz": NORMAL_RATE_HZ,
        "record_size": RECORD_SIZE,
        "record_struct": RECORD_FORMAT,
        "channels": [
            {
                "index": index,
                "name": name,
                "parameter_id": param_id,
                "unit": unit,
                "coordinate": coordinate,
            }
            for index, (name, param_id, unit, coordinate)
            in enumerate(CHANNELS)
        ],
        "parameters": {
            "pole_pairs": pp,
            "motor_dir": motor_dir,
            "enc_dir": enc_dir,
            "theta_off_rad": theta_off,
            "cal_valid": cal_valid,
            "encoder_protocol": encoder_protocol,
            "encoder_ready": encoder_ready,
            "encoder_fault": encoder_fault,
        },
        "plan": {
            "targets_mechanical_rad_s": targets,
            "first_hold_s": first_hold,
            "hold_s": hold,
            "host_ramp": False,
            "host_speed_settle_detection": False,
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Atomic binary capture for encoder speed-chain diagnosis"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument(
        "--target",
        type=float,
        action="append",
        dest="targets",
        help="mechanical rad/s; repeat for multiple points",
    )
    parser.add_argument(
        "--first-hold",
        type=float,
        default=12.0,
        help="seconds at the first target, including Align/I/F/handover/ramp",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=6.0,
        help="seconds at each following target",
    )
    parser.add_argument("--pre", type=float, default=0.25)
    parser.add_argument("--stop-timeout", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required because this command energizes the motor")
    if args.targets is None:
        args.targets = [80.0, 120.0, 150.0, 120.0, 80.0]
    if args.first_hold <= 0.0 or args.hold <= 0.0 or args.pre < 0.0:
        parser.error("--first-hold/--hold must be >0 and --pre must be >=0")
    if args.stop_timeout <= 0.0 or args.timeout <= 0.0:
        parser.error("timeouts must be positive")

    if args.output is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output = (
            Path(__file__).resolve().parents[1]
            / "build"
            / "captures"
            / f"encoder_speed_chain_{stamp}.axdr"
        )

    return args


def main():
    args = parse_args()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    output = args.output.resolve()
    client = None
    capture = None
    error = None

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.002,
            write_timeout=1.0,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.05)

            client = Axdr(ser, args.timeout)
            best_effort(client, ACTION_MOTOR_DISABLE, "initial DISABLE")
            try:
                client.plot_stop()
            except (TimeoutError, RuntimeError):
                pass

            try:
                metadata = make_metadata(
                    client,
                    args.targets,
                    args.first_hold,
                    args.hold,
                )
                params = metadata["parameters"]

                if params["encoder_protocol"] == 0:
                    raise RuntimeError("Encoder Protocol=None")
                if params["encoder_ready"] == 0:
                    raise RuntimeError("Encoder is not Ready")
                if params["encoder_fault"] != 0:
                    raise RuntimeError("Encoder Fault is active")
                if params["cal_valid"] == 0:
                    print(
                        "WARNING: Motor_Cal.Valid=0; sign/offset is uncalibrated.",
                        file=sys.stderr,
                    )

                capture = Capture(output, metadata)
                client.capture = capture
                client.plot_config()
                client.plot_start()

                print(f"Capture: {output}")
                print(
                    "Channels: " +
                    ", ".join(item[0] for item in CHANNELS)
                )
                print(
                    f"Pp={params['pole_pairs']} "
                    f"Motor_Dir={params['motor_dir']} "
                    f"Enc_Dir={params['enc_dir']}"
                )

                client.pump_for(args.pre)

                deadline = time.monotonic() + args.timeout
                while (
                    client.frames == 0
                    and not stop_requested
                    and time.monotonic() < deadline
                ):
                    client.pump()
                if stop_requested:
                    raise RuntimeError("Capture cancelled before motor start")
                if client.frames == 0:
                    raise RuntimeError("No valid NORMAL data before motor start")

                client.write(PARAM_MOTOR_MODE, PARAM_U8, MODE_SENSORLESS_SPEED)
                client.write(PARAM_TARGET_SPEED, PARAM_FLOAT, args.targets[0])
                client.action(ACTION_MOTOR_ENABLE)
                client.action(ACTION_MOTOR_RUN)

                print(
                    "RUN: firmware owns Align, I/F, Observer handover and ramp"
                )

                for index, target in enumerate(args.targets):
                    if stop_requested:
                        break

                    if index != 0:
                        client.write(PARAM_TARGET_SPEED, PARAM_FLOAT, target)

                    dwell = args.first_hold if index == 0 else args.hold
                    print(
                        f"[{index + 1}/{len(args.targets)}] "
                        f"target={target:+.6f} rad/s hold={dwell:.1f} s"
                    )

                    deadline = time.monotonic() + dwell
                    next_check = 0.0
                    while time.monotonic() < deadline and not stop_requested:
                        client.pump()
                        now = time.monotonic()
                        if now >= next_check:
                            if client.state() != MOTOR_RUN:
                                raise RuntimeError("motor left RUN during capture")
                            next_check = now + 0.25

                stop_motor(client, args.stop_timeout)

            except (TimeoutError, RuntimeError, ValueError) as exc:
                error = exc
                if client is not None:
                    stop_motor(client, args.stop_timeout)
            finally:
                if client is not None:
                    try:
                        client.plot_stop()
                    except (TimeoutError, RuntimeError) as exc:
                        print(f"PLOT_STOP warning: {exc}", file=sys.stderr)
                if capture is not None:
                    capture.close()

            if client is not None:
                print(
                    f"NORMAL frames={client.frames} lost={client.lost}"
                )

    except (serial.SerialException, OSError, KeyboardInterrupt) as exc:
        error = exc
        if capture is not None:
            capture.close()

    if capture is not None:
        print(
            f"Saved {capture.records} records, "
            f"{capture.records * RECORD_SIZE} data bytes -> {output}"
        )

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
