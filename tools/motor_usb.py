#!/usr/bin/env python3
"""USB Parameter/Action/Event transport used by motor commissioning and operation."""

import struct
import sys
import time

from parameter_ids_generated import *  # generated Parameter object IDs

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"AXDR"
NODE_ID = 1
MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18
MSG_PARAMETER = 0x07
MSG_EVENT = 0x08
PARAM_READ = 0x01
PARAM_WRITE = 0x02
PARAM_U8 = 0
PARAM_I8 = 1
PARAM_FLOAT = 2
PARAM_I32 = 3
PARAM_U32 = 4
PARAM_POSITION = 5
PARAM_ACTION = 6
PARAM_FORMAT = {
    PARAM_U8: "<B",
    PARAM_I8: "<b",
    PARAM_FLOAT: "<f",
    PARAM_I32: "<i",
    PARAM_U32: "<I",
    PARAM_POSITION: "<if",
}
PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03
PLOT_CAPS = 0x04
EVENT_ACTION_COMPLETE = 0x03
FAST_GROUP = 0
NORMAL_GROUP = 1
FAST_MASK = 1 << FAST_GROUP
NORMAL_MASK = 1 << NORMAL_GROUP
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
CANFD_LENGTHS = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)


def can_id(msg_type):
    return (msg_type << 6) | NODE_ID


def canfd_length(length):
    if length < 0 or length > 64:
        raise ValueError(f"CAN FD payload length out of range: {length}")
    for candidate in CANFD_LENGTHS:
        if length <= candidate:
            return candidate
    raise ValueError(f"unsupported CAN FD payload length: {length}")


def canfd_payload(payload, expected_length):
    if len(payload) < expected_length:
        return None
    if any(payload[expected_length:]):
        return None
    return payload[:expected_length]


def usb_frame(msg_type, payload):
    frame_length = canfd_length(len(payload))
    padded = payload + bytes(frame_length - len(payload))
    return MAGIC + struct.pack("<HB", can_id(msg_type), frame_length) + padded


def plot_capabilities(request):
    """Read the current firmware's paginated Plot channel and timing contract."""
    start = 0
    channels = {}
    header = None
    while True:
        payload = request(MSG_PLOT, PLOT_CAPS, bytes([start]))
        if len(payload) < 14:
            raise RuntimeError("short Plot capability response")
        page, following, count = payload[:3]
        data = canfd_payload(payload, 14 + count * 7)
        if data is None or page != start or not 1 <= count <= 6:
            raise RuntimeError("invalid Plot capability page")
        current = struct.unpack_from("<BBBII", data, 3)
        if header is not None and current != header:
            raise RuntimeError("Plot capabilities changed between pages")
        header = current
        for index in range(count):
            pid, modes, scale = struct.unpack_from("<HBf", data, 14 + 7 * index)
            if pid in channels:
                raise RuntimeError("duplicate Plot capability")
            channels[pid] = (modes, scale)
        if following == 0xFF:
            break
        if following != start + count or following <= start:
            raise RuntimeError("invalid Plot capability pagination")
        start = following
    fast_max, normal_max, block, fast_hz, normal_hz = header
    if not all(header):
        raise RuntimeError("invalid Plot timing or channel limits")
    return {"rates": {"fast": fast_hz, "normal": normal_hz},
            "max_channels": {"fast": fast_max, "normal": normal_max},
            "block_samples": block, "channels": channels}


class StreamParser:
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

            frame_len = 7 + length
            if len(self.buf) < frame_len:
                break

            payload = bytes(self.buf[7:frame_len])
            del self.buf[:frame_len]
            frames.append((msg_id, payload))

        return frames


class MotorUSB:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = StreamParser()
        self.txn = 1
        self.action_completions = []
        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_lost = 0
        self.fast_last = None

    def process(self, frames):
        responses = []
        for msg_id, payload in frames:
            msg_type = (msg_id >> 6) & 0x1F
            if msg_type == MSG_FAST_DATA:
                self.process_fast(payload)
            elif msg_type == MSG_NORMAL_DATA:
                self.process_normal(payload)
            elif msg_type == MSG_RESPONSE:
                responses.append(payload)
            elif (msg_type == MSG_EVENT and len(payload) == 5 and
                  payload[0] == EVENT_ACTION_COMPLETE):
                txn = payload[1]
                action_id, = struct.unpack_from("<H", payload, 2)
                self.action_completions.append((txn, action_id, payload[4]))
        return responses

    def request(self, msg_type, op, data=b""):
        txn = self.txn
        self.txn = (txn % 255) + 1
        payload = bytes([txn, op]) + data
        self.ser.write(usb_frame(msg_type, payload))

        # Do not call Serial.flush() here. On POSIX it drains the TTY output
        # queue and may block independently of pyserial's write_timeout. A
        # request already has two bounded waits: write_timeout for write() and
        # args.timeout below for the matching protocol response.
        deadline = time.monotonic() + self.args.timeout
        while time.monotonic() < deadline:
            rx = self.ser.read(4096)
            for response in self.process(self.parser.feed(rx)):
                if len(response) < 4:
                    continue
                rx_txn, req_msg, req_op, status = response[:4]
                if rx_txn != txn or req_msg != msg_type or req_op != op:
                    continue
                if status != 0:
                    name = STATUS_NAME.get(status, str(status))
                    raise RuntimeError(f"request {msg_type}/{op}: {name}")
                return response[4:]

        raise TimeoutError(f"request {msg_type}/{op} timeout")

    def parameter_read(self, param_id, param_type):
        data = self.request(MSG_PARAMETER, PARAM_READ,
                            struct.pack("<H", param_id))
        value_size = struct.calcsize(PARAM_FORMAT[param_type])
        data = canfd_payload(data, 3 + value_size)
        if data is None:
            raise RuntimeError("invalid parameter response length/padding")

        response_id, response_type = struct.unpack_from("<HB", data, 0)
        if response_id != param_id or response_type != param_type:
            raise RuntimeError(
                f"parameter response mismatch: "
                f"id=0x{response_id:04X} type={response_type}"
            )

        values = struct.unpack_from(PARAM_FORMAT[param_type], data, 3)
        return values if param_type == PARAM_POSITION else values[0]

    def parameter_write(self, param_id, param_type, value):
        if param_type == PARAM_ACTION:
            raise ValueError("use parameter_action() for actions")
        values = value if param_type == PARAM_POSITION else (value,)
        encoded = struct.pack(PARAM_FORMAT[param_type], *values)
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", param_id, param_type) + encoded,
        )
        if len(data) != 2:
            raise RuntimeError(
                f"invalid parameter write response length: {len(data)}"
            )
        response_id, = struct.unpack("<H", data)
        if response_id != param_id:
            raise RuntimeError(
                f"parameter write response mismatch: id=0x{response_id:04X}"
            )

    def parameter_action(self, action_id):
        txn = self.txn
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", action_id, PARAM_ACTION),
        )
        if len(data) != 2:
            raise RuntimeError(
                f"invalid action response length: {len(data)}"
            )
        response_id, = struct.unpack("<H", data)
        if response_id != action_id:
            raise RuntimeError(
                f"action response mismatch: id=0x{response_id:04X}"
            )
        return txn

    def action_complete_status(self, txn, action_id):
        for index, event in enumerate(self.action_completions):
            event_txn, event_id, status = event
            if event_txn == txn and event_id == action_id:
                del self.action_completions[index]
                return status
        return None

    def speed_set(self, wm):
        self.parameter_write(PARAM_TARGET_SPEED, PARAM_FLOAT, wm)

