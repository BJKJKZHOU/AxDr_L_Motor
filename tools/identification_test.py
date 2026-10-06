#!/usr/bin/env python3
"""Identify an unknown motor and optionally apply results to firmware RAM.

The tool ends after Rs/Ls and optional Flux and J/B identification. It never starts a
sensorless or servo run. Motor output is disabled on normal exit, failure, and
user interruption.

Examples:
    python3 tools/identification_test.py \
        --port /dev/ttyACM1 --pole-pairs 7 --current-limit 2.0 \
        --rs-ls-count 5 --run

    python3 tools/identification_test.py \
        --port /dev/ttyACM1 --pole-pairs 7 --current-limit 2.0 --if-current 0.5 \
        --rs-ls-count 5 --apply-rl \
        --flux-forward-count 2 --flux-reverse-count 2 --apply-flux --run

    python3 tools/identification_test.py \
        --port /dev/ttyACM1 --pole-pairs 7 --current-limit 2.0 \
        --rs-ls-count 500 --run
"""

import argparse
import json
import math
from pathlib import Path
import statistics
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
MSG_PARAMETER = 0x07
MSG_EVENT = 0x08
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18

PARAM_READ = 0x01
PARAM_WRITE = 0x02

PARAM_U8 = 0
PARAM_FLOAT = 2
PARAM_U32 = 4
PARAM_ACTION = 6
PARAM_FORMAT = {
    PARAM_U8: "<B",
    PARAM_FLOAT: "<f",
    PARAM_U32: "<I",
}

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03

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

IDENT_RS_LS = 0x01
IDENT_FLUX = 0x02
IDENT_JB = 0x03
IDENT_DONE = 2
IDENT_FAILED = 3
MODE_IDENT = 4
MOTOR_DISABLED = 0

IDENT_FAIL_REASON_NAME = {
    0: "NONE",
    1: "PHASE_CURRENT",
    2: "FLUX_INTERNAL",
    3: "START_CONFIG",
    4: "JB_INTERNAL",
}

OBJECT_NAME = {
    PARAM_MOTOR_PP: "PARAM_MOTOR_PP",
    PARAM_LIMIT_I_MAX: "PARAM_LIMIT_I_MAX",
    PARAM_MOTOR_MODE: "PARAM_MOTOR_MODE",
    PARAM_TARGET_SPEED: "PARAM_TARGET_SPEED",
    PARAM_MOTOR_STATE: "PARAM_MOTOR_STATE",
    PARAM_EVENT_ERROR: "PARAM_EVENT_ERROR",
    PARAM_EVENT_TRIP: "PARAM_EVENT_TRIP",
    PARAM_IDENT_RS_LS_VALID: "PARAM_IDENT_RS_LS_VALID",
    PARAM_IDENT_RS_RESULT: "PARAM_IDENT_RS_RESULT",
    PARAM_IDENT_LS_RESULT: "PARAM_IDENT_LS_RESULT",
    PARAM_IDENT_FLUX_VALID: "PARAM_IDENT_FLUX_VALID",
    PARAM_IDENT_FLUX_RESULT: "PARAM_IDENT_FLUX_RESULT",
    PARAM_IDENT_JB_VALID: "PARAM_IDENT_JB_VALID",
    PARAM_IDENT_J_RESULT: "PARAM_IDENT_J_RESULT",
    PARAM_IDENT_B_RESULT: "PARAM_IDENT_B_RESULT",
    PARAM_IDENT_JB_EXCITE_RATIO: "PARAM_IDENT_JB_EXCITE_RATIO",
    PARAM_IDENT_JB_EXCITE_HZ: "PARAM_IDENT_JB_EXCITE_HZ",
    PARAM_IDENT_FAIL_REASON: "PARAM_IDENT_FAIL_REASON",
    PARAM_MOTOR_ALIGN_CURRENT: "PARAM_MOTOR_ALIGN_CURRENT",
    PARAM_MOTOR_IF_CURRENT: "PARAM_MOTOR_IF_CURRENT",
    ACTION_MOTOR_ENABLE: "ACTION_MOTOR_ENABLE",
    ACTION_MOTOR_DISABLE: "ACTION_MOTOR_DISABLE",
    ACTION_IDENT_RS_LS_START: "ACTION_IDENT_RS_LS_START",
    ACTION_IDENT_FLUX_START: "ACTION_IDENT_FLUX_START",
    ACTION_IDENT_JB_START: "ACTION_IDENT_JB_START",
    ACTION_IDENT_ABORT: "ACTION_IDENT_ABORT",
    ACTION_IDENT_APPLY: "ACTION_IDENT_APPLY",
    ACTION_PROTECTION_CLEAR: "ACTION_PROTECTION_CLEAR",
}

PROTECTION_NAME = {
    1 << 0: "ENCODER",
    1 << 1: "ENCODER_FIELD",
    1 << 2: "ENCODER_OVERSPEED",
    1 << 3: "OVERCURRENT",
    1 << 4: "OVERVOLTAGE",
    1 << 5: "UNDERVOLTAGE",
    1 << 6: "OVERTEMP",
    1 << 7: "PHASE_LOSS",
    1 << 8: "DRIVER",
}

HOST_CURRENT_GUARD_RATIO = 1.10
HOST_CURRENT_HARD_LIMIT_A = 5.0
HOST_CURRENT_OVER_COUNT = 5
IF_WE_RAD_S = 120.0

CANFD_LENGTHS = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)

FAST_CONFIG_ID = 13
NORMAL_CONFIG_ID = 14
FAST_RATE_HZ = 20000.0
NORMAL_RATE_HZ = 1000.0
CAPTURE_MAGIC = b"AXDRCAP1"
CURRENT_FAST_VARS = (
    ("Ia", PARAM_ADC_IA, 0.001),
    ("Ib", PARAM_ADC_IB, 0.001),
)
FAST_INDEX = {
    name: index for index, (name, _, _) in enumerate(CURRENT_FAST_VARS)
}
VBUS_NORMAL_VARS = (("Vbus", PARAM_ADC_VBUS),)
FLUX_NORMAL_VARS = VBUS_NORMAL_VARS + (
    ("Ud", PARAM_RUN_UD),
    ("Uq", PARAM_RUN_UQ),
    ("Theta_e", PARAM_RUN_THETA_E),
)
FLUX_NORMAL_UNITS = {
    "Vbus": "V",
    "Ud": "V",
    "Uq": "V",
    "Theta_e": "rad",
}
NORMAL_INDEX = {
    name: index for index, (name, _) in enumerate(FLUX_NORMAL_VARS)
}


DEFAULT_RS_LS_COUNT = 5
DEFAULT_FLUX_COUNT = 2
DEFAULT_RS_REPEAT_LIMIT = 0.15
DEFAULT_LS_REPEAT_LIMIT = 0.20


class QualityFailure(RuntimeError):
    pass


class IdentificationFailed(RuntimeError):
    def __init__(self, message, result):
        super().__init__(message)
        self.result = result


def protection_text(events):
    names = [name for bit, name in PROTECTION_NAME.items() if events & bit]
    known = sum(PROTECTION_NAME)
    unknown = events & ~known
    if unknown:
        names.append(f"UNKNOWN(0x{unknown:08X})")
    return "|".join(names) if names else "none"


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


def capture_write(path, metadata, data):
    header = json.dumps(
        metadata,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    path.write_bytes(
        CAPTURE_MAGIC + struct.pack("<I", len(header)) + header + data
    )


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


class IdentificationClient:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = StreamParser()
        self.txn = 1
        self.action_completions = []

        self.plot_started = False
        self.ident_active = False
        self.run_peak = 0.0
        self.run_trip = False
        self.run_trip_reason = None
        self.run_over_count = 0
        self.vbus = []

        self.fast_last = None
        self.fast_lost = 0
        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_vars = CURRENT_FAST_VARS
        self.normal_last = None
        self.normal_lost = 0
        self.normal_frames = 0
        self.normal_vars = VBUS_NORMAL_VARS

        self.flux_current_path = None
        self.flux_current_data = None
        self.flux_state_path = None
        self.flux_state_data = None
        self.flux_fast_sample = 0
        self.flux_normal_sample = 0

    def flux_plot_start(self, run_number, direction):
        current_path = (
            self.args.capture_dir
            / f"flux_{direction}_{run_number:02d}_current.axdr"
        )
        state_path = (
            self.args.capture_dir
            / f"flux_{direction}_{run_number:02d}_state.axdr"
        )
        current_path.parent.mkdir(parents=True, exist_ok=True)

        self.flux_current_path = current_path
        self.flux_current_data = bytearray()
        self.flux_state_path = state_path
        self.flux_state_data = bytearray()
        self.flux_fast_sample = 0
        self.flux_normal_sample = 0
        return current_path, state_path

    def flux_plot_stop(self):
        if self.flux_current_data is not None:
            capture_write(
                self.flux_current_path,
                {
                    "capture": "flux_current",
                    "channels": [
                        {"name": name, "scale": scale, "unit": "A"}
                        for name, _, scale in CURRENT_FAST_VARS
                    ],
                    "derived_channels": [
                        {
                            "expression": "-(Ia + Ib)",
                            "name": "Ic",
                            "unit": "A",
                        }
                    ],
                    "dtype": "<i2",
                    "sample_count": self.flux_fast_sample,
                    "sample_rate_hz": FAST_RATE_HZ,
                    "version": 1,
                },
                self.flux_current_data,
            )
        if self.flux_state_data is not None:
            capture_write(
                self.flux_state_path,
                {
                    "capture": "flux_state",
                    "channels": [
                        {"name": name, "unit": FLUX_NORMAL_UNITS[name]}
                        for name, _ in FLUX_NORMAL_VARS
                    ],
                    "dtype": "<f4",
                    "sample_count": self.flux_normal_sample,
                    "sample_rate_hz": NORMAL_RATE_HZ,
                    "version": 1,
                },
                self.flux_state_data,
            )
        self.flux_current_path = None
        self.flux_current_data = None
        self.flux_state_path = None
        self.flux_state_data = None

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(self.fast_vars)
        expected_length = 4 + sample_count * count * 2
        payload = canfd_payload(payload, expected_length)
        if payload is None:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected) & 0xFFFF
        self.fast_last = seq
        self.fast_frames += 1
        self.fast_samples += sample_count

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        if not self.ident_active:
            return

        if self.flux_current_data is not None:
            self.flux_current_data.extend(payload[4:expected_length])
            self.flux_fast_sample += sample_count

        ia_index = FAST_INDEX["Ia"]
        ib_index = FAST_INDEX["Ib"]
        ia_scale = self.fast_vars[ia_index][2]
        ib_scale = self.fast_vars[ib_index][2]
        for sample in range(sample_count):
            start = sample * count
            ia = raw[start + ia_index] * ia_scale
            ib = raw[start + ib_index] * ib_scale
            ic = -(ia + ib)
            peak = max(abs(ia), abs(ib), abs(ic))
            self.run_peak = max(self.run_peak, peak)
            if self.run_trip:
                continue

            if peak > HOST_CURRENT_HARD_LIMIT_A:
                self.run_trip = True
                self.run_trip_reason = (
                    "identification phase current exceeded host hard limit "
                    f"{HOST_CURRENT_HARD_LIMIT_A:.3f} A"
                )
            elif peak > self.args.ident_current_limit:
                self.run_over_count += 1
                if self.run_over_count >= HOST_CURRENT_OVER_COUNT:
                    self.run_trip = True
                    self.run_trip_reason = (
                        "identification phase current exceeded host guard "
                        f"{self.args.ident_current_limit:.3f} A for "
                        f"{HOST_CURRENT_OVER_COUNT} consecutive samples"
                    )
            else:
                self.run_over_count = 0

    def process_normal(self, payload):
        count = len(self.normal_vars)
        if len(payload) < 4 or payload[2] != NORMAL_CONFIG_ID or payload[3] != count:
            return
        expected_length = 4 + count * 4
        payload = canfd_payload(payload, expected_length)
        if payload is None:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        self.normal_frames += 1
        vbus, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(vbus)

        if not self.ident_active or self.flux_state_data is None:
            return

        self.flux_state_data.extend(payload[4:expected_length])
        self.flux_normal_sample += 1

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
            elif (
                msg_type == MSG_EVENT
                and len(payload) == 5
                and payload[0] == EVENT_ACTION_COMPLETE
            ):
                txn = payload[1]
                action_id, = struct.unpack_from("<H", payload, 2)
                self.action_completions.append(
                    (txn, action_id, payload[4])
                )
        return responses

    def request(self, msg_type, op, data=b"", context=None):
        txn = self.txn
        payload = bytes([txn, op]) + data
        self.ser.write(usb_frame(msg_type, payload))

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
                    request_name = context or f"request {msg_type}/{op}"
                    raise RuntimeError(f"{request_name}: {name}")
                self.txn = (txn % 255) + 1
                return response[4:]

        request_name = context or f"request {msg_type}/{op}"
        raise TimeoutError(f"{request_name}: timeout")

    def parameter_read(self, param_id, param_type):
        name = OBJECT_NAME.get(param_id, "parameter")
        data = self.request(
            MSG_PARAMETER,
            PARAM_READ,
            struct.pack("<H", param_id),
            context=f"read {name} (0x{param_id:04X})",
        )
        value_size = struct.calcsize(PARAM_FORMAT[param_type])
        expected_length = 3 + value_size
        data = canfd_payload(data, expected_length)
        if data is None:
            raise RuntimeError(
                f"invalid parameter response length/padding: {len(data) if data is not None else 'invalid'}"
            )

        response_id, response_type = struct.unpack_from("<HB", data, 0)
        if response_id != param_id or response_type != param_type:
            raise RuntimeError(
                "parameter response mismatch: "
                f"id=0x{response_id:04X} type={response_type}"
            )

        return struct.unpack_from(PARAM_FORMAT[param_type], data, 3)[0]

    def parameter_write(self, param_id, param_type, value):
        name = OBJECT_NAME.get(param_id, "parameter")
        encoded = struct.pack(PARAM_FORMAT[param_type], value)
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", param_id, param_type) + encoded,
            context=f"write {name} (0x{param_id:04X})",
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
        name = OBJECT_NAME.get(action_id, "action")
        txn = self.txn
        data = self.request(
            MSG_PARAMETER,
            PARAM_WRITE,
            struct.pack("<HB", action_id, PARAM_ACTION),
            context=f"trigger {name} (0x{action_id:04X})",
        )
        if len(data) != 2:
            raise RuntimeError(
                f"invalid action response length: {len(data)}"
            )
        response_id, = struct.unpack("<H", data)
        if response_id != action_id:
            raise RuntimeError(
                f"action write response mismatch: id=0x{response_id:04X}"
            )
        return txn

    def action_complete_status(self, txn, action_id):
        for index, event in enumerate(self.action_completions):
            event_txn, event_id, status = event
            if event_txn == txn and event_id == action_id:
                del self.action_completions[index]
                return status
        return None

    def prepare(self):
        self.parameter_action(ACTION_MOTOR_DISABLE)
        state = self.parameter_read(PARAM_MOTOR_STATE, PARAM_U8)
        if state != MOTOR_DISABLED:
            raise RuntimeError(
                f"motor did not enter DISABLED state: state={state}"
            )

        self.parameter_write(PARAM_MOTOR_MODE, PARAM_U8, MODE_IDENT)

        error = self.parameter_read(PARAM_EVENT_ERROR, PARAM_U32)
        trip = self.parameter_read(PARAM_EVENT_TRIP, PARAM_U32)
        if error != 0 or trip != 0:
            print(
                "Clearing latched protection before identification: "
                f"error=0x{error:08X} ({protection_text(error)}), "
                f"trip=0x{trip:08X} ({protection_text(trip)})"
            )
            self.parameter_action(ACTION_PROTECTION_CLEAR)
            error = self.parameter_read(PARAM_EVENT_ERROR, PARAM_U32)
            trip = self.parameter_read(PARAM_EVENT_TRIP, PARAM_U32)
            if error != 0 or trip != 0:
                raise RuntimeError(
                    "protection remains active after clear: "
                    f"error=0x{error:08X} ({protection_text(error)}), "
                    f"trip=0x{trip:08X} ({protection_text(trip)})"
                )

        try:
            self.request(
                MSG_PLOT,
                PLOT_STOP,
                bytes([FAST_MASK | NORMAL_MASK]),
            )
        except (TimeoutError, RuntimeError):
            pass

    def current_limit_set(self):
        self.parameter_write(
            PARAM_LIMIT_I_MAX,
            PARAM_FLOAT,
            self.args.current_limit,
        )
        print(
            f"Commission current limit={self.args.current_limit:.3f} A; "
            f"host guard={self.args.ident_current_limit:.3f} A for "
            f"{HOST_CURRENT_OVER_COUNT} samples; "
            f"hard limit={HOST_CURRENT_HARD_LIMIT_A:.3f} A"
        )

    def if_current_set(self):
        self.parameter_write(
            PARAM_MOTOR_IF_CURRENT,
            PARAM_FLOAT,
            self.args.if_current,
        )
        value = self.parameter_read(PARAM_MOTOR_IF_CURRENT, PARAM_FLOAT)
        if not math.isclose(value, self.args.if_current, rel_tol=0.0, abs_tol=1.0e-6):
            raise RuntimeError(
                f"I/F current readback {value:.6g} != {self.args.if_current:.6g}"
            )
        print(f"Flux I/F current={value:.3f} A (RAM)")
        return value

    def pole_pairs_set(self):
        self.parameter_write(
            PARAM_MOTOR_PP,
            PARAM_U8,
            self.args.pole_pairs,
        )
        value = self.parameter_read(PARAM_MOTOR_PP, PARAM_U8)
        if value != self.args.pole_pairs:
            raise RuntimeError(
                f"pole-pairs readback {value} != {self.args.pole_pairs}"
            )
        print(f"Motor pole pairs={value} (RAM)")
        return value

    def configure_plot(self, flux=False):
        self.fast_last = None
        self.normal_last = None
        self.fast_lost = 0
        self.normal_lost = 0
        self.fast_vars = CURRENT_FAST_VARS
        self.normal_vars = FLUX_NORMAL_VARS if flux else VBUS_NORMAL_VARS

        fast_data = bytes([FAST_GROUP, FAST_CONFIG_ID, len(self.fast_vars)])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id, _ in self.fast_vars
        )
        self.request(MSG_PLOT, PLOT_CONFIG, fast_data)

        normal_data = bytes(
            [NORMAL_GROUP, NORMAL_CONFIG_ID, len(self.normal_vars)]
        )
        normal_data += b"".join(
            struct.pack("<H", var_id) for _, var_id in self.normal_vars
        )
        self.request(MSG_PLOT, PLOT_CONFIG, normal_data)
        self.request(
            MSG_PLOT,
            PLOT_START,
            bytes([FAST_MASK | NORMAL_MASK]),
        )
        self.plot_started = True

    def stop_plot(self):
        if not self.plot_started:
            return
        self.request(
            MSG_PLOT,
            PLOT_STOP,
            bytes([FAST_MASK | NORMAL_MASK]),
        )
        self.plot_started = False

    def check_vbus(self):
        self.vbus.clear()
        deadline = time.monotonic() + self.args.vbus_seconds
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))

        if not self.vbus:
            raise RuntimeError("no Vbus samples")
        mean = statistics.mean(self.vbus)
        print(
            f"Vbus={mean:.3f} V "
            f"({min(self.vbus):.3f} .. {max(self.vbus):.3f} V)"
        )
        if not self.args.vbus_min <= mean <= self.args.vbus_max:
            raise RuntimeError(
                f"Vbus {mean:.3f} V outside "
                f"{self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
            )

    def ident_result(self, mode):
        result = {"state": IDENT_DONE}
        if mode == IDENT_RS_LS:
            result["valid"] = bool(
                self.parameter_read(PARAM_IDENT_RS_LS_VALID, PARAM_U8)
            )
            result["rs_ohm"] = self.parameter_read(
                PARAM_IDENT_RS_RESULT,
                PARAM_FLOAT,
            )
            result["ls_h"] = self.parameter_read(
                PARAM_IDENT_LS_RESULT,
                PARAM_FLOAT,
            )
        elif mode == IDENT_FLUX:
            result["valid"] = bool(
                self.parameter_read(PARAM_IDENT_FLUX_VALID, PARAM_U8)
            )
            result["flux_wb"] = self.parameter_read(
                PARAM_IDENT_FLUX_RESULT,
                PARAM_FLOAT,
            )
        elif mode == IDENT_JB:
            result["valid"] = bool(
                self.parameter_read(PARAM_IDENT_JB_VALID, PARAM_U8)
            )
            result["j_kgm2"] = self.parameter_read(
                PARAM_IDENT_J_RESULT, PARAM_FLOAT
            )
            result["b_nms"] = self.parameter_read(
                PARAM_IDENT_B_RESULT, PARAM_FLOAT
            )
        return result

    def ident_fail_reason(self):
        reason = self.parameter_read(PARAM_IDENT_FAIL_REASON, PARAM_U8)
        return reason, IDENT_FAIL_REASON_NAME.get(reason, f"UNKNOWN({reason})")

    def run_ident(self, mode, run_number, direction=None):
        if direction is not None:
            sign = 1.0 if direction == "forward" else -1.0
            wm_test = sign * IF_WE_RAD_S / self.args.pole_pairs
            self.parameter_write(
                PARAM_TARGET_SPEED,
                PARAM_FLOAT,
                wm_test,
            )

        if mode == IDENT_RS_LS:
            start_action = ACTION_IDENT_RS_LS_START
        elif mode == IDENT_FLUX:
            start_action = ACTION_IDENT_FLUX_START
        else:
            start_action = ACTION_IDENT_JB_START

        self.parameter_write(PARAM_MOTOR_MODE, PARAM_U8, MODE_IDENT)
        self.parameter_action(ACTION_MOTOR_ENABLE)

        self.run_peak = 0.0
        self.run_trip = False
        self.run_trip_reason = None
        self.run_over_count = 0
        fast_lost_start = self.fast_lost
        normal_lost_start = self.normal_lost
        fast_samples_start = self.fast_samples
        normal_frames_start = self.normal_frames
        start = time.monotonic()
        plot_paths = None
        if mode == IDENT_FLUX:
            plot_paths = self.flux_plot_start(run_number, direction)
        self.ident_active = True
        result = None
        failure = None

        try:
            txn = self.parameter_action(start_action)
            deadline = start + self.args.ident_timeout

            while time.monotonic() < deadline:
                self.process(self.parser.feed(self.ser.read(4096)))

                if self.run_trip:
                    failure = RuntimeError(self.run_trip_reason)
                    break

                status = self.action_complete_status(txn, start_action)
                if status is not None:
                    if status != 0:
                        reason, reason_name = self.ident_fail_reason()
                        result = {
                            "state": IDENT_FAILED,
                            "valid": False,
                            "fail_reason": reason,
                            "fail_reason_name": reason_name,
                        }
                        status_name = STATUS_NAME.get(status, str(status))
                        failure = RuntimeError(
                            f"identification completion: {status_name}; "
                            f"reason={reason_name}"
                        )
                    else:
                        result = self.ident_result(mode)
                        if not result["valid"]:
                            reason, reason_name = self.ident_fail_reason()
                            result["state"] = IDENT_FAILED
                            result["fail_reason"] = reason
                            result["fail_reason_name"] = reason_name
                            failure = RuntimeError(
                                "identification completed without a valid result; "
                                f"reason={reason_name}"
                            )
                    if failure is not None:
                        name = {
                            IDENT_RS_LS: "Rs/Ls",
                            IDENT_FLUX: "Flux",
                            IDENT_JB: "J/B",
                        }[mode]
                        failure = RuntimeError(
                            f"{name} identification failed; {failure}; "
                            f"captured phase peak={self.run_peak:.3f} A"
                        )
                    break
            else:
                failure = TimeoutError("identification timeout")

            if result is None and failure is None:
                failure = RuntimeError("identification completion missing")
        except (TimeoutError, RuntimeError) as exc:
            failure = exc
        finally:
            self.ident_active = False
            if failure is not None:
                try:
                    self.parameter_action(ACTION_IDENT_ABORT)
                except (TimeoutError, RuntimeError):
                    pass
            try:
                self.parameter_action(ACTION_MOTOR_DISABLE)
            except (TimeoutError, RuntimeError) as exc:
                print(f"DISABLE warning: {exc}", file=sys.stderr)
            self.flux_plot_stop()

        if result is None:
            result = {
                "state": -1,
                "valid": False,
            }
        result["run"] = run_number
        result["direction"] = direction
        result["time_s"] = time.monotonic() - start
        result["phase_peak_a"] = self.run_peak
        result["fast_lost"] = self.fast_lost - fast_lost_start
        result["normal_lost"] = self.normal_lost - normal_lost_start
        result["fast_samples"] = self.fast_samples - fast_samples_start
        result["normal_frames"] = self.normal_frames - normal_frames_start
        result["fast_sample_rate_hz"] = result["fast_samples"] / result["time_s"]
        result["normal_sample_rate_hz"] = result["normal_frames"] / result["time_s"]
        if plot_paths is not None:
            result["current_binary"] = str(plot_paths[0])
            result["state_binary"] = str(plot_paths[1])
            result["current_rate_hz"] = FAST_RATE_HZ
            result["state_rate_hz"] = NORMAL_RATE_HZ
        if failure is not None:
            result["error"] = str(failure)
            raise IdentificationFailed(str(failure), result)
        return result

    def apply(self):
        self.parameter_action(ACTION_IDENT_APPLY)

    def stop_all(self):
        self.ident_active = False
        self.flux_plot_stop()
        try:
            self.parameter_action(ACTION_MOTOR_DISABLE)
        except (TimeoutError, RuntimeError) as exc:
            print(f"DISABLE warning: {exc}", file=sys.stderr)
        try:
            self.request(
                MSG_PLOT,
                PLOT_STOP,
                bytes([FAST_MASK | NORMAL_MASK]),
            )
        except (TimeoutError, RuntimeError) as exc:
            print(f"PLOT_STOP warning: {exc}", file=sys.stderr)
        self.plot_started = False


def max_relative_deviation(values):
    median = statistics.median(values)
    if median <= 0.0:
        return math.inf
    return max(abs(value - median) / median for value in values)


def value_stats(values):
    if not values:
        return None

    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "max_relative_deviation": max_relative_deviation(values),
    }


def summarize_rs_ls(results, requested_count, args):
    valid = [item for item in results if item.get("valid")]
    rs = [
        item["rs_ohm"]
        for item in valid
        if math.isfinite(item["rs_ohm"]) and item["rs_ohm"] > 0.0
    ]
    ls = [
        item["ls_h"]
        for item in valid
        if math.isfinite(item["ls_h"]) and item["ls_h"] > 0.0
    ]
    reasons = []

    if len(results) != requested_count:
        reasons.append("not all requested runs were attempted")
    if len(valid) != requested_count:
        reasons.append("one or more Rs/Ls runs failed")
    if len(rs) != len(valid) or not rs:
        reasons.append("Rs contains a non-positive or non-finite result")
    if len(ls) != len(valid) or not ls:
        reasons.append("Ls contains a non-positive or non-finite result")

    rs_stats = value_stats(rs)
    ls_stats = value_stats(ls)
    if (
        rs_stats is not None
        and rs_stats["max_relative_deviation"] > args.rs_repeat_limit
    ):
        reasons.append("Rs repeatability exceeds limit")
    if (
        ls_stats is not None
        and ls_stats["max_relative_deviation"] > args.ls_repeat_limit
    ):
        reasons.append("Ls repeatability exceeds limit")

    return {
        "requested_count": requested_count,
        "attempted_count": len(results),
        "success_count": len(valid),
        "failure_count": len(results) - len(valid),
        "rs_repeatability_limit": args.rs_repeat_limit,
        "ls_repeatability_limit": args.ls_repeat_limit,
        "rs_ohm": rs_stats,
        "ls_h": ls_stats,
        "passed": not reasons,
        "reasons": reasons,
    }


def summarize_flux(results, forward_count, reverse_count, args):
    requested_count = forward_count + reverse_count
    valid = [item for item in results if item.get("valid")]
    numeric = [
        item
        for item in valid
        if math.isfinite(item["flux_wb"]) and item["flux_wb"] > 0.0
    ]
    flux = [item["flux_wb"] for item in numeric]
    forward = [
        item["flux_wb"]
        for item in numeric
        if item.get("direction") == "forward"
    ]
    reverse = [
        item["flux_wb"]
        for item in numeric
        if item.get("direction") == "reverse"
    ]
    reasons = []

    if len(results) != requested_count:
        reasons.append("not all requested runs were attempted")
    if len(valid) != requested_count:
        reasons.append("one or more Flux runs failed")
    if len(numeric) != len(valid) or not flux:
        reasons.append("Flux contains a non-positive or non-finite result")

    flux_stats = value_stats(flux)
    forward_stats = value_stats(forward)
    reverse_stats = value_stats(reverse)
    if (
        flux_stats is not None
        and flux_stats["max_relative_deviation"] > args.flux_repeat_limit
    ):
        reasons.append("Flux repeatability exceeds limit")

    direction_difference = None
    if forward_stats is not None and reverse_stats is not None:
        denominator = flux_stats["median"] if flux_stats is not None else 0.0
        if denominator > 0.0:
            direction_difference = (
                abs(forward_stats["mean"] - reverse_stats["mean"])
                / denominator
            )
        else:
            direction_difference = math.inf

        if direction_difference > args.flux_direction_limit:
            reasons.append("forward/reverse Flux difference exceeds limit")
    else:
        reasons.append("both Flux directions require a valid result")

    return {
        "requested_count": requested_count,
        "requested_forward_count": forward_count,
        "requested_reverse_count": reverse_count,
        "attempted_count": len(results),
        "success_count": len(valid),
        "failure_count": len(results) - len(valid),
        "repeatability_limit": args.flux_repeat_limit,
        "direction_consistency_limit": args.flux_direction_limit,
        "flux_wb": flux_stats,
        "forward_flux_wb": forward_stats,
        "reverse_flux_wb": reverse_stats,
        "direction_relative_difference": direction_difference,
        "passed": not reasons,
        "reasons": reasons,
    }


def result_print(name, index, count, mode, result):
    passed = result.get("valid", False)
    text = f"{name} [{index}/{count}] {'PASS' if passed else 'FAIL'}"

    if passed and mode == IDENT_RS_LS:
        text += (
            f" Rs={result['rs_ohm']:.9g} ohm"
            f" Ls={result['ls_h'] * 1.0e6:.4f} uH"
        )
    elif passed and mode == IDENT_FLUX:
        text += f" Flux={result['flux_wb']:.10g} Wb"
    elif passed and mode == IDENT_JB:
        text += (
            f" J={result['j_kgm2']:.10g} kg*m^2"
            f" B={result['b_nms']:.10g} N*m*s"
        )

    text += (
        f" time={result['time_s']:.3f} s"
        f" I_peak={result['phase_peak_a']:.3f} A"
    )
    if not passed:
        text += f" error={result.get('error', 'invalid result')}"
    print(text)


def run_group(client, mode, count, args, results, direction=None, run_offset=0):
    name = {
        IDENT_RS_LS: "Rs/Ls",
        IDENT_FLUX: f"Flux {direction}",
        IDENT_JB: "J/B",
    }[mode]
    for index in range(1, count + 1):
        run = run_offset + index
        try:
            result = client.run_ident(mode, run, direction=direction)
        except IdentificationFailed as exc:
            result = exc.result
            results.append(result)
            result_print(name, index, count, mode, result)
            if result.get("current_binary"):
                print(f"  Current binary: {result['current_binary']}")
                print(f"  State binary: {result['state_binary']}")
            if args.stop_on_error:
                return False
        else:
            results.append(result)
            result_print(name, index, count, mode, result)

        if args.interval > 0.0 and index < count:
            time.sleep(args.interval)

    return True


def print_rs_ls(summary):
    print("\nRs/Ls summary")
    print(
        f" attempted={summary['attempted_count']}/{summary['requested_count']}"
        f" success={summary['success_count']} failure={summary['failure_count']}"
    )
    print(
        " limits:"
        f" Rs={100.0 * summary['rs_repeatability_limit']:.2f}%"
        f" Ls={100.0 * summary['ls_repeatability_limit']:.2f}%"
    )
    if summary["rs_ohm"] is not None and summary["ls_h"] is not None:
        print(
            f" Rs={summary['rs_ohm']['median']:.7g} ohm"
            f" ({summary['rs_ohm']['min']:.7g} .. {summary['rs_ohm']['max']:.7g})"
        )
        print(
            f" Ls={summary['ls_h']['median'] * 1.0e6:.4f} uH"
            f" ({summary['ls_h']['min'] * 1.0e6:.4f} .."
            f" {summary['ls_h']['max'] * 1.0e6:.4f})"
        )
        print(
            " repeatability:"
            f" Rs={100.0 * summary['rs_ohm']['max_relative_deviation']:.2f}%"
            f" Ls={100.0 * summary['ls_h']['max_relative_deviation']:.2f}%"
        )
    print(f" quality={'PASS' if summary['passed'] else 'FAIL'}")
    for reason in summary["reasons"]:
        print(f"  - {reason}")


def print_flux(summary):
    print("\nFlux summary")
    print(
        f" attempted={summary['attempted_count']}/{summary['requested_count']}"
        f" success={summary['success_count']} failure={summary['failure_count']}"
    )
    if summary["flux_wb"] is not None:
        print(
            f" Flux={summary['flux_wb']['median']:.7g} Wb"
            f" ({summary['flux_wb']['min']:.7g} .. {summary['flux_wb']['max']:.7g})"
        )
        print(
            " repeatability:"
            f" {100.0 * summary['flux_wb']['max_relative_deviation']:.2f}%"
        )
    difference = summary["direction_relative_difference"]
    if difference is not None:
        print(f" direction difference={100.0 * difference:.2f}%")
    print(f" quality={'PASS' if summary['passed'] else 'FAIL'}")
    for reason in summary["reasons"]:
        print(f"  - {reason}")


def log_write(record, output):
    if output:
        path = Path(output).expanduser().resolve()
    else:
        path = (
            Path(__file__).resolve().parents[1]
            / "build"
            / "Release"
            / f"identification_test_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"Log: {path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Identify an unknown motor without starting a motor run"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument(
        "--run",
        action="store_true",
        help="required confirmation to energize the motor",
    )
    parser.add_argument("--pole-pairs", type=int, required=True)
    parser.add_argument("--current-limit", type=float, required=True)
    parser.add_argument(
        "--if-current",
        type=float,
        help="Flux I/F open-loop q-axis current in A; required when Flux identification is requested",
    )
    parser.add_argument(
        "--rs-ls-count",
        type=int,
        default=DEFAULT_RS_LS_COUNT,
        help=f"number of Rs/Ls runs, default {DEFAULT_RS_LS_COUNT}",
    )
    parser.add_argument(
        "--flux-forward-count",
        type=int,
        default=DEFAULT_FLUX_COUNT,
        help=f"number of forward Flux runs, default {DEFAULT_FLUX_COUNT}",
    )
    parser.add_argument(
        "--flux-reverse-count",
        type=int,
        default=DEFAULT_FLUX_COUNT,
        help=f"number of reverse Flux runs, default {DEFAULT_FLUX_COUNT}",
    )
    parser.add_argument(
        "--apply-rl",
        action="store_true",
        help="apply accepted Rs/Ls to firmware RAM and allow Flux identification",
    )
    parser.add_argument(
        "--skip-flux",
        action="store_true",
        help="finish after the Rs/Ls stage even when --apply-rl is used",
    )
    parser.add_argument(
        "--apply-flux",
        action="store_true",
        help="apply accepted Flux to firmware RAM",
    )
    parser.add_argument("--jb-count", type=int, default=0)
    parser.add_argument(
        "--jb-ratio", type=float, default=0.20,
        help="J/B speed excitation amplitude ratio, default 0.20",
    )
    parser.add_argument(
        "--jb-hz", type=float, default=3.0,
        help="J/B speed excitation frequency in Hz, default 3.0",
    )
    parser.add_argument(
        "--apply-jb",
        action="store_true",
        help="apply accepted J/B to firmware RAM",
    )
    parser.add_argument(
        "--rs-repeat-limit",
        type=float,
        default=DEFAULT_RS_REPEAT_LIMIT,
        help="maximum Rs deviation from median, default 0.10",
    )
    parser.add_argument(
        "--ls-repeat-limit",
        type=float,
        default=DEFAULT_LS_REPEAT_LIMIT,
        help="maximum Ls deviation from median, default 0.15",
    )
    parser.add_argument(
        "--rl-repeat-limit",
        type=float,
        help="deprecated: override both Rs and Ls repeatability limits",
    )
    parser.add_argument("--flux-repeat-limit", type=float, default=0.05)
    parser.add_argument("--flux-direction-limit", type=float, default=0.05)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument(
        "--ident-timeout",
        type=float,
        default=8.0,
        help="maximum seconds for each identification run, default 8",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="delay between runs in seconds",
    )
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    if args.rl_repeat_limit is not None:
        args.rs_repeat_limit = args.rl_repeat_limit
        args.ls_repeat_limit = args.rl_repeat_limit

    flux_requested = args.apply_rl and not args.skip_flux

    if not args.run:
        parser.error("--run is required to energize the motor")
    if not 1 <= args.pole_pairs <= 255:
        parser.error("--pole-pairs must be between 1 and 255")
    if args.current_limit <= 0.0:
        parser.error("--current-limit must be positive")
    if flux_requested and args.if_current is None:
        parser.error("--if-current is required when Flux identification is requested")
    if args.if_current is not None:
        if args.if_current <= 0.0:
            parser.error("--if-current must be positive")
        if args.if_current > args.current_limit:
            parser.error("--if-current must not exceed --current-limit")
    if args.rs_ls_count <= 0:
        parser.error("--rs-ls-count must be positive")
    if args.flux_forward_count <= 0 or args.flux_reverse_count <= 0:
        parser.error("Flux direction counts must be positive")
    if args.apply_flux and (args.skip_flux or not args.apply_rl):
        parser.error("--apply-flux requires --apply-rl and Flux identification")
    if args.jb_count < 0:
        parser.error("--jb-count must be non-negative")
    if args.jb_count > 0 and (not flux_requested or not args.apply_flux):
        parser.error("--jb-count requires --apply-rl, Flux identification and --apply-flux")
    if args.apply_jb and args.jb_count == 0:
        parser.error("--apply-jb requires a positive --jb-count")
    if not 0.0 < args.jb_ratio < 1.0:
        parser.error("--jb-ratio must be between 0 and 1 (exclusive)")
    if not math.isfinite(args.jb_hz) or args.jb_hz <= 0.0:
        parser.error("--jb-hz must be finite and positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")
    for name in (
        "rs_repeat_limit",
        "ls_repeat_limit",
        "flux_repeat_limit",
        "flux_direction_limit",
        "vbus_seconds",
        "ident_timeout",
        "timeout",
    ):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    if args.max_lost < 0:
        parser.error("--max-lost must be non-negative")

    args.ident_current_limit = (
        HOST_CURRENT_GUARD_RATIO * args.current_limit
    )
    return args


def abort(client):
    try:
        client.parameter_action(ACTION_IDENT_ABORT)
    except (TimeoutError, RuntimeError):
        pass


def main():
    args = parse_args()
    if args.output:
        log_path = Path(args.output).expanduser().resolve()
    else:
        log_path = (
            Path(__file__).resolve().parents[1]
            / "build"
            / "Release"
            / f"identification_test_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )
    args.output = str(log_path)
    args.capture_dir = log_path.parent / f"{log_path.stem}_plot"
    flux_requested = args.apply_rl and not args.skip_flux
    record = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "configuration": {
            "pole_pairs": args.pole_pairs,
            "current_limit_a": args.current_limit,
            "if_current_a": args.if_current,
            "host_current_guard_a": args.ident_current_limit,
            "host_current_guard_samples": HOST_CURRENT_OVER_COUNT,
            "host_current_hard_limit_a": HOST_CURRENT_HARD_LIMIT_A,
            "rs_ls_count": args.rs_ls_count,
            "flux_forward_count": args.flux_forward_count,
            "flux_reverse_count": args.flux_reverse_count,
            "jb_count": args.jb_count,
            "jb_ratio": args.jb_ratio,
            "jb_hz": args.jb_hz,
            "rs_repeatability_limit": args.rs_repeat_limit,
            "ls_repeatability_limit": args.ls_repeat_limit,
            "flux_repeat_limit": args.flux_repeat_limit,
            "flux_direction_limit": args.flux_direction_limit,
            "ident_timeout_s": args.ident_timeout,
            "apply_rl": args.apply_rl,
            "skip_flux": args.skip_flux,
            "apply_flux": args.apply_flux,
            "apply_jb": args.apply_jb,
            "interval_s": args.interval,
            "max_lost": args.max_lost,
            "flux_current_rate_hz": FAST_RATE_HZ,
            "flux_state_rate_hz": NORMAL_RATE_HZ,
            "flux_current_channels": [
                name for name, _, _ in CURRENT_FAST_VARS
            ],
            "flux_state_channels": [
                name for name, _ in FLUX_NORMAL_VARS
            ],
            "flux_capture_format": "AXDRCAP1",
        },
        "rs_ls": {
            "results": [],
            "applied": False,
        },
        "flux": {
            "requested": flux_requested,
            "results": [],
            "applied": False,
        },
        "jb": {
            "requested": args.jb_count > 0,
            "results": [],
            "applied": False,
        },
        "status": "running",
    }
    client = None
    error = None
    exit_code = 0

    print(
        "Unknown motor identification:"
        f" Rs/Ls={args.rs_ls_count},"
        f" Flux={args.flux_forward_count} forward +"
        f" {args.flux_reverse_count} reverse"
        f", J/B={args.jb_count}"
    )
    print("RAM updates are temporary; reset restores compiled parameters.")

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.003,
            write_timeout=1.0,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            client = IdentificationClient(ser, args)
            try:
                client.prepare()
                client.configure_plot()
                client.check_vbus()
                record["firmware_pole_pairs"] = client.pole_pairs_set()
                client.current_limit_set()
                if flux_requested:
                    record["firmware_if_current_a"] = client.if_current_set()

                print(f"\nRs/Ls identification: {args.rs_ls_count} run(s)")
                run_group(
                    client,
                    IDENT_RS_LS,
                    args.rs_ls_count,
                    args,
                    record["rs_ls"]["results"],
                )
                client.stop_plot()
                summary = summarize_rs_ls(
                    record["rs_ls"]["results"], args.rs_ls_count, args
                )
                record["rs_ls"]["summary"] = summary
                print_rs_ls(summary)
                if not summary["passed"]:
                    raise QualityFailure(
                        "Rs/Ls quality gate: " + "; ".join(summary["reasons"])
                    )

                if args.apply_rl:
                    client.apply()
                    record["rs_ls"]["applied"] = True
                    print("Rs/Ls applied to firmware RAM.")
                else:
                    record["flux"]["skip_reason"] = "Rs/Ls was not applied"
                    print("Rs/Ls not applied; Flux identification skipped.")

                if flux_requested:
                    client.configure_plot(flux=True)
                    print(
                        "\nFlux identification:"
                        f" {args.flux_forward_count} forward +"
                        f" {args.flux_reverse_count} reverse"
                    )
                    forward_complete = run_group(
                        client,
                        IDENT_FLUX,
                        args.flux_forward_count,
                        args,
                        record["flux"]["results"],
                        direction="forward",
                    )
                    if forward_complete:
                        run_group(
                            client,
                            IDENT_FLUX,
                            args.flux_reverse_count,
                            args,
                            record["flux"]["results"],
                            direction="reverse",
                            run_offset=args.flux_forward_count,
                        )
                    client.stop_plot()
                    summary = summarize_flux(
                        record["flux"]["results"],
                        args.flux_forward_count,
                        args.flux_reverse_count,
                        args,
                    )
                    record["flux"]["summary"] = summary
                    print_flux(summary)
                    if not summary["passed"]:
                        raise QualityFailure(
                            "Flux quality gate: " + "; ".join(summary["reasons"])
                        )

                    if args.apply_flux:
                        client.apply()
                        record["flux"]["applied"] = True
                        print("Flux applied to firmware RAM.")
                    else:
                        print("Flux not applied; results remain in the JSON log.")

                elif args.skip_flux:
                    record["flux"]["skip_reason"] = "disabled by --skip-flux"

                if args.jb_count > 0:
                    client.parameter_write(
                        PARAM_IDENT_JB_EXCITE_RATIO, PARAM_FLOAT, args.jb_ratio
                    )
                    client.parameter_write(
                        PARAM_IDENT_JB_EXCITE_HZ, PARAM_FLOAT, args.jb_hz
                    )
                    client.configure_plot()
                    print(f"\nJ/B identification: {args.jb_count} run(s)")
                    run_group(
                        client,
                        IDENT_JB,
                        args.jb_count,
                        args,
                        record["jb"]["results"],
                    )
                    client.stop_plot()
                    if (
                        len(record["jb"]["results"]) != args.jb_count
                        or not all(item.get("valid") for item in record["jb"]["results"])
                    ):
                        raise QualityFailure("one or more J/B runs failed")
                    if args.apply_jb:
                        client.apply()
                        record["jb"]["applied"] = True
                        print("J/B applied to firmware RAM.")
                    else:
                        print("J/B not applied; results remain in the JSON log.")

                record["status"] = "completed"
            finally:
                if record["status"] != "completed":
                    abort(client)
                client.stop_all()

    except KeyboardInterrupt:
        record["status"] = "interrupted"
        record["error"] = "KeyboardInterrupt"
        error = "interrupted by user"
        exit_code = 130
    except QualityFailure as exc:
        record["status"] = "quality_failed"
        record["error"] = str(exc)
        error = str(exc)
        exit_code = 1
    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        record["status"] = "failed"
        record["error"] = str(exc) or type(exc).__name__
        error = record["error"]
        exit_code = 1
    finally:
        record["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        try:
            log_write(record, args.output)
        except OSError as exc:
            print(f"Log warning: {exc}", file=sys.stderr)

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
