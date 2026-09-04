#!/usr/bin/env python3
"""Minimal AxDr USB CDC plot protocol tester.

Requires pyserial:
    python -m pip install pyserial

Examples:
    python tools/plot_test.py --port /dev/ttyACM0 --fast Ia Ib Ic
    python tools/plot_test.py --port /dev/ttyACM0 --normal Wm Theta_m Vbus Iq
    python tools/plot_test.py --port /dev/ttyACM0 \
        --fast Ia Ib Ic --normal Wm Theta_m Vbus Iq --seconds 10

All USB traffic is binary. Variable names are only a PC-side convenience and are
translated to the generated AxDr parameter ID table before transmission.
"""

import argparse
import struct
import sys
import time

from parameter_ids_generated import (
    PARAM_ADC_IA,
    PARAM_ADC_IB,
    PARAM_ADC_IC,
    PARAM_ADC_VBUS,
    PARAM_RUN_ID,
    PARAM_RUN_IQ,
    PARAM_RUN_UD,
    PARAM_RUN_UQ,
    PARAM_RUN_THETA_E,
    PARAM_RUN_THETA_M,
    PARAM_RUN_WM,
)

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

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03

PLOT_FAST = 0
PLOT_NORMAL = 1
PLOT_FAST_MASK = 1 << PLOT_FAST
PLOT_NORMAL_MASK = 1 << PLOT_NORMAL

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

VAR = {
    "ia": (PARAM_ADC_IA, 0.001),
    "ib": (PARAM_ADC_IB, 0.001),
    "ic": (PARAM_ADC_IC, 0.001),
    "vbus": (PARAM_ADC_VBUS, None),
    "id": (PARAM_RUN_ID, 0.001),
    "iq": (PARAM_RUN_IQ, 0.001),
    "ud": (PARAM_RUN_UD, 0.001),
    "uq": (PARAM_RUN_UQ, 0.001),
    "theta_e": (PARAM_RUN_THETA_E, 0.0002),
    "theta_m": (PARAM_RUN_THETA_M, 0.0002),
    "wm": (PARAM_RUN_WM, 0.1),
}

VAR_ID_NAME = {value[0]: name for name, value in VAR.items()}


def can_id(msg_type):
    return (msg_type << 6) | NODE_ID


def usb_frame(msg_id, payload):
    if len(payload) > 64:
        raise ValueError("AxDr payload exceeds 64 bytes")
    return MAGIC + struct.pack("<HB", msg_id, len(payload)) + payload


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data)


def parse_vars(names, fast):
    result = []
    for name in names or []:
        key = name.lower()
        if key not in VAR:
            known = ", ".join(sorted(VAR))
            raise ValueError(f"unknown variable '{name}', known: {known}")
        var_id, scale = VAR[key]
        if fast and scale is None:
            raise ValueError(f"{name} is not available in FAST plot")
        result.append((name, var_id, scale))
    return result


def config_payload(txn, group, config_id, variables):
    payload = bytearray([txn, PLOT_CONFIG, group, config_id, len(variables)])
    for _, var_id, _ in variables:
        payload += struct.pack("<H", var_id)
    return bytes(payload)


def start_stop_payload(txn, op, mask):
    return bytes([txn, op, mask])


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
            if msg_id > 0x07FF or length > 64:
                del self.buf[0]
                continue

            frame_len = 7 + length
            if len(self.buf) < frame_len:
                break

            raw = bytes(self.buf[:frame_len])
            payload = bytes(self.buf[7:frame_len])
            del self.buf[:frame_len]
            frames.append((msg_id, payload, raw))

        return frames


def send(ser, msg_id, payload, label):
    raw = usb_frame(msg_id, payload)
    ser.write(raw)
    ser.flush()
    print(f"TX {label}: {hex_bytes(raw)}")


def wait_response(ser, parser, txn, req_op, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = ser.read(256)
        if not data:
            continue
        for msg_id, payload, raw in parser.feed(data):
            msg_type = (msg_id >> 6) & 0x1F
            if msg_type != MSG_RESPONSE:
                continue
            print(f"RX RESPONSE: {hex_bytes(raw)}")
            if len(payload) < 4:
                continue
            rx_txn, req_msg, rx_op, status = payload[:4]
            if rx_txn != txn or req_msg != MSG_PLOT or rx_op != req_op:
                continue
            return status, payload
    raise TimeoutError("response timeout")


def check_config_response(payload, group, config_id, variables):
    if len(payload) < 7:
        raise RuntimeError("CONFIG response is too short")

    rx_group = payload[4]
    rx_config = payload[5]
    count = payload[6]
    expected_len = 7 + count * 2
    if len(payload) != expected_len:
        raise RuntimeError("CONFIG response length mismatch")

    ids = list(struct.unpack_from(f"<{count}H", payload, 7)) if count else []
    expected_ids = [item[1] for item in variables]
    if rx_group != group or rx_config != config_id or ids != expected_ids:
        raise RuntimeError(
            f"CONFIG echo mismatch: group={rx_group}, config={rx_config}, ids={ids}"
        )

    names = [VAR_ID_NAME.get(var_id, f"0x{var_id:04X}") for var_id in ids]
    print(
        f"CONFIG OK: group={'FAST' if group == PLOT_FAST else 'NORMAL'} "
        f"config={config_id} vars={names}"
    )


def seq_update(last, seq):
    if last is None:
        return seq, 0
    expected = (last + 1) & 0xFFFF
    lost = (seq - expected) & 0xFFFF
    return seq, lost


def run_data(ser, parser, seconds, fast_vars, normal_vars, hex_data):
    end = time.monotonic() + seconds
    fast_frames = 0
    fast_samples = 0
    fast_lost = 0
    fast_last = None
    normal_frames = 0
    normal_lost = 0
    normal_last = None
    shown_fast = 0
    shown_normal = 0

    while time.monotonic() < end:
        data = ser.read(512)
        if not data:
            continue

        for msg_id, payload, raw in parser.feed(data):
            msg_type = (msg_id >> 6) & 0x1F

            if msg_type == MSG_FAST_DATA:
                if len(payload) < 4:
                    continue
                seq, = struct.unpack_from("<H", payload, 0)
                sample_count = payload[3]
                channel_count = len(fast_vars)
                expected = 4 + sample_count * channel_count * 2
                if channel_count == 0 or len(payload) != expected:
                    print(f"FAST malformed: {hex_bytes(raw)}")
                    continue

                fast_last, lost = seq_update(fast_last, seq)
                fast_lost += lost
                fast_frames += 1
                fast_samples += sample_count

                if shown_fast < hex_data:
                    print(f"RX FAST seq={seq} samples={sample_count}: {hex_bytes(raw)}")
                    shown_fast += 1

            elif msg_type == MSG_NORMAL_DATA:
                if len(payload) < 4:
                    continue
                seq, = struct.unpack_from("<H", payload, 0)
                count = payload[3]
                if len(payload) != 4 + count * 4:
                    print(f"NORMAL malformed: {hex_bytes(raw)}")
                    continue

                normal_last, lost = seq_update(normal_last, seq)
                normal_lost += lost
                normal_frames += 1

                if shown_normal < hex_data:
                    values = struct.unpack_from(f"<{count}f", payload, 4) if count else ()
                    print(
                        f"RX NORMAL seq={seq} values={tuple(round(v, 6) for v in values)}: "
                        f"{hex_bytes(raw)}"
                    )
                    shown_normal += 1

            elif msg_type == MSG_RESPONSE:
                print(f"RX RESPONSE: {hex_bytes(raw)}")

    return fast_frames, fast_samples, fast_lost, normal_frames, normal_lost


def main():
    ap = argparse.ArgumentParser(description="AxDr USB CDC plot protocol tester")
    ap.add_argument("--port", required=True, help="CDC serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=115200, help="CDC line coding; USB CDC transport is not UART baud-limited")
    ap.add_argument("--fast", nargs="*", default=[], metavar="VAR", help="FAST variables, e.g. Ia Ib Ic")
    ap.add_argument("--normal", nargs="*", default=[], metavar="VAR", help="NORMAL variables, e.g. Wm Theta_m Vbus Iq")
    ap.add_argument("--seconds", type=float, default=5.0, help="stream duration")
    ap.add_argument("--hex-data", type=int, default=3, help="number of FAST/NORMAL data frames to print as hex")
    ap.add_argument("--timeout", type=float, default=1.0, help="command response timeout")
    args = ap.parse_args()

    if not args.fast and not args.normal:
        ap.error("at least one of --fast or --normal is required")

    try:
        fast_vars = parse_vars(args.fast, True)
        normal_vars = parse_vars(args.normal, False)
    except ValueError as exc:
        ap.error(str(exc))

    parser = StreamParser()
    txn = 1
    mask = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            if fast_vars:
                payload = config_payload(txn, PLOT_FAST, 1, fast_vars)
                send(ser, can_id(MSG_PLOT), payload, "PLOT_CONFIG FAST")
                status, response = wait_response(ser, parser, txn, PLOT_CONFIG, args.timeout)
                if status != 0:
                    raise RuntimeError(f"FAST CONFIG failed: {STATUS_NAME.get(status, status)}")
                check_config_response(response, PLOT_FAST, 1, fast_vars)
                mask |= PLOT_FAST_MASK
                txn = (txn % 255) + 1

            if normal_vars:
                payload = config_payload(txn, PLOT_NORMAL, 2, normal_vars)
                send(ser, can_id(MSG_PLOT), payload, "PLOT_CONFIG NORMAL")
                status, response = wait_response(ser, parser, txn, PLOT_CONFIG, args.timeout)
                if status != 0:
                    raise RuntimeError(f"NORMAL CONFIG failed: {STATUS_NAME.get(status, status)}")
                check_config_response(response, PLOT_NORMAL, 2, normal_vars)
                mask |= PLOT_NORMAL_MASK
                txn = (txn % 255) + 1

            send(ser, can_id(MSG_PLOT), start_stop_payload(txn, PLOT_START, mask), "PLOT_START")
            status, _ = wait_response(ser, parser, txn, PLOT_START, args.timeout)
            if status != 0:
                raise RuntimeError(f"PLOT_START failed: {STATUS_NAME.get(status, status)}")
            print("PLOT_START OK")
            txn = (txn % 255) + 1

            try:
                stats = run_data(
                    ser, parser, args.seconds, fast_vars, normal_vars, args.hex_data
                )
            finally:
                send(ser, can_id(MSG_PLOT), start_stop_payload(txn, PLOT_STOP, mask), "PLOT_STOP")
                try:
                    status, _ = wait_response(ser, parser, txn, PLOT_STOP, args.timeout)
                    print(f"PLOT_STOP {STATUS_NAME.get(status, status)}")
                except TimeoutError:
                    print("PLOT_STOP response timeout")

            fast_frames, fast_samples, fast_lost, normal_frames, normal_lost = stats
            print("\nSummary")
            if fast_vars:
                print(
                    f"FAST: frames={fast_frames} samples={fast_samples} "
                    f"seq_lost={fast_lost}"
                )
            if normal_vars:
                print(f"NORMAL: frames={normal_frames} seq_lost={normal_lost}")

            if fast_lost or normal_lost:
                raise SystemExit(1)

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
