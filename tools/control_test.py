#!/usr/bin/env python3
"""AxDr USB CDC servo control and identification tool.

Requires pyserial:
    python -m pip install pyserial

Examples:
    python tools/control_test.py --port /dev/ttyACM0 --mode open_loop
    python tools/control_test.py --port /dev/ttyACM0 --open-loop-run --seconds 3
    python tools/control_test.py --port /dev/ttyACM0 --identify rs_ls
    python tools/control_test.py --port /dev/ttyACM0 --ident-status
    python tools/control_test.py --port /dev/ttyACM0 --ident-abort
    python tools/control_test.py --port /dev/ttyACM0 --ident-apply
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"AXDR"
NODE_ID = 1

MSG_RESPONSE = 0x02
MSG_CONTROL = 0x03
MSG_IDENTIFICATION = 0x05

CTRL_ENABLE = 0x01
CTRL_RUN = 0x02
CTRL_STOP = 0x03
CTRL_DISABLE = 0x04
CTRL_MODE_SET = 0x05

IDENT_START = 0x01
IDENT_STATUS = 0x02
IDENT_ABORT = 0x03
IDENT_APPLY = 0x04

IDENT_TYPE = {
    "rs_ls": 1,
}

IDENT_STATE = {
    0: "IDLE",
    1: "RUNNING",
    2: "DONE",
    3: "FAILED",
}

RS_LS_STAGE = {
    0: "IDLE",
    1: "PROBE_RAMP",
    2: "PROBE_MEASURE",
    3: "ALIGN",
    4: "RAMP",
    5: "SETTLE",
    6: "MEASURE_A",
    7: "MEASURE_B",
    8: "DONE",
    9: "FAILED",
}

MODE = {
    "torque": 0,
    "speed": 1,
    "position": 2,
    "open_loop": 3,
}

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


def can_id(msg_type):
    return (msg_type << 6) | NODE_ID


def usb_frame(msg_id, payload):
    return MAGIC + struct.pack("<HB", msg_id, len(payload)) + payload


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

            payload = bytes(self.buf[7:frame_len])
            del self.buf[:frame_len]
            frames.append((msg_id, payload))

        return frames


def request(ser, parser, txn, msg_type, op, data=b"", timeout=1.0):
    payload = bytes([txn, op]) + data
    ser.write(usb_frame(can_id(msg_type), payload))
    ser.flush()

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx = ser.read(128)
        if not rx:
            continue

        for msg_id, response in parser.feed(rx):
            rx_type = (msg_id >> 6) & 0x1F
            if rx_type != MSG_RESPONSE or len(response) < 4:
                continue

            rx_txn, req_msg, req_op, status = response[:4]
            if rx_txn != txn or req_msg != msg_type or req_op != op:
                continue

            if status != 0:
                raise RuntimeError(STATUS_NAME.get(status, str(status)))

            return (txn % 255) + 1, response[4:]

    raise TimeoutError("response timeout")


def control(ser, parser, txn, op, value=None, timeout=1.0):
    data = b"" if value is None else bytes([value])
    txn, _ = request(ser, parser, txn, MSG_CONTROL, op, data, timeout)
    return txn


def ident_status(ser, parser, txn, timeout=1.0):
    txn, data = request(ser, parser, txn,
                        MSG_IDENTIFICATION, IDENT_STATUS,
                        timeout=timeout)

    if len(data) != 12:
        raise RuntimeError(f"invalid identification status length: {len(data)}")

    mode, state, stage, valid = data[:4]
    rs, ls = struct.unpack_from("<ff", data, 4)

    return txn, {
        "mode": mode,
        "state": state,
        "stage": stage,
        "valid": bool(valid),
        "rs": rs,
        "ls": ls,
    }


def print_ident_status(status):
    state = IDENT_STATE.get(status["state"], str(status["state"]))
    stage = RS_LS_STAGE.get(status["stage"], str(status["stage"]))
    print(f"IDENT state={state} stage={stage} valid={int(status['valid'])} "
          f"Rs={status['rs']:.6g} ohm Ls={status['ls']:.6g} H")


def main():
    ap = argparse.ArgumentParser(description="AxDr USB CDC servo control tool")
    ap.add_argument("--port", required=True, help="CDC serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=MODE)
    ap.add_argument("--enable", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--stop", action="store_true")
    ap.add_argument("--disable", action="store_true")
    ap.add_argument("--open-loop-run", action="store_true",
                    help="MODE_SET OPEN_LOOP -> ENABLE -> RUN")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="with --open-loop-run, hold before STOP -> DISABLE")
    ap.add_argument("--identify", choices=IDENT_TYPE,
                    help="start identification and wait for completion")
    ap.add_argument("--ident-status", action="store_true",
                    help="read identification status")
    ap.add_argument("--ident-abort", action="store_true",
                    help="abort active identification")
    ap.add_argument("--ident-apply", action="store_true",
                    help="apply valid identification result to RAM parameters")
    ap.add_argument("--poll-interval", type=float, default=0.1,
                    help="identification status polling interval in seconds")
    ap.add_argument("--timeout", type=float, default=1.0)
    args = ap.parse_args()

    parser = StreamParser()
    txn = 1

    try:
        with serial.Serial(args.port, args.baud, timeout=0.02,
                           write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            if args.identify is not None:
                ident = IDENT_TYPE[args.identify]
                txn, _ = request(ser, parser, txn,
                                 MSG_IDENTIFICATION, IDENT_START,
                                 bytes([ident]), args.timeout)
                print(f"IDENT_START {args.identify.upper()} OK")

                last = None
                while True:
                    txn, status = ident_status(ser, parser, txn, args.timeout)
                    current = (status["state"], status["stage"])
                    if current != last:
                        print_ident_status(status)
                        last = current

                    if status["state"] == 2:
                        print_ident_status(status)
                        return
                    if status["state"] == 3:
                        print_ident_status(status)
                        raise RuntimeError("identification failed")

                    time.sleep(args.poll_interval)

            if args.ident_status:
                txn, status = ident_status(ser, parser, txn, args.timeout)
                print_ident_status(status)

            if args.ident_abort:
                txn, _ = request(ser, parser, txn,
                                 MSG_IDENTIFICATION, IDENT_ABORT,
                                 timeout=args.timeout)
                print("IDENT_ABORT OK")

            if args.ident_apply:
                txn, _ = request(ser, parser, txn,
                                 MSG_IDENTIFICATION, IDENT_APPLY,
                                 timeout=args.timeout)
                print("IDENT_APPLY OK")

            if args.open_loop_run:
                txn = control(ser, parser, txn, CTRL_MODE_SET,
                              MODE["open_loop"], args.timeout)
                print("MODE_SET OPEN_LOOP OK")
                txn = control(ser, parser, txn, CTRL_ENABLE,
                              timeout=args.timeout)
                print("ENABLE OK")
                txn = control(ser, parser, txn, CTRL_RUN,
                              timeout=args.timeout)
                print("RUN OK")

                if args.seconds > 0.0:
                    time.sleep(args.seconds)
                    txn = control(ser, parser, txn, CTRL_STOP,
                                  timeout=args.timeout)
                    print("STOP OK")
                    control(ser, parser, txn, CTRL_DISABLE,
                            timeout=args.timeout)
                    print("DISABLE OK")
                return

            if args.mode is not None:
                txn = control(ser, parser, txn, CTRL_MODE_SET,
                              MODE[args.mode], args.timeout)
                print(f"MODE_SET {args.mode.upper()} OK")

            if args.enable:
                txn = control(ser, parser, txn, CTRL_ENABLE,
                              timeout=args.timeout)
                print("ENABLE OK")

            if args.run:
                txn = control(ser, parser, txn, CTRL_RUN,
                              timeout=args.timeout)
                print("RUN OK")

            if args.stop:
                txn = control(ser, parser, txn, CTRL_STOP,
                              timeout=args.timeout)
                print("STOP OK")

            if args.disable:
                control(ser, parser, txn, CTRL_DISABLE,
                        timeout=args.timeout)
                print("DISABLE OK")

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
