#!/usr/bin/env python3
"""Capture phase-current telemetry around Flux identification finish.

The firmware must already contain valid Rs/Ls parameters in Motor_Para. This
tool does not build, flash, or apply the Flux result.
"""

import argparse
import csv
from collections import deque
import json
from pathlib import Path
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial",
          file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"AXDR"
NODE_ID = 1

MSG_RESPONSE = 0x02
MSG_CONTROL = 0x03
MSG_PLOT = 0x04
MSG_IDENTIFICATION = 0x05
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18

CTRL_ENABLE = 0x01
CTRL_RUN = 0x02
CTRL_DISABLE = 0x04
CTRL_MODE_SET = 0x05

MODE_IDENT = 4

IDENT_MODE_SET = 0x01
IDENT_STATUS = 0x02
IDENT_FLUX = 0x02
IDENT_DONE = 2
IDENT_FAILED = 3

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03

FAST_GROUP = 0
NORMAL_GROUP = 1
FAST_MASK = 1 << FAST_GROUP
NORMAL_MASK = 1 << NORMAL_GROUP
FAST_CONFIG_ID = 3
NORMAL_CONFIG_ID = 4
VBUS_ID = 0x0004

FAST_RATE_HZ = 20000.0
FIRMWARE_PHASE_LIMIT_A = 2.0

FAST_VARS = (
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
    ("Id", 0x0010),
    ("Iq", 0x0011),
    ("Ud", 0x0012),
    ("Uq", 0x0013),
)

IDENT_STATE = {
    0: "IDLE",
    1: "RUNNING",
    2: "DONE",
    3: "FAILED",
}

FLUX_STAGE = {
    0: "IDLE",
    1: "ALIGN",
    2: "ACCEL",
    3: "SETTLE",
    4: "MEASURE",
    5: "CALC",
    6: "DONE",
    7: "FAILED",
    8: "FINISH",
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


def usb_frame(msg_type, payload):
    return MAGIC + struct.pack("<HB", can_id(msg_type), len(payload)) + payload


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


class FluxCapture:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = StreamParser()
        self.txn = 1

        ring_samples = max(1, round(args.pre_seconds * FAST_RATE_HZ))
        self.ring = deque(maxlen=ring_samples)
        self.capture = []
        self.capture_active = False
        self.capture_reason = None

        self.plot_started = False
        self.motion_active = False
        self.run_sample_zero = 0
        self.disable_sample = None

        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_lost = 0
        self.fast_last = None
        self.run_peak = None
        self.first_over = None
        self.host_tripped = False
        self.vbus = []
        self.status_log = []
        self.result = None

    def start_capture(self, reason):
        if self.capture_active:
            return
        self.capture = list(self.ring)
        self.capture_active = True
        self.capture_reason = reason

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        channel_count = len(FAST_VARS)
        expected = 4 + sample_count * channel_count * 2
        if len(payload) != expected:
            return

        if self.fast_last is not None:
            expected_seq = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected_seq) & 0xFFFF
        self.fast_last = seq
        self.fast_frames += 1

        raw = struct.unpack_from(
            f"<{sample_count * channel_count}h", payload, 4
        )

        for sample in range(sample_count):
            index = self.fast_samples
            base = sample * channel_count
            values = tuple(raw[base + item] * 0.001
                           for item in range(channel_count))
            phase_abs = max(abs(values[0]), abs(values[1]), abs(values[2]))
            record = (index, *values, phase_abs)
            self.fast_samples += 1
            self.ring.append(record)

            if self.capture_active:
                self.capture.append(record)

            if not self.motion_active:
                continue

            if self.run_peak is None or phase_abs > self.run_peak[-1]:
                self.run_peak = record

            if (self.first_over is None and
                    phase_abs > FIRMWARE_PHASE_LIMIT_A):
                self.first_over = record
                self.start_capture("phase_current_over_2A")

            if phase_abs > self.args.host_phase_limit:
                self.host_tripped = True

    def process_normal(self, payload):
        if (len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or
                payload[3] != 1):
            return
        value, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(value)

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
        return responses

    def pump(self):
        rx = self.ser.read(4096)
        self.process(self.parser.feed(rx))

    def request(self, msg_type, op, data=b""):
        txn = self.txn
        payload = bytes([txn, op]) + data
        self.ser.write(usb_frame(msg_type, payload))
        self.ser.flush()

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
                self.txn = (txn % 255) + 1
                return response[4:]

        raise TimeoutError(f"request {msg_type}/{op} timeout")

    def disable(self):
        self.disable_sample = self.fast_samples
        self.request(MSG_CONTROL, CTRL_DISABLE)
        self.motion_active = False

    def configure_plot(self):
        fast_data = bytes([FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id)
                              for _, var_id in FAST_VARS)
        self.request(MSG_PLOT, PLOT_CONFIG, fast_data)

        normal_data = bytes([NORMAL_GROUP, NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(MSG_PLOT, PLOT_CONFIG, normal_data)
        self.request(MSG_PLOT, PLOT_START,
                     bytes([FAST_MASK | NORMAL_MASK]))
        self.plot_started = True

    def check_vbus(self):
        self.vbus.clear()
        deadline = time.monotonic() + self.args.vbus_seconds
        while time.monotonic() < deadline:
            self.pump()

        if not self.vbus:
            raise RuntimeError("no Vbus samples")
        mean = sum(self.vbus) / len(self.vbus)
        print(f"Vbus={mean:.3f} V "
              f"({min(self.vbus):.3f} .. {max(self.vbus):.3f} V)")
        if not self.args.vbus_min <= mean <= self.args.vbus_max:
            raise RuntimeError(
                f"Vbus {mean:.3f} V outside "
                f"{self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
            )

    def read_status(self, start_time):
        data = self.request(MSG_IDENTIFICATION, IDENT_STATUS)
        if len(data) != 16:
            raise RuntimeError(f"invalid Flux status length: {len(data)}")

        mode, state, stage, valid = data[:4]
        flux, v_offset, fit_r2 = struct.unpack_from("<fff", data, 4)
        if mode != IDENT_FLUX:
            raise RuntimeError(f"unexpected identification mode: {mode}")

        elapsed = time.monotonic() - start_time
        status = {
            "time_s": elapsed,
            "sample_index": self.fast_samples,
            "state": state,
            "stage": stage,
            "valid": bool(valid),
            "flux_wb": flux,
            "v_offset_v": v_offset,
            "fit_r2": fit_r2,
        }
        previous = self.status_log[-1] if self.status_log else None
        self.status_log.append(status)
        self.result = status

        if stage == 8:
            self.start_capture("FLUX_FINISH")

        changed = (previous is None or previous["state"] != state or
                   previous["stage"] != stage or
                   previous["valid"] != bool(valid))
        if changed:
            state_name = IDENT_STATE.get(state, str(state))
            stage_name = FLUX_STAGE.get(stage, str(stage))
            print(f"t={elapsed:.3f} s sample={self.fast_samples} "
                  f"state={state_name} stage={stage_name} "
                  f"valid={valid} Flux={flux:.8g} Wb "
                  f"Voff={v_offset:.7g} V R2={fit_r2:.5f}")

        return state

    def run(self):
        self.request(MSG_CONTROL, CTRL_MODE_SET, bytes([MODE_IDENT]))
        self.request(MSG_IDENTIFICATION, IDENT_MODE_SET,
                     bytes([IDENT_FLUX]))
        self.request(MSG_CONTROL, CTRL_ENABLE)

        self.run_sample_zero = self.fast_samples
        start = time.monotonic()
        self.motion_active = True
        self.request(MSG_CONTROL, CTRL_RUN)
        print("t=0.000 s state=RUNNING")

        deadline = start + self.args.ident_timeout
        next_status = start
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                state = self.read_status(start)
                next_status = now + self.args.poll_interval
                if state == IDENT_DONE:
                    return
                if state == IDENT_FAILED:
                    raise RuntimeError("Flux identification failed")
            else:
                self.pump()

            if self.host_tripped:
                raise RuntimeError(
                    f"host phase current exceeded "
                    f"{self.args.host_phase_limit:.3f} A"
                )

        raise TimeoutError("Flux identification timeout")

    def stop_plot(self):
        if self.plot_started:
            self.request(MSG_PLOT, PLOT_STOP,
                         bytes([FAST_MASK | NORMAL_MASK]))
            self.plot_started = False


def record_dict(record, run_sample_zero):
    if record is None:
        return None
    index = record[0]
    result = {
        "sample_index": index,
        "run_time_s": (index - run_sample_zero) / FAST_RATE_HZ,
        "phase_abs_a": record[-1],
    }
    for item, (name, _) in enumerate(FAST_VARS):
        result[name.lower()] = record[item + 1]
    return result


def write_capture(test, args, error):
    base = (Path(args.output).resolve() if args.output else
            Path(__file__).resolve().parents[1] / "build" / "Release" /
            f"flux_capture_{time.strftime('%Y%m%d_%H%M%S')}")
    if base.suffix:
        base = base.with_suffix("")
    csv_path = base.with_suffix(".csv")
    json_path = base.with_suffix(".json")
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    header = ["sample_index", "run_time_s"]
    header += [name for name, _ in FAST_VARS]
    header += ["phase_abs"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for record in test.capture:
            index = record[0]
            run_time = (index - test.run_sample_zero) / FAST_RATE_HZ
            writer.writerow((index, f"{run_time:.8f}", *record[1:]))

    capture_peak = None
    if test.capture:
        capture_peak = max(test.capture, key=lambda item: item[-1])

    summary = {
        "head": git_head(),
        "error": error,
        "fast_rate_hz": FAST_RATE_HZ,
        "fast_frames": test.fast_frames,
        "fast_samples": test.fast_samples,
        "fast_lost": test.fast_lost,
        "capture_reason": test.capture_reason,
        "capture_samples": len(test.capture),
        "run_sample_zero": test.run_sample_zero,
        "disable_sample": test.disable_sample,
        "run_peak": record_dict(test.run_peak, test.run_sample_zero),
        "capture_peak": record_dict(capture_peak, test.run_sample_zero),
        "first_over_2a": record_dict(test.first_over,
                                      test.run_sample_zero),
        "status": test.status_log,
        "result": test.result,
    }
    json_path.write_text(json.dumps(summary, indent=2) + "\n",
                         encoding="utf-8")
    print(f"CSV:  {csv_path}")
    print(f"JSON: {json_path}")
    return summary


def git_head():
    head = Path(__file__).resolve().parents[1] / ".git" / "HEAD"
    try:
        value = head.read_text(encoding="utf-8").strip()
        if value.startswith("ref: "):
            ref = head.parent / value[5:]
            return ref.read_text(encoding="utf-8").strip()
        return value
    except OSError:
        return "unknown"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture FAST telemetry around Flux FINISH"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--pre-seconds", type=float, default=0.5)
    parser.add_argument("--post-disable-seconds", type=float, default=0.1)
    parser.add_argument("--poll-interval", type=float, default=0.01)
    parser.add_argument("--ident-timeout", type=float, default=25.0)
    parser.add_argument("--host-phase-limit", type=float, default=2.2)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    for name in ("pre_seconds", "post_disable_seconds", "poll_interval",
                 "ident_timeout", "host_phase_limit", "vbus_min",
                 "vbus_max", "vbus_seconds", "timeout"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")

    return args


def main():
    args = parse_args()
    error = None

    with serial.Serial(args.port, args.baud, timeout=0.003,
                       write_timeout=1.0) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        test = FluxCapture(ser, args)

        try:
            test.disable()
            print("DISABLE OK")
            test.configure_plot()
            test.check_vbus()
            test.run()
        except (serial.SerialException, OSError, TimeoutError,
                RuntimeError) as exc:
            error = str(exc)
            print(f"ERROR: {exc}", file=sys.stderr)
        finally:
            try:
                test.disable()
                print("DISABLE final OK")
            except (TimeoutError, RuntimeError) as exc:
                print(f"DISABLE warning: {exc}", file=sys.stderr)

            deadline = time.monotonic() + args.post_disable_seconds
            while time.monotonic() < deadline:
                test.pump()

            try:
                test.stop_plot()
            except (TimeoutError, RuntimeError) as exc:
                print(f"PLOT_STOP warning: {exc}", file=sys.stderr)

        summary = write_capture(test, args, error)

    peak = summary["run_peak"]
    first_over = summary["first_over_2a"]
    if peak is not None:
        print(f"Run phase peak={peak['phase_abs_a']:.3f} A "
              f"at t={peak['run_time_s']:.6f} s")
    if first_over is not None:
        print(f"First >2 A at t={first_over['run_time_s']:.6f} s: "
              f"Ia={first_over['ia']:+.3f} A "
              f"Ib={first_over['ib']:+.3f} A "
              f"Ic={first_over['ic']:+.3f} A")

    if error is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
