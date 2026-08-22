#!/usr/bin/env python3
"""Run one guarded SENSORLESS_SPEED Align -> I/F test over USB CDC.

This test intentionally does not run motor identification. The firmware defaults
must already contain usable current-loop parameters.

Example:
    python3 tools/sensorless_test.py --port /dev/ttyACM0 --run
"""

import argparse
import math
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
MSG_PLOT = 0x04
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18
MSG_SENSORLESS = 0x06

CTRL_ENABLE = 0x01
CTRL_RUN = 0x02
CTRL_STOP = 0x03
CTRL_DISABLE = 0x04
CTRL_MODE_SET = 0x05

MODE_SENSORLESS_SPEED = 5

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03

SENSORLESS_DIR_SET = 0x01
SENSORLESS_STATUS = 0x02

FAST_GROUP = 0
NORMAL_GROUP = 1
FAST_MASK = 1 << FAST_GROUP
NORMAL_MASK = 1 << NORMAL_GROUP

FAST_VARS = (
    ("Id", 0x0010),
    ("Iq", 0x0011),
    ("Ud", 0x0012),
    ("Uq", 0x0013),
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
)
VBUS_ID = 0x0004

SENSORLESS_STAGE = {
    0: "ALIGN",
    1: "IF",
}

IF_STAGE = {
    0: "ACCEL",
    1: "HOLD",
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


class SignalStats:
    def __init__(self):
        self.count = 0
        self.sum = [0.0] * len(FAST_VARS)
        self.sum_sq = [0.0] * len(FAST_VARS)
        self.uvec_max = 0.0

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            self.sum[index] += value
            self.sum_sq[index] += value * value
        self.uvec_max = max(self.uvec_max, math.hypot(values[2], values[3]))

    def mean(self, index):
        return self.sum[index] / self.count

    def ripple_rms(self, index):
        mean = self.mean(index)
        variance = self.sum_sq[index] / self.count - mean * mean
        return math.sqrt(max(variance, 0.0))


class SensorlessTest:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = StreamParser()
        self.txn = 1

        self.motion_active = False
        self.stage = 0
        self.if_stage = 0
        self.ready = False
        self.we = 0.0

        self.start_time = None
        self.align_end = None
        self.accel_end = None
        self.ready_time = None
        self.status_log = []

        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_lost = 0
        self.fast_last = None
        self.phase_peak = 0.0
        self.tripped = False
        self.vbus = []

        self.block_sum = [0.0] * len(FAST_VARS)
        self.block_count = 0
        self.stats = {
            "ALIGN": SignalStats(),
            "ACCEL": SignalStats(),
            "HOLD": SignalStats(),
        }

    def stage_name(self):
        if self.stage == 0:
            return "ALIGN"
        if self.if_stage == 0:
            return "ACCEL"
        return "HOLD"

    def process_fast(self, payload):
        if len(payload) < 4:
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
        self.fast_samples += sample_count

        raw = struct.unpack_from(
            f"<{sample_count * channel_count}h", payload, 4
        )

        for sample in range(sample_count):
            base = sample * channel_count
            values = [raw[base + index] * 0.001
                      for index in range(channel_count)]

            if not self.motion_active:
                continue

            phase_abs = max(abs(values[4]), abs(values[5]), abs(values[6]))
            self.phase_peak = max(self.phase_peak, phase_abs)
            if phase_abs > self.args.phase_limit:
                self.tripped = True

            for index, value in enumerate(values):
                self.block_sum[index] += value
            self.block_count += 1

            if self.block_count == 20:
                block = [value / 20.0 for value in self.block_sum]
                self.stats[self.stage_name()].add(block)
                self.block_sum = [0.0] * channel_count
                self.block_count = 0

    def process_normal(self, payload):
        if len(payload) != 8 or payload[3] != 1:
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

    def configure_plot(self):
        fast_data = bytes([FAST_GROUP, 1, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id)
                              for _, var_id in FAST_VARS)
        self.request(MSG_PLOT, PLOT_CONFIG, fast_data)

        normal_data = bytes([NORMAL_GROUP, 2, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(MSG_PLOT, PLOT_CONFIG, normal_data)
        self.request(MSG_PLOT, PLOT_START, bytes([FAST_MASK | NORMAL_MASK]))

    def check_vbus(self):
        self.vbus.clear()
        deadline = time.monotonic() + self.args.vbus_seconds
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))

        if not self.vbus:
            raise RuntimeError("no Vbus samples")

        mean = sum(self.vbus) / len(self.vbus)
        print(f"Vbus={mean:.3f} V ({min(self.vbus):.3f} .. {max(self.vbus):.3f} V)")
        if mean < self.args.vbus_min:
            raise RuntimeError(
                f"Vbus {mean:.3f} V is below {self.args.vbus_min:.3f} V"
            )

    def read_status(self):
        data = self.request(MSG_SENSORLESS, SENSORLESS_STATUS)
        if len(data) != 8:
            raise RuntimeError(f"invalid Sensorless status length: {len(data)}")

        active, ready, stage, if_stage = data[:4]
        we, = struct.unpack_from("<f", data, 4)
        now = time.monotonic()

        changed = (stage != self.stage or if_stage != self.if_stage
                   or bool(ready) != self.ready)
        self.stage = stage
        self.if_stage = if_stage
        self.ready = bool(ready)
        self.we = we

        elapsed = now - self.start_time
        self.status_log.append((elapsed, we, stage, if_stage))

        if self.align_end is None and stage == 1:
            self.align_end = now
        if self.accel_end is None and stage == 1 and if_stage == 1:
            self.accel_end = now
        if self.ready_time is None and ready:
            self.ready_time = now

        if changed:
            stage_text = SENSORLESS_STAGE.get(stage, str(stage))
            if_text = IF_STAGE.get(if_stage, str(if_stage))
            print(f"t={elapsed:.3f} s stage={stage_text}/{if_text} "
                  f"We={we:.3f} rad/s ready={int(ready)}")

        if not active:
            raise RuntimeError("Sensorless stopped before test completion")

    def run(self, direction):
        self.request(MSG_CONTROL, CTRL_MODE_SET, bytes([MODE_SENSORLESS_SPEED]))
        self.request(MSG_SENSORLESS, SENSORLESS_DIR_SET, bytes([direction]))
        self.request(MSG_CONTROL, CTRL_ENABLE)

        self.start_time = time.monotonic()
        self.motion_active = True
        self.request(MSG_CONTROL, CTRL_RUN)
        print("t=0.000 s stage=ALIGN")

        next_status = self.start_time
        deadline = self.start_time + self.args.ready_timeout

        while True:
            now = time.monotonic()
            if now >= next_status:
                self.read_status()
                next_status = now + self.args.poll_interval
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if self.tripped:
                raise RuntimeError(
                    f"phase current exceeded {self.args.phase_limit:.3f} A"
                )
            if self.ready_time is not None:
                deadline = self.ready_time + self.args.hold_seconds
                if now >= deadline:
                    break
            elif now >= deadline:
                raise TimeoutError("Sensorless did not become ready")

    def stop(self):
        self.motion_active = False
        for msg_type, op, data in (
            (MSG_CONTROL, CTRL_STOP, b""),
            (MSG_CONTROL, CTRL_DISABLE, b""),
            (MSG_PLOT, PLOT_STOP, bytes([FAST_MASK | NORMAL_MASK])),
        ):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError) as exc:
                print(f"STOP warning: {exc}", file=sys.stderr)

    def print_summary(self):
        print("\nResult")
        print(f"FAST samples={self.fast_samples} frames={self.fast_frames} "
              f"seq_lost={self.fast_lost}")
        print(f"phase peak={self.phase_peak:.3f} A "
              f"limit={self.args.phase_limit:.3f} A")

        if self.align_end is not None:
            print(f"ALIGN -> IF={self.align_end - self.start_time:.3f} s")
        if self.accel_end is not None:
            print(f"ACCEL -> HOLD={self.accel_end - self.start_time:.3f} s")
        if self.ready_time is not None:
            print(f"Ready={self.ready_time - self.start_time:.3f} s")

        accel = [(elapsed, we) for elapsed, we, stage, if_stage
                 in self.status_log
                 if stage == 1 and if_stage == 0 and we > 0.0]
        if len(accel) >= 3:
            t_mean = sum(item[0] for item in accel) / len(accel)
            w_mean = sum(item[1] for item in accel) / len(accel)
            denominator = sum((item[0] - t_mean) ** 2 for item in accel)
            slope = sum((item[0] - t_mean) * (item[1] - w_mean)
                        for item in accel) / denominator
            print(f"We slope={slope:.3f} rad/s^2, HOLD={self.we:.3f} rad/s")

        for name in ("ALIGN", "ACCEL", "HOLD"):
            stats = self.stats[name]
            if stats.count == 0:
                continue
            print(
                f"{name}: Id mean={stats.mean(0):.4f} A "
                f"ripple={stats.ripple_rms(0):.4f} A, "
                f"Iq mean={stats.mean(1):.4f} A "
                f"ripple={stats.ripple_rms(1):.4f} A, "
                f"Uvec max={stats.uvec_max:.4f} V"
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded SENSORLESS_SPEED Align -> I/F USB test"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--hold-seconds", type=float, default=3.0,
                        help="time to hold after Sensorless Ready")
    parser.add_argument("--phase-limit", type=float, default=2.2,
                        help="host-side absolute phase-current limit")
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.hold_seconds < 0.0:
        parser.error("--hold-seconds must be non-negative")
    if args.phase_limit <= 0.0:
        parser.error("--phase-limit must be positive")
    if args.vbus_seconds <= 0.0:
        parser.error("--vbus-seconds must be positive")
    if args.ready_timeout <= 0.0:
        parser.error("--ready-timeout must be positive")
    if args.poll_interval <= 0.0:
        parser.error("--poll-interval must be positive")

    return args


def main():
    args = parse_args()
    direction = 1 if args.direction == "forward" else 2

    try:
        with serial.Serial(args.port, args.baud, timeout=0.003,
                           write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            test = SensorlessTest(ser, args)
            error = None
            try:
                test.configure_plot()
                test.check_vbus()
                test.run(direction)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop()
            test.print_summary()

            if error is not None:
                raise error
            if test.fast_lost:
                raise RuntimeError(f"FAST lost {test.fast_lost} frames")

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
