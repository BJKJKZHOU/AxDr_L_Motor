#!/usr/bin/env python3
"""Run guarded SENSORLESS_SPEED with NORMAL-only safety telemetry."""

import argparse
import struct
import sys
import time

import sensorless_test as base


NORMAL_CONFIG_ID = 7
NORMAL_VARS = (
    ("Vbus", 0x0004),
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
)


class FastProfileTest(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.normal_frames = 0
        self.normal_lost = 0
        self.normal_last = None

    def configure_plot(self):
        data = bytes([
            base.NORMAL_GROUP,
            NORMAL_CONFIG_ID,
            len(NORMAL_VARS),
        ])
        data += b"".join(struct.pack("<H", var_id)
                         for _, var_id in NORMAL_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, data)
        self.request(base.MSG_PLOT, base.PLOT_START,
                     bytes([base.NORMAL_MASK]))

    def process_fast(self, payload):
        _ = payload

    def process_normal(self, payload):
        expected = 4 + len(NORMAL_VARS) * 4
        if (len(payload) != expected or
                payload[2] != NORMAL_CONFIG_ID or
                payload[3] != len(NORMAL_VARS)):
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected_seq = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected_seq) & 0xFFFF
        self.normal_last = seq
        self.normal_frames += 1

        vbus, ia, ib, ic = struct.unpack_from("<4f", payload, 4)
        self.vbus.append(vbus)

        if not self.motion_active:
            return

        phase_abs = max(abs(ia), abs(ib), abs(ic))
        self.phase_peak = max(self.phase_peak, phase_abs)
        if phase_abs > self.args.phase_limit:
            self.tripped = True

    def print_profile_summary(self):
        print("\nResult")
        print(
            f"NORMAL frames={self.normal_frames} "
            f"seq_lost={self.normal_lost}"
        )
        print(
            f"phase peak={self.phase_peak:.3f} A "
            f"limit={self.args.phase_limit:.3f} A"
        )
        if self.align_end is not None:
            print(f"ALIGN -> IF={self.align_end - self.start_time:.3f} s")
        if self.accel_end is not None:
            print(f"ACCEL -> HOLD={self.accel_end - self.start_time:.3f} s")
        if self.ready_time is not None:
            print(f"Ready={self.ready_time - self.start_time:.3f} s")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded NORMAL-only fast-loop profiling run"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--phase-limit", type=float, default=2.2)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.hold_seconds <= 0.0:
        parser.error("--hold-seconds must be positive")
    if args.phase_limit <= 0.0:
        parser.error("--phase-limit must be positive")
    if args.vbus_seconds <= 0.0 or args.ready_timeout <= 0.0:
        parser.error("Vbus and ready timeouts must be positive")
    if args.poll_interval <= 0.0:
        parser.error("poll interval must be positive")
    return args


def main():
    args = parse_args()
    direction = 1 if args.direction == "forward" else 2

    try:
        with base.serial.Serial(
            args.port,
            args.baud,
            timeout=0.003,
            write_timeout=1.0,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            test = FastProfileTest(ser, args)
            error = None
            try:
                test.configure_plot()
                test.check_vbus()
                test.run(direction)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop()

            test.print_profile_summary()
            if error is not None:
                raise error
            if test.normal_lost:
                raise RuntimeError(f"NORMAL lost {test.normal_lost} frames")

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
