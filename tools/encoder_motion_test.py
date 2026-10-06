#!/usr/bin/env python3
"""Exercise the encoder-based position-control chain over Parameter/Action.

The test uses the current calibrated encoder coordinate and performs repeated
position moves around the starting position:

    P0 -> P0 + N turns -> P0 -> P0 - N turns -> P0

It is intended as a smoke/regression test for the complete sensed path:
encoder -> Enc_Dir/Theta_Off -> mechanical position -> position/speed/current
loops -> motor torque direction.

Example:
    python3 tools/encoder_motion_test.py \
        --port /dev/ttyACM1 \
        --turns 1 \
        --speed 5 \
        --current-limit 2.0 \
        --cycles 2 \
        --run
"""

import argparse
import math
import struct
import sys
import time

import identification_test as base

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MODE_POSITION = 2
MOTOR_DISABLED = 0
MOTOR_ENABLED = 1
MOTOR_RUN = 2
PARAM_POSITION = 5
TWO_PI = 2.0 * math.pi


class EncoderMotionTest(base.IdentificationClient):
    def position_read(self, param_id):
        name = base.OBJECT_NAME.get(param_id, "position")
        data = self.request(
            base.MSG_PARAMETER,
            base.PARAM_READ,
            struct.pack("<H", param_id),
            context=f"read {name} (0x{param_id:04X})",
        )
        data = base.canfd_payload(data, 11)
        if data is None:
            raise RuntimeError(
                "invalid position read response length/padding"
            )
        response_id, response_type = struct.unpack_from("<HB", data, 0)
        if response_id != param_id or response_type != PARAM_POSITION:
            raise RuntimeError(
                "position response mismatch: "
                f"id=0x{response_id:04X} type={response_type}"
            )
        return struct.unpack_from("<if", data, 3)

    def position_write(self, param_id, turn, theta):
        name = base.OBJECT_NAME.get(param_id, "position")
        data = self.request(
            base.MSG_PARAMETER,
            base.PARAM_WRITE,
            struct.pack("<HBif", param_id, PARAM_POSITION, turn, theta),
            context=f"write {name} (0x{param_id:04X})",
        )
        if len(data) != 2:
            raise RuntimeError(
                f"invalid position write response length: {len(data)}"
            )
        response_id, = struct.unpack("<H", data)
        if response_id != param_id:
            raise RuntimeError(
                f"position write response mismatch: id=0x{response_id:04X}"
            )

    @staticmethod
    def position_scalar(position):
        turn, theta = position
        return float(turn) * TWO_PI + float(theta)

    @staticmethod
    def scalar_position(value):
        turn = math.floor(value / TWO_PI)
        theta = value - float(turn) * TWO_PI
        if theta >= TWO_PI:
            turn += 1
            theta -= TWO_PI
        elif theta < 0.0:
            turn -= 1
            theta += TWO_PI
        return int(turn), float(theta)

    def protection_check(self):
        error = self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32)
        trip = self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32)
        if error != 0 or trip != 0:
            raise RuntimeError(
                "protection active: "
                f"error=0x{error:08X} ({base.protection_text(error)}), "
                f"trip=0x{trip:08X} ({base.protection_text(trip)})"
            )

    def encoder_check(self):
        cal_valid = self.parameter_read(base.PARAM_CAL_VALID, base.PARAM_U8)
        ready = self.parameter_read(base.PARAM_ENCODER_READY, base.PARAM_U8)
        valid = self.parameter_read(base.PARAM_ENCODER_VALID, base.PARAM_U8)
        fault = self.parameter_read(base.PARAM_ENCODER_FAULT, base.PARAM_U8)
        if cal_valid == 0:
            raise RuntimeError("Motor_Cal.Valid=0; run servo phase search first")
        if ready == 0 or valid == 0 or fault != 0:
            raise RuntimeError(
                "encoder not healthy: "
                f"ready={ready} valid={valid} fault={fault}"
            )

    def prepare(self):
        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError):
            pass

        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_DISABLED:
            raise RuntimeError(f"motor did not enter DISABLED state: state={state}")

        self.protection_check()
        self.encoder_check()

        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_POSITION)
        self.parameter_write(
            base.PARAM_LIMIT_I_MAX,
            base.PARAM_FLOAT,
            self.args.current_limit,
        )
        self.parameter_write(
            base.PARAM_TARGET_SPEED,
            base.PARAM_FLOAT,
            self.args.speed,
        )

        origin = self.position_read(base.PARAM_RUN_POSITION)
        self.position_write(base.PARAM_TARGET_POSITION, *origin)

        print(
            "Calibration/encoder ready; "
            f"origin=({origin[0]}, {origin[1]:.6f} rad)"
        )
        print(
            f"Position speed command={self.args.speed:.3f} rad/s; "
            f"current limit={self.args.current_limit:.3f} A"
        )
        return origin

    def start(self):
        self.parameter_action(base.ACTION_MOTOR_ENABLE)
        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_ENABLED:
            raise RuntimeError(f"motor enable failed: state={state}")

        self.parameter_action(base.ACTION_MOTOR_RUN)
        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_RUN:
            raise RuntimeError(f"motor run failed: state={state}")

    def move_to(self, label, target_scalar):
        start = self.position_read(base.PARAM_RUN_POSITION)
        start_scalar = self.position_scalar(start)
        target = self.scalar_position(target_scalar)
        command_delta = target_scalar - start_scalar

        self.position_write(base.PARAM_TARGET_POSITION, *target)
        print(
            f"{label}: target=({target[0]}, {target[1]:.6f} rad) "
            f"delta={command_delta / TWO_PI:+.3f} turn"
        )

        deadline = time.monotonic() + self.args.move_timeout
        settled = 0
        last_print = 0.0
        last_position = start_scalar
        max_signed_progress = 0.0
        direction = 1.0 if command_delta >= 0.0 else -1.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            self.protection_check()
            self.encoder_check()

            position = self.position_read(base.PARAM_RUN_POSITION)
            actual = self.position_scalar(position)
            wm = self.parameter_read(base.PARAM_RUN_WM, base.PARAM_FLOAT)
            error = target_scalar - actual
            signed_progress = direction * (actual - start_scalar)
            if signed_progress > max_signed_progress:
                max_signed_progress = signed_progress
            last_position = actual

            if abs(error) <= self.args.position_tolerance and abs(wm) <= self.args.speed_tolerance:
                settled += 1
                if settled >= self.args.settle_samples:
                    print(
                        f"  reached: pos=({position[0]}, {position[1]:.6f}) "
                        f"error={error:+.5f} rad wm={wm:+.4f} rad/s"
                    )
                    return
            else:
                settled = 0

            if now - last_print >= self.args.print_interval:
                print(
                    f"  pos=({position[0]}, {position[1]:.4f}) "
                    f"error={error:+.4f} rad wm={wm:+.3f} rad/s"
                )
                last_print = now

            time.sleep(self.args.poll_interval)

        raise TimeoutError(
            f"{label} timeout: remaining error={target_scalar - last_position:+.5f} rad, "
            f"forward progress={max_signed_progress / TWO_PI:.3f} turn"
        )

    def stop_disable(self):
        try:
            state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
            if state == MOTOR_RUN:
                self.parameter_action(base.ACTION_MOTOR_STOP)
        except (serial.SerialException, TimeoutError, RuntimeError) as exc:
            print(f"STOP warning: {exc}", file=sys.stderr)

        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError) as exc:
            print(f"DISABLE warning: {exc}", file=sys.stderr)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Repeated encoder position-control forward/reverse motion test"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--turns", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=5.0, help="position motion speed command magnitude, rad/s")
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--position-tolerance", type=float, default=0.03, help="settled position error, rad")
    parser.add_argument("--speed-tolerance", type=float, default=0.20, help="settled mechanical speed, rad/s")
    parser.add_argument("--settle-samples", type=int, default=3)
    parser.add_argument("--move-timeout", type=float, default=10.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--print-interval", type=float, default=0.5)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--run",
        action="store_true",
        help="required confirmation that the motor may be energized",
    )
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.turns <= 0.0:
        parser.error("--turns must be positive")
    if args.speed <= 0.0:
        parser.error("--speed must be positive")
    if args.current_limit <= 0.0:
        parser.error("--current-limit must be positive")
    if args.cycles <= 0:
        parser.error("--cycles must be positive")
    if args.position_tolerance <= 0.0 or args.speed_tolerance <= 0.0:
        parser.error("settling tolerances must be positive")
    if args.settle_samples <= 0:
        parser.error("--settle-samples must be positive")
    if args.move_timeout <= 0.0 or args.poll_interval <= 0.0 or args.print_interval <= 0.0:
        parser.error("timeouts/intervals must be positive")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    return args


def main():
    args = parse_args()

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

            test = EncoderMotionTest(ser, args)
            try:
                origin = test.prepare()
                origin_scalar = test.position_scalar(origin)
                test.start()

                span = args.turns * TWO_PI
                for cycle in range(1, args.cycles + 1):
                    print(f"\nCycle {cycle}/{args.cycles}")
                    test.move_to("forward", origin_scalar + span)
                    test.move_to("return", origin_scalar)
                    test.move_to("reverse", origin_scalar - span)
                    test.move_to("return", origin_scalar)

                print("\nEncoder position motion test PASS")
            finally:
                test.stop_disable()

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
