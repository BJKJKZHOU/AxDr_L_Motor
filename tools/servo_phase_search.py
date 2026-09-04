#!/usr/bin/env python3
"""Run servo phase search for an encoder motor over USB CDC.

For an unknown motor, first run new_motor_commission.py --identify-only and
accept both Rs/Ls and Flux results so they are applied to firmware RAM. Do not
reset the MCU between the two scripts because the applied parameters are still
RAM-only.

The script configures/validates encoder type and pole pairs, optionally overrides
the phase-search current, waits for a healthy encoder, runs PHASE_SEARCH, and
prints the complete result including Enc_Dir and Theta_Off.

Example prerequisite:
    python3 tools/new_motor_commission.py \
        --port /dev/ttyACM0 \
        --current-limit 2.0 \
        --pole-pairs 11 \
        --identify-only \
        --run

Then, without resetting the MCU:
    python3 tools/servo_phase_search.py \
        --port /dev/ttyACM0 \
        --encoder mt6835 \
        --pole-pairs 11 \
        --phase-current 0.5 \
        --run
"""

import argparse
import struct
import sys
import time

import sensorless_test as base

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MODE_PHASE_SEARCH = 6

CTRL_PHASE_STATUS = 0x10

ENCODER_TYPES = {
    "mt6816": 1,
    "mt6835": 2,
}

RESULT_NAME = {
    0: "NONE",
    1: "RUNNING",
    2: "PASS",
    3: "FAIL",
}

FAIL_NAME = {
    0: "NONE",
    1: "ENCODER",
    2: "NO_POS_MOVE",
    3: "NO_NEG_MOVE",
    4: "OFFSET_MISMATCH",
    5: "VERIFY_DIR",
    6: "ABORTED",
}


class PhaseSearch(base.SensorlessTest):
    def prepare(self):
        for op in (base.CTRL_STOP, base.CTRL_DISABLE):
            try:
                self.request(base.MSG_CONTROL, op)
            except (TimeoutError, RuntimeError):
                pass

    def read_encoder_type(self):
        return self.parameter_read(
            base.PARAM_ENCODER_TYPE, base.PARAM_U8
        )

    def set_encoder_type(self, encoder_type):
        self.parameter_write(
            base.PARAM_ENCODER_TYPE, base.PARAM_U8, encoder_type
        )

    def read_pole_pairs(self):
        return self.parameter_read(base.PARAM_MOTOR_PP, base.PARAM_U8)

    def set_pole_pairs(self, pole_pairs):
        self.parameter_write(
            base.PARAM_MOTOR_PP, base.PARAM_U8, pole_pairs
        )

    def read_phase_current(self):
        return self.parameter_read(
            base.PARAM_PHASE_I_SEARCH, base.PARAM_FLOAT
        )

    def set_phase_current(self, current):
        self.parameter_write(
            base.PARAM_PHASE_I_SEARCH, base.PARAM_FLOAT, current
        )
        return self.read_phase_current()

    def set_current_limit(self, current):
        self.parameter_write(
            base.PARAM_LIMIT_I_MAX, base.PARAM_FLOAT, current
        )

    def phase_status(self):
        data = self.request(base.MSG_CONTROL, CTRL_PHASE_STATUS)
        if len(data) != 40:
            raise RuntimeError(f"invalid phase status length: {len(data)}")

        state, fail, cal_valid, enc_dir = struct.unpack_from("<BBBb", data, 0)
        encoder_ready, encoder_fault, encoder_valid, encoder_type = data[4:8]
        values = struct.unpack_from("<8f", data, 8)
        return {
            "state": state,
            "fail": fail,
            "cal_valid": bool(cal_valid),
            "enc_dir": enc_dir,
            "encoder_ready": bool(encoder_ready),
            "encoder_fault": bool(encoder_fault),
            "encoder_valid": bool(encoder_valid),
            "encoder_type": encoder_type,
            "theta_off_pos": values[0],
            "theta_off_neg": values[1],
            "theta_off_error": values[2],
            "theta_off": values[3],
            "pos_move": values[4],
            "neg_move": values[5],
            "verify_move": values[6],
            "i_search": values[7],
        }

    def wait_encoder_ready(self):
        deadline = time.monotonic() + self.args.encoder_timeout
        last = None
        while time.monotonic() < deadline:
            status = self.phase_status()
            last = status
            if status["encoder_fault"]:
                raise RuntimeError("encoder reported fault before phase search")
            if status["encoder_ready"] and status["encoder_valid"]:
                return status
            time.sleep(self.args.poll_interval)

        raise TimeoutError(
            "encoder did not become ready/valid; "
            f"last status={last}"
        )

    def run_phase_search(self):
        self.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([MODE_PHASE_SEARCH]))
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)
        self.request(base.MSG_CONTROL, base.CTRL_RUN)

        deadline = time.monotonic() + self.args.phase_timeout
        previous = None
        last = None

        while time.monotonic() < deadline:
            status = self.phase_status()
            last = status
            state = status["state"]
            visible_state = (state, status["cal_valid"])
            if visible_state != previous:
                print(
                    "Phase state="
                    f"{RESULT_NAME.get(state, state)} "
                    f"encoder_ready={int(status['encoder_ready'])} "
                    f"cal_valid={int(status['cal_valid'])}"
                )
                previous = visible_state

            if state == 2 and status["cal_valid"]:
                return status
            if state == 3:
                return status

            time.sleep(self.args.poll_interval)

        raise TimeoutError(f"phase search timeout; last status={last}")

    def disable(self):
        try:
            self.request(base.MSG_CONTROL, base.CTRL_DISABLE)
        except (TimeoutError, RuntimeError) as exc:
            print(f"DISABLE warning: {exc}", file=sys.stderr)


def print_result(result):
    state = RESULT_NAME.get(result["state"], str(result["state"]))
    fail = FAIL_NAME.get(result["fail"], str(result["fail"]))
    print("\nServo phase result")
    print(f"  state={state} fail={fail}")
    print(
        f"  encoder: type={result['encoder_type']} "
        f"ready={int(result['encoder_ready'])} "
        f"valid={int(result['encoder_valid'])} "
        f"fault={int(result['encoder_fault'])}"
    )
    print(f"  Motor_Cal.Valid={int(result['cal_valid'])}")
    print(f"  Enc_Dir={result['enc_dir']:+d}")
    print(f"  Theta_Off_Pos={result['theta_off_pos']:+.6f} rad")
    print(f"  Theta_Off_Neg={result['theta_off_neg']:+.6f} rad")
    print(f"  Theta_Off_Error={result['theta_off_error']:+.6f} rad")
    print(f"  Theta_Off={result['theta_off']:+.6f} rad")
    print(f"  Pos_Move={result['pos_move']:+.6f} rad")
    print(f"  Neg_Move={result['neg_move']:+.6f} rad")
    print(f"  Verify_Move={result['verify_move']:+.6f} rad")
    print(f"  I_Search={result['i_search']:.3f} A")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run encoder servo phase search after unknown-motor identification"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--encoder", required=True, choices=tuple(ENCODER_TYPES))
    parser.add_argument("--pole-pairs", required=True, type=int)
    parser.add_argument(
        "--phase-current",
        type=float,
        default=None,
        help="optional phase-search current override; firmware default is used when omitted",
    )
    parser.add_argument(
        "--current-limit",
        type=float,
        default=None,
        help="optional user current limit to apply before phase search",
    )
    parser.add_argument("--encoder-timeout", type=float, default=2.0)
    parser.add_argument("--phase-timeout", type=float, default=6.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--run",
        action="store_true",
        help="required confirmation that motor identification has been applied and the motor may be energized",
    )
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if not 1 <= args.pole_pairs <= 255:
        parser.error("--pole-pairs must be in 1..255")
    if args.phase_current is not None and args.phase_current <= 0.0:
        parser.error("--phase-current must be positive")
    if args.current_limit is not None and args.current_limit <= 0.0:
        parser.error("--current-limit must be positive")
    if args.encoder_timeout <= 0.0 or args.phase_timeout <= 0.0:
        parser.error("timeouts must be positive")
    if args.poll_interval <= 0.0:
        parser.error("--poll-interval must be positive")

    return args


def main():
    args = parse_args()

    print(
        "Prerequisite: run new_motor_commission.py --identify-only and apply "
        "both Rs/Ls and Flux to firmware RAM."
    )
    print("Do not reset the MCU between identification and servo phase search.")

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

            test = PhaseSearch(ser, args)
            test.prepare()

            encoder_type = ENCODER_TYPES[args.encoder]
            test.set_encoder_type(encoder_type)
            test.set_pole_pairs(args.pole_pairs)

            if args.current_limit is not None:
                test.set_current_limit(args.current_limit)
                print(f"User current limit={args.current_limit:.3f} A")

            if args.phase_current is not None:
                phase_current = test.set_phase_current(args.phase_current)
            else:
                phase_current = test.read_phase_current()

            print(f"Encoder={args.encoder} ({encoder_type})")
            print(f"Pole pairs={test.read_pole_pairs()}")
            print(f"Configured phase-search current={phase_current:.3f} A")

            ready = test.wait_encoder_ready()
            print(
                "Encoder ready: "
                f"valid={int(ready['encoder_valid'])} "
                f"fault={int(ready['encoder_fault'])}"
            )

            try:
                result = test.run_phase_search()
            finally:
                test.disable()

            print_result(result)
            if result["state"] != 2:
                fail = FAIL_NAME.get(result["fail"], str(result["fail"]))
                raise RuntimeError(f"servo phase search failed: {fail}")

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
