#!/usr/bin/env python3
"""Run servo phase search for an encoder motor over USB CDC.

The current firmware exposes servo phase search through the Parameter/Action
interface. Configure encoder/pole-pairs/search current while DISABLED, then use
MOTOR_ENABLE + MOTOR_RUN in PHASE_SEARCH mode. MOTOR_RUN completes
asynchronously when the phase search finishes.

Servo phase search establishes a self-consistent encoder/FOC coordinate:
encoder native direction -> Enc_Dir, aligned encoder zero -> Theta_Off, and
+Iq -> internal positive mechanical motion.

Example:
    python3 tools/servo_phase_search.py \
        --port /dev/ttyACM0 \
        --encoder mt6835 \
        --pole-pairs 11 \
        --phase-current 0.5 \
        --current-limit 2.0 \
        --run
"""

import argparse
import sys
import time

import identification_test as base

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MODE_PHASE_SEARCH = 6
MOTOR_DISABLED = 0
PARAM_I8 = 1
base.PARAM_FORMAT[PARAM_I8] = "<b"

ENCODER_TYPES = {
    "mt6816": 1,
    "mt6835": 2,
}


class PhaseSearch(base.IdentificationClient):
    def prepare(self):
        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError):
            pass

        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state != MOTOR_DISABLED:
            raise RuntimeError(
                f"motor did not enter DISABLED state: state={state}"
            )

        error = self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32)
        trip = self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32)
        if error != 0 or trip != 0:
            print(
                "Clearing latched protection before phase search: "
                f"error=0x{error:08X} ({base.protection_text(error)}), "
                f"trip=0x{trip:08X} ({base.protection_text(trip)})"
            )
            self.parameter_action(base.ACTION_PROTECTION_CLEAR)
            error = self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32)
            trip = self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32)
            if error != 0 or trip != 0:
                raise RuntimeError(
                    "protection remains active after clear: "
                    f"error=0x{error:08X} ({base.protection_text(error)}), "
                    f"trip=0x{trip:08X} ({base.protection_text(trip)})"
                )

    def configure(self):
        encoder_type = ENCODER_TYPES[self.args.encoder]
        self.parameter_write(base.PARAM_ENCODER_TYPE, base.PARAM_U8, encoder_type)
        self.parameter_write(base.PARAM_MOTOR_PP, base.PARAM_U8, self.args.pole_pairs)
        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_PHASE_SEARCH)

        if self.args.current_limit is not None:
            self.parameter_write(
                base.PARAM_LIMIT_I_MAX,
                base.PARAM_FLOAT,
                self.args.current_limit,
            )

        if self.args.phase_current is not None:
            self.parameter_write(
                base.PARAM_PHASE_I_SEARCH,
                base.PARAM_FLOAT,
                self.args.phase_current,
            )

        encoder_readback = self.parameter_read(base.PARAM_ENCODER_TYPE, base.PARAM_U8)
        pp_readback = self.parameter_read(base.PARAM_MOTOR_PP, base.PARAM_U8)
        phase_current = self.parameter_read(base.PARAM_PHASE_I_SEARCH, base.PARAM_FLOAT)

        if encoder_readback != encoder_type:
            raise RuntimeError(
                f"encoder type readback {encoder_readback} != {encoder_type}"
            )
        if pp_readback != self.args.pole_pairs:
            raise RuntimeError(
                f"pole-pairs readback {pp_readback} != {self.args.pole_pairs}"
            )

        print(f"Encoder={self.args.encoder} ({encoder_readback})")
        print(f"Pole pairs={pp_readback}")
        if self.args.current_limit is not None:
            print(f"User current limit={self.args.current_limit:.3f} A")
        print(f"Configured phase-search current={phase_current:.3f} A")

    def encoder_status(self):
        return {
            "ready": bool(self.parameter_read(base.PARAM_ENCODER_READY, base.PARAM_U8)),
            "valid": bool(self.parameter_read(base.PARAM_ENCODER_VALID, base.PARAM_U8)),
            "fault": bool(self.parameter_read(base.PARAM_ENCODER_FAULT, base.PARAM_U8)),
        }

    def wait_encoder_ready(self):
        deadline = time.monotonic() + self.args.encoder_timeout
        last = None
        while time.monotonic() < deadline:
            last = self.encoder_status()
            if last["fault"]:
                raise RuntimeError("encoder reported fault before phase search")
            if last["ready"] and last["valid"]:
                return last
            time.sleep(self.args.poll_interval)

        raise TimeoutError(
            "encoder did not become ready/valid; "
            f"last status={last}"
        )

    def read_result(self):
        return {
            "cal_valid": bool(self.parameter_read(base.PARAM_CAL_VALID, base.PARAM_U8)),
            "enc_dir": self.parameter_read(base.PARAM_CAL_ENC_DIR, PARAM_I8),
            "theta_off": self.parameter_read(base.PARAM_CAL_THETA_OFF, base.PARAM_FLOAT),
            "verify_move": self.parameter_read(
                base.PARAM_PHASE_VERIFY_MOVE, base.PARAM_FLOAT
            ),
            "i_search": self.parameter_read(
                base.PARAM_PHASE_I_SEARCH, base.PARAM_FLOAT
            ),
        }

    def run_phase_search(self):
        self.parameter_action(base.ACTION_MOTOR_ENABLE)
        state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
        if state == MOTOR_DISABLED:
            raise RuntimeError("motor enable did not take effect")

        txn = self.parameter_action(base.ACTION_MOTOR_RUN)
        deadline = time.monotonic() + self.args.phase_timeout

        while time.monotonic() < deadline:
            rx = self.ser.read(4096)
            self.process(self.parser.feed(rx))

            status = self.action_complete_status(txn, base.ACTION_MOTOR_RUN)
            if status is None:
                continue

            result = self.read_result()
            if status != 0:
                print_result(result)
                name = base.STATUS_NAME.get(status, str(status))
                raise RuntimeError(f"servo phase search completion: {name}")

            if not result["cal_valid"]:
                print_result(result)
                raise RuntimeError(
                    "servo phase search completed without a valid calibration"
                )
            return result

        raise TimeoutError("servo phase search timeout")

    def disable(self):
        try:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
        except (serial.SerialException, TimeoutError, RuntimeError) as exc:
            print(f"DISABLE warning: {exc}", file=sys.stderr)


def print_result(result):
    print("\nServo phase result")
    print(f"  Motor_Cal.Valid={int(result['cal_valid'])}")
    print(f"  Enc_Dir={result['enc_dir']:+d}")
    print(f"  Theta_Off={result['theta_off']:+.6f} rad")
    print(f"  Verify_Move={result['verify_move']:+.6f} rad")
    print(f"  I_Search={result['i_search']:.3f} A")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run encoder servo phase search through Parameter/Action"
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
        help="optional user current limit applied before phase search",
    )
    parser.add_argument("--encoder-timeout", type=float, default=2.0)
    parser.add_argument("--phase-timeout", type=float, default=8.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
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
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    return args


def main():
    args = parse_args()

    print(
        "Servo phase search aligns encoder direction, electrical zero and +Iq "
        "mechanical direction."
    )

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
            try:
                test.prepare()
                test.configure()

                ready = test.wait_encoder_ready()
                print(
                    "Encoder ready: "
                    f"valid={int(ready['valid'])} "
                    f"fault={int(ready['fault'])}"
                )

                result = test.run_phase_search()
                print_result(result)
            finally:
                test.disable()

    except (serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
