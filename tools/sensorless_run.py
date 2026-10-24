#!/usr/bin/env python3
"""Run sensorless speed using the current firmware's trajectory and protection.

Example:
    python3 tools/sensorless_run.py --port /dev/ttyACM0 --run --rpm 1000

Ctrl+C or the requested duration sends STOP, waits for firmware deceleration,
then disables the power stage. A second interrupt requests immediate disable.
"""

import argparse
import math
import signal
import sys
import time

import motor_usb as base

MODE_SENSORLESS_SPEED = 5
MOTOR_RUN = 2


class StopRequest:
    def __init__(self):
        self.count = 0

    @property
    def requested(self):
        return self.count > 0

    @property
    def force(self):
        return self.count > 1

    def handle(self, signum, frame):
        self.count += 1
        print("\nStopping..." if self.count == 1 else "\nDisabling...")


class SensorlessRun(base.MotorUSB):
    def process_fast(self, payload):
        pass

    def process_normal(self, payload):
        pass

    def prepare(self):
        self.parameter_action(base.ACTION_MOTOR_DISABLE)
        self.request(base.MSG_PLOT, base.PLOT_STOP,
                     bytes([base.FAST_MASK | base.NORMAL_MASK]))

    def __init__(self, ser, args, stop):
        super().__init__(ser, args)
        self.stop_request = stop
        self.last_status = None

    def snapshot(self):
        self.last_status = {
            "state": self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8),
            "error": self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32),
            "trip": self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32),
            "wm": self.parameter_read(base.PARAM_RUN_WM, base.PARAM_FLOAT),
            "wm_ref": self.parameter_read(base.PARAM_REF_WM, base.PARAM_FLOAT),
            "vbus": self.parameter_read(base.PARAM_ADC_VBUS, base.PARAM_FLOAT),
            "iq": self.parameter_read(base.PARAM_RUN_IQ, base.PARAM_FLOAT),
        }
        return self.last_status

    def run_motor(self, wm_target):
        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8,
                             MODE_SENSORLESS_SPEED)
        self.speed_set(wm_target)
        self.parameter_action(base.ACTION_MOTOR_ENABLE)
        self.parameter_action(base.ACTION_MOTOR_RUN)
        start = time.monotonic()
        print(f"Target {wm_target * 60 / (2 * math.pi):.2f} RPM; firmware controls acceleration.")
        while not self.stop_request.requested:
            values = self.snapshot()
            if values["error"] or values["trip"]:
                raise RuntimeError(f"firmware fault: error=0x{values['error']:x}, trip=0x{values['trip']:x}")
            if values["state"] != MOTOR_RUN:
                raise RuntimeError(f"firmware left RUN: state={values['state']}")
            print(f"{time.monotonic() - start:.1f}s "
                  f"ref={values['wm_ref'] * 60 / (2 * math.pi):.2f} RPM "
                  f"fbk={values['wm'] * 60 / (2 * math.pi):.2f} RPM "
                  f"Vbus={values['vbus']:.2f} V Iq={values['iq']:.3f} A")
            if self.args.duration and time.monotonic() - start >= self.args.duration:
                break
            time.sleep(self.args.status_interval)

    def stop_all(self, graceful):
        try:
            if graceful and not self.stop_request.force:
                self.parameter_action(base.ACTION_MOTOR_STOP)
                deadline = time.monotonic() + self.args.stop_timeout
                while self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8) == MOTOR_RUN:
                    if self.stop_request.force:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("firmware STOP did not finish")
                    error = self.parameter_read(base.PARAM_EVENT_ERROR, base.PARAM_U32)
                    trip = self.parameter_read(base.PARAM_EVENT_TRIP, base.PARAM_U32)
                    if error or trip:
                        raise RuntimeError(f"firmware fault during STOP: error=0x{error:x}, trip=0x{trip:x}")
                    time.sleep(0.05)
        finally:
            self.parameter_action(base.ACTION_MOTOR_DISABLE)
            state = self.parameter_read(base.PARAM_MOTOR_STATE, base.PARAM_U8)
            if state != 0:
                raise RuntimeError(f"DISABLE did not finish: state={state}")
            print("DISABLED")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true", help="execute motor operation")
    parser.add_argument("--rpm", type=float, required=True, help="mechanical target RPM")
    parser.add_argument("--direction", choices=("forward", "reverse"), default="forward")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds from RUN; 0 runs until interrupted")
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--stop-timeout", type=float, default=30.0,
                        help="seconds to wait for firmware STOP before disabling")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()
    if not args.run:
        parser.error("--run is required to execute motor operation")
    if not math.isfinite(args.rpm) or args.rpm < 0:
        parser.error("--rpm must be finite and non-negative")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    for name in ("status_interval", "stop_timeout", "timeout"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    wm_target = args.rpm * 2 * math.pi / 60
    if args.direction == "reverse":
        wm_target = -wm_target
    return args, wm_target


def main():
    args, wm_target = parse_args()
    stop = StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.01,
                                write_timeout=args.timeout, exclusive=True) as ser:
            runner = SensorlessRun(ser, args, stop)
            graceful = False
            try:
                runner.prepare()
                runner.run_motor(wm_target)
                graceful = True
            finally:
                runner.stop_all(graceful)
    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
