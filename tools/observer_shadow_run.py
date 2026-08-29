#!/usr/bin/env python3
"""Characterize the sensorless observer at fixed I/F speed without takeover.

Known motor parameters are written to firmware RAM before the run. Sensorless
shadow mode keeps FOC on the I/F angle/current path while Flux Observer + PLL
run normally in parallel. No Rs/Ls or Flux identification is performed and the
observer is never allowed to take control.
"""

import argparse
import math
import signal
import statistics
import struct
import sys
import time

import sensorless_run
import sensorless_test as base


CTRL_I_LIMIT_SET = 0x07
CTRL_MOTOR_PARA_SET = 0x0A
CTRL_MOTOR_PARA_GET = 0x0B
SENSORLESS_SHADOW_SET = 0x04
SENSORLESS_SHADOW_GET = 0x05
STAGE_IF_TO_OBS = 2

REJECT_WE = 1 << 0
REJECT_PLL = 1 << 1
REJECT_FLUX = 1 << 2


def mean(values):
    return statistics.fmean(values) if values else math.nan


def stdev(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def rms(values):
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else math.nan


class ShadowRun(sensorless_run.SensorlessRun):
    def motor_para_set(self):
        payload = bytes([self.args.pole_pairs]) + struct.pack(
            "<ffff",
            self.args.rs,
            self.args.ld,
            self.args.lq,
            self.args.flux,
        )
        self.request(base.MSG_CONTROL, CTRL_MOTOR_PARA_SET, payload)

        data = self.request(base.MSG_CONTROL, CTRL_MOTOR_PARA_GET)
        if len(data) != 17:
            raise RuntimeError(f"invalid motor parameter readback length: {len(data)}")

        pp = data[0]
        rs, ld, lq, flux = struct.unpack_from("<ffff", data, 1)
        expected = (
            ("Pp", float(pp), float(self.args.pole_pairs), 0.0),
            ("Rs", rs, self.args.rs, 1e-5),
            ("Ld", ld, self.args.ld, 1e-8),
            ("Lq", lq, self.args.lq, 1e-8),
            ("Flux", flux, self.args.flux, 1e-7),
        )
        for name, actual, target, tol in expected:
            if abs(actual - target) > tol:
                raise RuntimeError(
                    f"motor parameter readback mismatch: {name}={actual} expected {target}"
                )

        print(
            "Known motor parameters loaded to RAM: "
            f"Pp={pp}, Rs={rs:.6f} ohm, "
            f"Ld={ld * 1e6:.3f} uH, Lq={lq * 1e6:.3f} uH, "
            f"Flux={flux * 1e3:.5f} mWb"
        )

    def current_limit_set(self):
        self.request(
            base.MSG_CONTROL,
            CTRL_I_LIMIT_SET,
            struct.pack("<f", self.args.current_limit),
        )
        print(f"Current limit={self.args.current_limit:.3f} A (RAM)")

    def shadow_set(self, enable):
        self.request(
            base.MSG_SENSORLESS,
            SENSORLESS_SHADOW_SET,
            bytes([1 if enable else 0]),
        )
        data = self.request(base.MSG_SENSORLESS, SENSORLESS_SHADOW_GET)
        if len(data) != 1 or bool(data[0]) != bool(enable):
            raise RuntimeError("sensorless shadow mode readback mismatch")
        print(f"Sensorless shadow mode={'ON' if enable else 'OFF'}")

    def wait_shadow(self):
        deadline = time.monotonic() + self.args.ready_timeout
        last_stage = None

        while time.monotonic() < deadline:
            stage = self.read_stage()
            if stage != last_stage:
                print(f"  Sensorless stage={sensorless_run.STAGE_NAME.get(stage, stage)}")
                last_stage = stage
            if stage == STAGE_IF_TO_OBS:
                return
            self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(require_run=False)
            time.sleep(0.01)

        raise TimeoutError("Sensorless shadow did not reach IF_TO_OBS")

    def collect_shadow(self):
        start = time.monotonic()
        deadline = start + self.args.duration
        next_poll = start
        rows = []

        while time.monotonic() < deadline:
            now = time.monotonic()
            self.process(self.parser.feed(self.ser.read(4096)))
            self.guard(require_run=False)

            if now >= next_poll:
                stage = self.read_stage()
                if stage != STAGE_IF_TO_OBS:
                    raise RuntimeError(f"left shadow IF_TO_OBS: stage={stage}")
                if self.handover_diag is not None:
                    rows.append(dict(self.handover_diag))
                next_poll = now + self.args.diag_interval

        if not rows:
            raise RuntimeError("no observer shadow diagnostics collected")
        return rows


def print_report(rows):
    we_if = [row["We_IF"] for row in rows]
    we_obs = [row["We_Obs_F"] for row in rows]
    we_err = [row["We_Err"] for row in rows]
    pll = [row["PLL_Err"] for row in rows]
    flux = [row["Flux_Ratio"] for row in rows]
    theta = [row["Theta_Err"] for row in rows]

    reject_we = sum(bool(row["Reject"] & REJECT_WE) for row in rows)
    reject_pll = sum(bool(row["Reject"] & REJECT_PLL) for row in rows)
    reject_flux = sum(bool(row["Reject"] & REJECT_FLUX) for row in rows)
    stable = sum(row["Reject"] == 0 for row in rows)
    count = len(rows)
    stable_max = max(row["Stable_Max_s"] for row in rows)
    reject_seen = 0
    for row in rows:
        reject_seen |= row["Reject_Seen"]

    names = []
    if reject_seen & REJECT_WE:
        names.append("WE")
    if reject_seen & REJECT_PLL:
        names.append("PLL")
    if reject_seen & REJECT_FLUX:
        names.append("FLUX")

    print("\nObserver shadow report")
    print(f"  samples={count}, We_IF mean={mean(we_if):+.3f} rad/s")
    print(
        f"  We_obs_f mean={mean(we_obs):+.3f} rad/s "
        f"std={stdev(we_obs):.3f} min={min(we_obs):+.3f} max={max(we_obs):+.3f}"
    )
    print(
        f"  We_err   mean={mean(we_err):+.3f} rad/s "
        f"std={stdev(we_err):.3f} min={min(we_err):+.3f} max={max(we_err):+.3f}"
    )
    print(
        f"  PLL_err  mean={mean(pll):+.5f} rad RMS={rms(pll):.5f} "
        f"peak={max(abs(value) for value in pll):.5f}"
    )
    print(
        f"  Flux ratio mean={mean(flux):.4f} std={stdev(flux):.4f} "
        f"min={min(flux):.4f} max={max(flux):.4f}"
    )
    print(
        f"  Theta_err mean={mean(theta):+.5f} rad std={stdev(theta):.5f}"
    )
    print(
        f"  criteria poll pass={100.0 * stable / count:.1f}% "
        f"reject WE={100.0 * reject_we / count:.1f}% "
        f"PLL={100.0 * reject_pll / count:.1f}% "
        f"FLUX={100.0 * reject_flux / count:.1f}%"
    )
    print(
        f"  longest continuous stable={1000.0 * stable_max:.1f} ms; "
        f"reject seen={'|'.join(names) if names else 'NONE'}"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run fixed-speed I/F while characterizing the observer in shadow mode"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")

    # Latest repeatable 4010 commissioning medians; override for another motor.
    parser.add_argument("--rs", type=float, default=6.334559)
    parser.add_argument("--ld", type=float, default=0.001564084)
    parser.add_argument("--lq", type=float, default=0.001564084)
    parser.add_argument("--flux", type=float, default=0.01513128)
    parser.add_argument("--pole-pairs", type=int, default=11)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--we", type=float, default=120.0,
                        help="fixed electrical I/F speed in rad/s")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--diag-interval", type=float, default=0.02,
                        help="status diagnostic poll interval in seconds")

    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)

    # SensorlessRun guard/config fields. Closed-loop-only limits are retained but
    # are not enforced while require_run=False in shadow mode.
    parser.add_argument("--pll-rms-limit", type=float, default=0.08)
    parser.add_argument("--pll-window", type=float, default=0.2)
    parser.add_argument("--we-tolerance", type=float, default=20.0)
    parser.add_argument("--we-relative-tolerance", type=float, default=0.03)
    parser.add_argument("--speed-error-time", type=float, default=1.0)
    parser.add_argument("--voltage-util-limit", type=float, default=0.985)
    parser.add_argument("--ramp-step", type=float, default=0.5)
    parser.add_argument("--ramp-interval", type=float, default=0.02)
    parser.add_argument("--target-timeout", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--rpm", type=float, default=1.0,
                        help=argparse.SUPPRESS)
    parser.add_argument("--direction", default="forward",
                        help=argparse.SUPPRESS)

    args = parser.parse_args()
    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.pole_pairs <= 0:
        parser.error("--pole-pairs must be positive")
    for name in ("rs", "ld", "lq", "flux", "current_limit", "we",
                 "duration", "diag_interval", "ready_timeout", "timeout"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("vbus-min must be less than vbus-max")
    if args.max_lost < 0:
        parser.error("max-lost must be non-negative")
    return args


def main():
    args = parse_args()
    stop = sensorless_run.StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    wm = args.we / args.pole_pairs
    print(
        f"Observer shadow target: We={args.we:.3f} rad/s, "
        f"Wm={wm:.3f} rad/s ({wm * 60.0 / (2.0 * math.pi):.2f} RPM)"
    )

    test = None
    error = None
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = ShadowRun(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.motor_para_set()
                test.current_limit_set()
                test.shadow_set(True)
                test.configure_plot()
                test.check_vbus()

                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET,
                             bytes([base.MODE_SENSORLESS_SPEED]))
                test.speed_set(wm)
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print("ALIGN -> I/F -> observer shadow (no takeover)")

                test.wait_shadow()
                print(f"Holding shadow observation for {args.duration:.2f} s...")
                rows = test.collect_shadow()
                print_report(rows)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop_all(False)
                try:
                    test.shadow_set(False)
                except (TimeoutError, RuntimeError):
                    pass

            print(
                f"Peak current magnitude={test.current_peak:.3f} A, "
                f"FAST lost={test.fast_lost}, NORMAL lost={test.normal_lost}"
            )

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, KeyboardInterrupt) as exc:
        error = exc

    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
