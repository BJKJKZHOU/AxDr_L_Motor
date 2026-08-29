#!/usr/bin/env python3
"""Sweep sensorless PLL bandwidth at fixed I/F speed without takeover.

Known motor parameters are written to firmware RAM before the run. For each PLL
bandwidth point, sensorless shadow mode independently repeats ALIGN -> I/F and
keeps FOC on the I/F path while Flux Observer + PLL run in parallel. No motor
identification is performed and the observer is never allowed to take control.
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
SENSORLESS_PLL_BW_SET = 0x06
SENSORLESS_PLL_BW_GET = 0x07
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
            "<ffff", self.args.rs, self.args.ld, self.args.lq, self.args.flux
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

    def shadow_set(self, enable, quiet=False):
        self.request(
            base.MSG_SENSORLESS,
            SENSORLESS_SHADOW_SET,
            bytes([1 if enable else 0]),
        )
        data = self.request(base.MSG_SENSORLESS, SENSORLESS_SHADOW_GET)
        if len(data) != 1 or bool(data[0]) != bool(enable):
            raise RuntimeError("sensorless shadow mode readback mismatch")
        if not quiet:
            print(f"Sensorless shadow mode={'ON' if enable else 'OFF'}")

    def pll_bw_set(self, bw_hz):
        self.request(
            base.MSG_SENSORLESS,
            SENSORLESS_PLL_BW_SET,
            struct.pack("<f", bw_hz),
        )
        data = self.request(base.MSG_SENSORLESS, SENSORLESS_PLL_BW_GET)
        if len(data) != 4:
            raise RuntimeError(f"invalid PLL bandwidth readback length: {len(data)}")
        actual, = struct.unpack("<f", data)
        if abs(actual - bw_hz) > 1e-4:
            raise RuntimeError(
                f"PLL bandwidth readback mismatch: {actual:.4f} != {bw_hz:.4f} Hz"
            )
        return actual

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


def analyze_rows(rows):
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
    reject_seen = 0
    for row in rows:
        reject_seen |= row["Reject_Seen"]

    return {
        "samples": count,
        "we_if_mean": mean(we_if),
        "we_obs_mean": mean(we_obs),
        "we_obs_std": stdev(we_obs),
        "we_obs_min": min(we_obs),
        "we_obs_max": max(we_obs),
        "we_err_mean": mean(we_err),
        "we_err_std": stdev(we_err),
        "we_err_min": min(we_err),
        "we_err_max": max(we_err),
        "pll_mean": mean(pll),
        "pll_rms": rms(pll),
        "pll_peak": max(abs(value) for value in pll),
        "flux_mean": mean(flux),
        "flux_std": stdev(flux),
        "flux_min": min(flux),
        "flux_max": max(flux),
        "theta_mean": mean(theta),
        "theta_std": stdev(theta),
        "stable_pct": 100.0 * stable / count,
        "reject_we_pct": 100.0 * reject_we / count,
        "reject_pll_pct": 100.0 * reject_pll / count,
        "reject_flux_pct": 100.0 * reject_flux / count,
        "stable_max_s": max(row["Stable_Max_s"] for row in rows),
        "reject_seen": reject_seen,
    }


def reject_names(mask):
    names = []
    if mask & REJECT_WE:
        names.append("WE")
    if mask & REJECT_PLL:
        names.append("PLL")
    if mask & REJECT_FLUX:
        names.append("FLUX")
    return "|".join(names) if names else "NONE"


def print_report(bw_hz, result):
    print(f"\nObserver shadow report: PLL BW={bw_hz:.1f} Hz")
    print(f"  samples={result['samples']}, We_IF mean={result['we_if_mean']:+.3f} rad/s")
    print(
        f"  We_obs_f mean={result['we_obs_mean']:+.3f} rad/s "
        f"std={result['we_obs_std']:.3f} min={result['we_obs_min']:+.3f} "
        f"max={result['we_obs_max']:+.3f}"
    )
    print(
        f"  We_err   mean={result['we_err_mean']:+.3f} rad/s "
        f"std={result['we_err_std']:.3f} min={result['we_err_min']:+.3f} "
        f"max={result['we_err_max']:+.3f}"
    )
    print(
        f"  PLL_err  mean={result['pll_mean']:+.5f} rad "
        f"RMS={result['pll_rms']:.5f} peak={result['pll_peak']:.5f}"
    )
    print(
        f"  Flux ratio mean={result['flux_mean']:.4f} std={result['flux_std']:.4f} "
        f"min={result['flux_min']:.4f} max={result['flux_max']:.4f}"
    )
    print(
        f"  Theta_err mean={result['theta_mean']:+.5f} rad "
        f"std={result['theta_std']:.5f}"
    )
    print(
        f"  criteria poll pass={result['stable_pct']:.1f}% "
        f"reject WE={result['reject_we_pct']:.1f}% "
        f"PLL={result['reject_pll_pct']:.1f}% "
        f"FLUX={result['reject_flux_pct']:.1f}%"
    )
    print(
        f"  longest continuous stable={1000.0 * result['stable_max_s']:.1f} ms; "
        f"reject seen={reject_names(result['reject_seen'])}"
    )


def print_summary(results):
    print("\nPLL bandwidth sweep summary")
    print("  BW(Hz)  We mean   We std  Err mean  PLL RMS  Flux mean  Pass%  Stable max")
    for bw_hz, result in results:
        print(
            f"  {bw_hz:6.1f}  {result['we_obs_mean']:7.2f}  "
            f"{result['we_obs_std']:7.2f}  {result['we_err_mean']:+8.2f}  "
            f"{result['pll_rms']:7.4f}  {result['flux_mean']:9.4f}  "
            f"{result['stable_pct']:5.1f}%  {1000.0 * result['stable_max_s']:8.1f} ms"
        )


def parse_bw_list(text):
    try:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("PLL bandwidth list must be comma-separated numbers") from exc
    if not values or any((not math.isfinite(value)) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("PLL bandwidth values must be positive finite numbers")
    return values


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sweep PLL bandwidth while characterizing the observer in shadow mode"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")

    parser.add_argument("--rs", type=float, default=6.334559)
    parser.add_argument("--ld", type=float, default=0.001564084)
    parser.add_argument("--lq", type=float, default=0.001564084)
    parser.add_argument("--flux", type=float, default=0.01513128)
    parser.add_argument("--pole-pairs", type=int, default=11)
    parser.add_argument("--current-limit", type=float, default=2.0)
    parser.add_argument("--we", type=float, default=120.0,
                        help="fixed electrical I/F speed in rad/s")
    parser.add_argument("--pll-bw", type=parse_bw_list, default=[10.0, 20.0, 30.0, 50.0],
                        help="comma-separated PLL bandwidth sweep in Hz")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="shadow observation seconds per PLL bandwidth")
    parser.add_argument("--diag-interval", type=float, default=0.02)

    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)

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
    parser.add_argument("--rpm", type=float, default=1.0, help=argparse.SUPPRESS)
    parser.add_argument("--direction", default="forward", help=argparse.SUPPRESS)

    args = parser.parse_args()
    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.pole_pairs <= 0:
        parser.error("--pole-pairs must be positive")
    for name in ("rs", "ld", "lq", "flux", "current_limit", "we",
                 "duration", "diag_interval", "ready_timeout", "timeout"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if any(value < 1.0 or value > 200.0 for value in args.pll_bw):
        parser.error("--pll-bw values must be within 1 .. 200 Hz")
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
    print("PLL bandwidth sweep: " + ", ".join(f"{value:g} Hz" for value in args.pll_bw))

    test = None
    error = None
    results = []
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
                test.check_vbus()

                for index, requested_bw in enumerate(args.pll_bw, start=1):
                    if stop.requested:
                        break

                    test.prepare()
                    time.sleep(0.05)
                    actual_bw = test.pll_bw_set(requested_bw)
                    test.shadow_set(True, quiet=True)
                    test.configure_plot()

                    test.request(base.MSG_CONTROL, base.CTRL_MODE_SET,
                                 bytes([base.MODE_SENSORLESS_SPEED]))
                    test.speed_set(wm)
                    test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                    test.request(base.MSG_CONTROL, base.CTRL_RUN)

                    print(f"\nPLL sweep {index}/{len(args.pll_bw)}: {actual_bw:.1f} Hz")
                    print("ALIGN -> I/F -> observer shadow (no takeover)")
                    test.wait_shadow()
                    print(f"Holding shadow observation for {args.duration:.2f} s...")
                    rows = test.collect_shadow()
                    result = analyze_rows(rows)
                    results.append((actual_bw, result))
                    print_report(actual_bw, result)

                    test.stop_all(False)
                    test.shadow_set(False, quiet=True)
                    time.sleep(0.1)

            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                try:
                    test.stop_all(False)
                except (TimeoutError, RuntimeError):
                    pass
                try:
                    test.shadow_set(False, quiet=True)
                except (TimeoutError, RuntimeError):
                    pass

            if results:
                print_summary(results)
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
