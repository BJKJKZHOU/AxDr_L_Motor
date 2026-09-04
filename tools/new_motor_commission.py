#!/usr/bin/env python3
"""Identify an unknown motor, apply results to RAM, then optionally run sensorless.

The workflow is fixed intentionally:

1. Apply the known pole pairs and commissioning current limit to firmware RAM.
2. Run Rs/Ls identification five times.
3. Check repeatability and ask whether the fifth result may be applied to RAM.
4. Run Flux identification twice forward and twice reverse.
5. Check Flux repeatability and direction consistency.
6. Ask whether the fourth Flux result may be applied to RAM.
7. Unless --identify-only is used, ramp to the requested sensorless test speed
   and run until Ctrl+C by default.

Declining either identification result stops the workflow. Applied values, pole
pairs and the user current limit are not saved to nonvolatile memory and are lost
after a firmware reset. --identify-only is intended for workflows that continue
immediately into another commissioning step, such as servo phase search.

Examples:
    python3 tools/new_motor_commission.py \
        --port /dev/ttyACM0 --current-limit 2.0 --run

    python3 tools/new_motor_commission.py \
        --port /dev/ttyACM0 --current-limit 2.0 --identify-only --run
"""

import argparse
import json
import math
from pathlib import Path
import signal
import statistics
import struct
import sys
import time

import sensorless_run
import sensorless_test as base


MSG_IDENTIFICATION = 0x05

IDENT_RS_LS_START = 0x01
IDENT_FLUX_START = 0x02
IDENT_STATUS = 0x03
IDENT_ABORT = 0x04
IDENT_APPLY = 0x05

IDENT_RS_LS = 0x01
IDENT_FLUX = 0x02

IDENT_DONE = 2
IDENT_FAILED = 3

MODE_IDENT = 4

HOST_CURRENT_GUARD_RATIO = 1.10

FAST_CONFIG_ID = 13
NORMAL_CONFIG_ID = 14
FAST_VARS = (
    ("Ia", base.PARAM_ADC_IA),
    ("Ib", base.PARAM_ADC_IB),
    ("Ic", base.PARAM_ADC_IC),
)
VBUS_ID = base.PARAM_ADC_VBUS


class UserDeclined(Exception):
    pass


class IdentificationFailed(RuntimeError):
    def __init__(self, message, result):
        super().__init__(message)
        self.result = result


class Commission(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.plot_started = False
        self.ident_active = False
        self.run_peak = 0.0
        self.run_trip = False
        self.vbus = []
        self.normal_last = None
        self.normal_lost = 0
        self.normal_frames = 0

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        if len(payload) != 4 + sample_count * count * 2:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (seq - expected) & 0xFFFF
        self.fast_last = seq
        self.fast_frames += 1
        self.fast_samples += sample_count

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        if not self.ident_active:
            return

        for sample in range(sample_count):
            start = sample * count
            peak = max(abs(raw[start + index] * 0.001)
                       for index in range(count))
            self.run_peak = max(self.run_peak, peak)
            if peak > self.args.ident_current_limit:
                self.run_trip = True

    def process_normal(self, payload):
        if (len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or
                payload[3] != 1):
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        self.normal_frames += 1
        value, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(value)

    def prepare(self):
        for msg_type, op, data in (
                (base.MSG_CONTROL, base.CTRL_STOP, b""),
                (base.MSG_CONTROL, base.CTRL_DISABLE, b""),
                (base.MSG_PLOT, base.PLOT_STOP,
                 bytes([base.FAST_MASK | base.NORMAL_MASK]))):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError):
                pass

    def current_limit_set(self):
        self.parameter_write(
            base.PARAM_LIMIT_I_MAX,
            base.PARAM_FLOAT,
            self.args.current_limit,
        )
        print(
            f"Commission current limit={self.args.current_limit:.3f} A; "
            f"host guard={self.args.ident_current_limit:.3f} A"
        )

    def pole_pairs_set(self):
        self.parameter_write(
            base.PARAM_MOTOR_PP,
            base.PARAM_U8,
            self.args.pole_pairs,
        )

        value = self.parameter_read(base.PARAM_MOTOR_PP, base.PARAM_U8)
        if value != self.args.pole_pairs:
            raise RuntimeError(
                f"pole-pairs readback {value} != {self.args.pole_pairs}"
            )

        print(f"Motor pole pairs={value} (RAM)")
        return value

    def configure_plot(self):
        self.fast_last = None
        self.normal_last = None
        self.fast_lost = 0
        self.normal_lost = 0

        fast_data = bytes([
            base.FAST_GROUP,
            FAST_CONFIG_ID,
            len(FAST_VARS),
        ])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id in FAST_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(
            base.MSG_PLOT,
            base.PLOT_START,
            bytes([base.FAST_MASK | base.NORMAL_MASK]),
        )
        self.plot_started = True

    def stop_plot(self):
        if not self.plot_started:
            return
        self.request(
            base.MSG_PLOT,
            base.PLOT_STOP,
            bytes([base.FAST_MASK | base.NORMAL_MASK]),
        )
        self.plot_started = False

    def check_vbus(self):
        self.vbus.clear()
        deadline = time.monotonic() + self.args.vbus_seconds
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))

        if not self.vbus:
            raise RuntimeError("no Vbus samples")
        mean = statistics.mean(self.vbus)
        print(
            f"Vbus={mean:.3f} V "
            f"({min(self.vbus):.3f} .. {max(self.vbus):.3f} V)"
        )
        if not self.args.vbus_min <= mean <= self.args.vbus_max:
            raise RuntimeError(
                f"Vbus {mean:.3f} V outside "
                f"{self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
            )

    def ident_status(self, mode):
        data = self.request(MSG_IDENTIFICATION, IDENT_STATUS)
        expected = 11 if mode == IDENT_RS_LS else 7
        if len(data) != expected:
            raise RuntimeError(
                f"invalid identification status length: {len(data)}"
            )

        rx_mode, state, valid = data[:3]
        if rx_mode != mode:
            raise RuntimeError(f"unexpected identification mode: {rx_mode}")

        result = {
            "state": state,
            "valid": bool(valid),
        }
        if mode == IDENT_RS_LS:
            result["rs_ohm"], result["ls_h"] = struct.unpack_from(
                "<ff", data, 3
            )
        else:
            result["flux_wb"], = struct.unpack_from("<f", data, 3)
        return result

    def run_ident(self, mode, run_number, direction=None):
        if direction is not None:
            sign = 1.0 if direction == "forward" else -1.0
            wm_test = sign * base.IF_WE_RAD_S / self.args.pole_pairs
            self.speed_set(wm_test)

        start_op = (IDENT_RS_LS_START if mode == IDENT_RS_LS
                    else IDENT_FLUX_START)

        self.parameter_write(base.PARAM_MOTOR_MODE, base.PARAM_U8, MODE_IDENT)
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)

        self.run_peak = 0.0
        self.run_trip = False
        fast_lost_start = self.fast_lost
        normal_lost_start = self.normal_lost
        fast_samples_start = self.fast_samples
        normal_frames_start = self.normal_frames
        start = time.monotonic()
        self.ident_active = True
        result = None
        failure = None

        try:
            self.request(MSG_IDENTIFICATION, start_op)
            deadline = start + self.args.ident_timeout
            next_status = start

            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_status:
                    result = self.ident_status(mode)
                    next_status = now + self.args.poll_interval
                else:
                    self.process(self.parser.feed(self.ser.read(4096)))

                if self.run_trip:
                    failure = RuntimeError(
                        "identification phase current exceeded host guard "
                        f"{self.args.ident_current_limit:.3f} A"
                    )
                    break
                if result is not None and result["state"] == IDENT_DONE:
                    break
                if result is not None and result["state"] == IDENT_FAILED:
                    name = "Rs/Ls" if mode == IDENT_RS_LS else "Flux"
                    failure = RuntimeError(
                        f"{name} identification failed; "
                        f"captured phase peak={self.run_peak:.3f} A"
                    )
                    break
            else:
                failure = TimeoutError("identification timeout")
        except (TimeoutError, RuntimeError) as exc:
            failure = exc
        finally:
            self.ident_active = False
            try:
                self.request(base.MSG_CONTROL, base.CTRL_DISABLE)
            except (TimeoutError, RuntimeError) as exc:
                print(f"DISABLE warning: {exc}", file=sys.stderr)

        if result is None:
            result = {
                "state": -1,
                "valid": False,
            }
        result["run"] = run_number
        result["direction"] = direction
        result["time_s"] = time.monotonic() - start
        result["phase_peak_a"] = self.run_peak
        result["fast_lost"] = self.fast_lost - fast_lost_start
        result["normal_lost"] = self.normal_lost - normal_lost_start
        result["fast_samples"] = self.fast_samples - fast_samples_start
        result["normal_frames"] = self.normal_frames - normal_frames_start
        if failure is not None:
            result["error"] = str(failure)
            raise IdentificationFailed(str(failure), result)
        return result

    def apply(self):
        self.request(MSG_IDENTIFICATION, IDENT_APPLY)

    def stop_all(self):
        self.ident_active = False
        for name, msg_type, op, data in (
                ("STOP", base.MSG_CONTROL, base.CTRL_STOP, b""),
                ("DISABLE", base.MSG_CONTROL, base.CTRL_DISABLE, b""),
                ("PLOT_STOP", base.MSG_PLOT, base.PLOT_STOP,
                 bytes([base.FAST_MASK | base.NORMAL_MASK]))):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError) as exc:
                print(f"{name} warning: {exc}", file=sys.stderr)
        self.plot_started = False


def max_relative_deviation(values):
    median = statistics.median(values)
    if median <= 0.0:
        return math.inf
    return max(abs(value - median) / median for value in values)


def evaluate_rs_ls(results, args):
    rs = [item["rs_ohm"] for item in results]
    ls = [item["ls_h"] for item in results]
    summary = {
        "rs_median_ohm": statistics.median(rs),
        "ls_median_h": statistics.median(ls),
        "rs_max_relative_deviation": max_relative_deviation(rs),
        "ls_max_relative_deviation": max_relative_deviation(ls),
    }
    reasons = []
    if not all(item["valid"] for item in results):
        reasons.append("one or more results are invalid")
    if not all(math.isfinite(value) and value > 0.0 for value in rs + ls):
        reasons.append("Rs/Ls contains a non-positive or non-finite value")
    if summary["rs_max_relative_deviation"] > args.rl_repeat_limit:
        reasons.append("Rs repeatability exceeds limit")
    if summary["ls_max_relative_deviation"] > args.rl_repeat_limit:
        reasons.append("Ls repeatability exceeds limit")
    if any(item["fast_lost"] > args.max_lost or
           item["normal_lost"] > args.max_lost for item in results):
        reasons.append("telemetry loss exceeds limit")
    if any(item["fast_samples"] == 0 or item["normal_frames"] == 0
           for item in results):
        reasons.append("one or more runs have no FAST/NORMAL telemetry")
    return summary, reasons


def evaluate_flux(results, args):
    flux = [item["flux_wb"] for item in results]
    forward = [item["flux_wb"] for item in results
               if item["direction"] == "forward"]
    reverse = [item["flux_wb"] for item in results
               if item["direction"] == "reverse"]
    median = statistics.median(flux)
    direction_error = (abs(statistics.mean(forward) - statistics.mean(reverse)) /
                       median if median > 0.0 else math.inf)
    summary = {
        "flux_median_wb": median,
        "flux_max_relative_deviation": max_relative_deviation(flux),
        "direction_relative_difference": direction_error,
    }
    reasons = []
    if not all(item["valid"] for item in results):
        reasons.append("one or more results are invalid")
    if not all(math.isfinite(value) and value > 0.0 for value in flux):
        reasons.append("Flux contains a non-positive or non-finite value")
    if summary["flux_max_relative_deviation"] > args.flux_repeat_limit:
        reasons.append("Flux repeatability exceeds limit")
    if direction_error > args.flux_repeat_limit:
        reasons.append("forward/reverse Flux difference exceeds limit")
    if any(item["fast_lost"] > args.max_lost or
           item["normal_lost"] > args.max_lost for item in results):
        reasons.append("telemetry loss exceeds limit")
    if any(item["fast_samples"] == 0 or item["normal_frames"] == 0
           for item in results):
        reasons.append("one or more runs have no FAST/NORMAL telemetry")
    return summary, reasons


def print_rs_ls(results, summary):
    print("\nRs/Ls results")
    print(" run       Rs (ohm)      Ls (uH)   peak (A)  lost F/N")
    for item in results:
        print(
            f" {item['run']:>3d}  {item['rs_ohm']:>13.7g}  "
            f"{item['ls_h'] * 1.0e6:>11.4f}  "
            f"{item['phase_peak_a']:>8.3f}  "
            f"{item['fast_lost']}/{item['normal_lost']}"
        )
    print(
        f" median: Rs={summary['rs_median_ohm']:.7g} ohm, "
        f"Ls={summary['ls_median_h'] * 1.0e6:.4f} uH; "
        f"max deviation={100.0 * summary['rs_max_relative_deviation']:.2f}%/"
        f"{100.0 * summary['ls_max_relative_deviation']:.2f}%"
    )


def print_flux(results, summary):
    print("\nFlux results")
    print(" run  direction     Flux (Wb)  peak (A)  lost F/N")
    for item in results:
        print(
            f" {item['run']:>3d}  {item['direction']:<9}  "
            f"{item['flux_wb']:>12.7g}  "
            f"{item['phase_peak_a']:>8.3f}  "
            f"{item['fast_lost']}/{item['normal_lost']}"
        )
    print(
        f" median: Flux={summary['flux_median_wb']:.7g} Wb; "
        f"run deviation={100.0 * summary['flux_max_relative_deviation']:.2f}%, "
        f"forward/reverse={100.0 * summary['direction_relative_difference']:.2f}%"
    )


def confirm_ram(prompt):
    while True:
        try:
            answer = input(f"{prompt} [y/N]: ").strip().lower()
        except EOFError:
            return False
        if answer in ("y", "yes"):
            return True
        if answer in ("", "n", "no"):
            return False
        print("Please enter y or n.")


def log_write(record, output):
    path = (Path(output).expanduser().resolve() if output else
            Path(__file__).resolve().parents[1] / "build" / "Release" /
            f"new_motor_commission_{time.strftime('%Y%m%d_%H%M%S')}.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"Log: {path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Identify an unknown motor and optionally run guarded sensorless speed"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument(
        "--identify-only",
        action="store_true",
        help="stop after Rs/Ls and Flux are accepted and applied to firmware RAM",
    )
    parser.add_argument("--target-we", type=int, default=1000,
                        help="final sensorless test point in electrical rad/s")
    parser.add_argument("--pole-pairs", type=int, default=16,
                        help="known pole pairs, written to firmware RAM and used for speed conversion")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="final hold seconds; 0 runs until Ctrl+C")
    parser.add_argument(
        "--current-limit", type=float, required=True,
        help="maximum phase current allowed for this motor during commissioning",
    )
    parser.add_argument("--rl-repeat-limit", type=float, default=0.10)
    parser.add_argument("--flux-repeat-limit", type=float, default=0.05)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ident-timeout", type=float, default=25.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--ramp-step", type=float, default=0.5,
                        help="mechanical rad/s per final-run ramp step")
    parser.add_argument("--ramp-interval", type=float, default=0.02)
    parser.add_argument("--pll-rms-limit", type=float, default=0.08)
    parser.add_argument("--pll-window", type=float, default=0.2)
    parser.add_argument("--we-tolerance", type=float, default=20.0)
    parser.add_argument("--we-relative-tolerance", type=float, default=0.03)
    parser.add_argument("--speed-error-time", type=float, default=1.0)
    parser.add_argument("--voltage-util-limit", type=float, default=0.985)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--target-timeout", type=float, default=10.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--max-lost", type=int, default=0)
    parser.add_argument("--output", help="JSON log path")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if not 120 <= args.target_we <= 3000:
        parser.error("--target-we must be between 120 and 3000 electrical rad/s")
    if not 1 <= args.pole_pairs <= 64:
        parser.error("--pole-pairs must be between 1 and 64")
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    for name in (
            "current_limit", "rl_repeat_limit", "flux_repeat_limit",
            "vbus_min", "vbus_max", "vbus_seconds", "ident_timeout",
            "poll_interval", "ramp_step", "ramp_interval", "pll_rms_limit",
            "pll_window", "we_tolerance", "we_relative_tolerance",
            "speed_error_time", "voltage_util_limit", "ready_timeout",
            "target_timeout", "settle_seconds", "status_interval", "timeout"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("vbus-min must be less than vbus-max")
    if args.voltage_util_limit > 1.0:
        parser.error("voltage-util-limit must not exceed 1")
    if args.max_lost < 0:
        parser.error("max-lost must be non-negative")

    args.ident_current_limit = HOST_CURRENT_GUARD_RATIO * args.current_limit
    sign = 1.0 if args.direction == "forward" else -1.0
    args.target_wm = sign * args.target_we / args.pole_pairs
    args.rpm = abs(args.target_wm) * 60.0 / (2.0 * math.pi)
    return args


def main():
    args = parse_args()
    record = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "current_limit_a": args.current_limit,
        "host_current_guard_a": args.ident_current_limit,
        "target_we_rad_s": args.target_we,
        "target_wm_rad_s": args.target_wm,
        "pole_pairs": args.pole_pairs,
        "direction": args.direction,
        "identify_only": args.identify_only,
        "status": "started",
        "rs_ls": [],
        "flux": [],
    }
    ident = None
    runner = None
    graceful = False
    error = None
    declined = None

    print(
        "Unknown-motor commissioning: 5 x Rs/Ls, "
        "2 x forward Flux, 2 x reverse Flux"
    )
    if args.identify_only:
        print("Final sensorless run: skipped (--identify-only)")
    else:
        print(
            f"Final target: {args.target_wm:.3f} mechanical rad/s "
            f"({args.target_we} electrical rad/s, {args.rpm:.2f} RPM at Pp={args.pole_pairs})"
        )
    print("RAM updates are temporary; reset restores compiled parameters.")

    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            ident = Commission(ser, args)
            try:
                ident.prepare()
                ident.configure_plot()
                ident.check_vbus()
                record["firmware_pole_pairs"] = ident.pole_pairs_set()
                ident.current_limit_set()

                for run in range(1, 6):
                    print(f"\nRs/Ls identification {run}/5")
                    try:
                        result = ident.run_ident(IDENT_RS_LS, run)
                    except IdentificationFailed as exc:
                        record["rs_ls"].append(exc.result)
                        raise
                    record["rs_ls"].append(result)

                rl_summary, reasons = evaluate_rs_ls(record["rs_ls"], args)
                record["rs_ls_summary"] = rl_summary
                print_rs_ls(record["rs_ls"], rl_summary)
                ident.stop_plot()
                if reasons:
                    raise RuntimeError("Rs/Ls quality gate: " + "; ".join(reasons))
                if not confirm_ram(
                        "Apply run 5 Rs/Ls to RAM (Ld=Lq=Ls)?"):
                    raise UserDeclined("Rs/Ls RAM update declined")
                ident.apply()
                record["rs_ls_applied"] = True
                print("Rs/Ls applied to RAM from run 5.")

                ident.configure_plot()
                directions = ("forward", "forward", "reverse", "reverse")
                for run, direction in enumerate(directions, 1):
                    print(f"\nFlux identification {run}/4 ({direction})")
                    try:
                        result = ident.run_ident(
                            IDENT_FLUX,
                            run,
                            direction=direction,
                        )
                    except IdentificationFailed as exc:
                        record["flux"].append(exc.result)
                        raise
                    record["flux"].append(result)

                flux_summary, reasons = evaluate_flux(record["flux"], args)
                record["flux_summary"] = flux_summary
                print_flux(record["flux"], flux_summary)
                ident.stop_plot()
                if reasons:
                    raise RuntimeError("Flux quality gate: " + "; ".join(reasons))
                if not confirm_ram("Apply run 4 Flux to RAM?"):
                    raise UserDeclined("Flux RAM update declined")
                ident.apply()
                record["flux_applied"] = True
                print("Flux applied to RAM from run 4 (second reverse run).")

                ident.stop_all()
                ident = None

                if args.identify_only:
                    record["status"] = "completed"
                    print(
                        "Identification complete. Rs/Ls and Flux remain in firmware RAM; "
                        "do not reset the MCU before the next commissioning step."
                    )
                    return

                ser.reset_input_buffer()
                time.sleep(0.05)

                stop = sensorless_run.StopRequest()
                signal.signal(signal.SIGINT, stop.handle)
                signal.signal(signal.SIGTERM, stop.handle)
                runner = sensorless_run.SensorlessRun(ser, args, stop)
                runner.prepare()
                runner.configure_plot()
                runner.check_vbus()
                runner.run_motor(args.target_wm)
                graceful = True
                record["status"] = "completed"
            finally:
                if runner is not None:
                    runner.stop_all(graceful)
                    record["sensorless"] = {
                        "current_peak_a": runner.current_peak,
                        "fast_lost": runner.fast_lost,
                        "normal_lost": runner.normal_lost,
                    }
                elif ident is not None:
                    ident.stop_all()

    except UserDeclined as exc:
        declined = str(exc)
        record["status"] = "declined"
        record["message"] = declined
    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, KeyboardInterrupt) as exc:
        error = exc
        record["status"] = "failed"
        record["error"] = str(exc)
    finally:
        record["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        try:
            log_write(record, args.output)
        except OSError as exc:
            print(f"Log warning: {exc}", file=sys.stderr)

    if declined is not None:
        print(f"Stopped safely: {declined}")
    if error is not None:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
