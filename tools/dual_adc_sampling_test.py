#!/usr/bin/env python3
"""Automated validation for dual-ADC current sampling.

The default static phase keeps the power stage disabled and checks:
- FAST Ia/Ib/Ic telemetry continuity and effective 20 kHz sample rate.
- Ia/Ib zero-current mean, RMS noise, percentiles, peaks and tail counts.
- ADC2 regular Vbus telemetry continuity and range.

With --dynamic --run, a second phase runs one Rs/Ls identification cycle only
as a guarded PWM/current excitation. The result is NOT applied to Motor_Para.
The dynamic phase checks that both current channels remain active and continuous,
Vbus regular sampling keeps running, and measured current stays below the host
current guard.

Ic is reconstructed in firmware as -(Ia + Ib), so Ia+Ib+Ic is intentionally not
used as an independent pass criterion.

Examples:
    python3 tools/dual_adc_sampling_test.py \
        --port /dev/ttyACM0 --vbus-min 10 --vbus-max 20

    python3 tools/dual_adc_sampling_test.py \
        --port /dev/ttyACM0 --vbus-min 10 --vbus-max 20 \
        --dynamic --current-limit 1.0 --run
"""

import argparse
import math
import statistics
import struct
import sys
import time

import sensorless_test as base


FAST_CONFIG_ID = 21
NORMAL_CONFIG_ID = 22
FAST_VARS = (
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
)
VBUS_ID = 0x0004

MSG_IDENTIFICATION = 0x05
MODE_IDENT = 4
CTRL_I_LIMIT_SET = 0x07
IDENT_MODE_SET = 0x01
IDENT_STATUS = 0x02
IDENT_RS_LS = 0x01
IDENT_DONE = 2
IDENT_FAILED = 3

HOST_CURRENT_GUARD_RATIO = 1.10


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


class RunningStats:
    def __init__(self):
        self.count = 0
        self.sum = [0.0] * len(FAST_VARS)
        self.sum_sq = [0.0] * len(FAST_VARS)
        self.peak = [0.0] * len(FAST_VARS)
        self.abs_samples = [[] for _ in FAST_VARS]

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            self.sum[index] += value
            self.sum_sq[index] += value * value
            absolute = abs(value)
            self.peak[index] = max(self.peak[index], absolute)
            self.abs_samples[index].append(absolute)

    def mean(self, index):
        return self.sum[index] / self.count

    def rms_noise(self, index):
        mean = self.mean(index)
        variance = self.sum_sq[index] / self.count - mean * mean
        return math.sqrt(max(variance, 0.0))

    def p99(self, index):
        return percentile(self.abs_samples[index], 0.99)

    def p999(self, index):
        return percentile(self.abs_samples[index], 0.999)

    def above(self, index, threshold):
        return sum(value > threshold for value in self.abs_samples[index])


class PhaseCapture:
    def __init__(self):
        self.stats = RunningStats()
        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_lost = 0
        self.fast_last = None
        self.normal_frames = 0
        self.normal_lost = 0
        self.normal_last = None
        self.vbus = []
        self.started = time.monotonic()

    def elapsed(self):
        return max(time.monotonic() - self.started, 1.0e-9)


class DualADCTest(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.capture = PhaseCapture()

    def reset_capture(self):
        self.capture = PhaseCapture()

    def prepare(self):
        for msg_type, op, data in (
            (base.MSG_CONTROL, base.CTRL_STOP, b""),
            (base.MSG_CONTROL, base.CTRL_DISABLE, b""),
            (base.MSG_PLOT, base.PLOT_STOP,
             bytes([base.FAST_MASK | base.NORMAL_MASK])),
        ):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError):
                pass

    def configure_plot(self):
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

    def stop_plot(self):
        try:
            self.request(
                base.MSG_PLOT,
                base.PLOT_STOP,
                bytes([base.FAST_MASK | base.NORMAL_MASK]),
            )
        except (TimeoutError, RuntimeError) as exc:
            print(f"PLOT_STOP warning: {exc}", file=sys.stderr)

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return

        cap = self.capture
        seq, = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        expected_len = 4 + sample_count * count * 2
        if len(payload) != expected_len:
            return

        if cap.fast_last is not None:
            expected_seq = (cap.fast_last + 1) & 0xFFFF
            cap.fast_lost += (seq - expected_seq) & 0xFFFF
        cap.fast_last = seq
        cap.fast_frames += 1
        cap.fast_samples += sample_count

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for sample in range(sample_count):
            start = sample * count
            values = [raw[start + index] * 0.001 for index in range(count)]
            cap.stats.add(values)

    def process_normal(self, payload):
        if (len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or
                payload[3] != 1):
            return

        cap = self.capture
        seq, = struct.unpack_from("<H", payload, 0)
        if cap.normal_last is not None:
            expected_seq = (cap.normal_last + 1) & 0xFFFF
            cap.normal_lost += (seq - expected_seq) & 0xFFFF
        cap.normal_last = seq
        cap.normal_frames += 1
        value, = struct.unpack_from("<f", payload, 4)
        if math.isfinite(value):
            cap.vbus.append(value)

    def collect_for(self, seconds):
        start = time.monotonic()
        deadline = start + seconds
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))
        return time.monotonic() - start

    def current_limit_set(self):
        self.request(
            base.MSG_CONTROL,
            CTRL_I_LIMIT_SET,
            struct.pack("<f", self.args.current_limit),
        )

    def ident_status(self):
        data = self.request(MSG_IDENTIFICATION, IDENT_STATUS)
        if len(data) != 11:
            raise RuntimeError(
                f"invalid Rs/Ls status response length: {len(data)}"
            )
        mode, state, valid = data[:3]
        if mode != IDENT_RS_LS:
            raise RuntimeError(f"unexpected identification mode: {mode}")
        rs_ohm, ls_h = struct.unpack_from("<ff", data, 3)
        return state, bool(valid), rs_ohm, ls_h

    def run_dynamic(self):
        self.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([MODE_IDENT]))
        self.request(MSG_IDENTIFICATION, IDENT_MODE_SET, bytes([IDENT_RS_LS]))
        self.current_limit_set()
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)

        self.reset_capture()
        self.request(base.MSG_CONTROL, base.CTRL_RUN)
        start = time.monotonic()
        next_status = start
        status = None

        try:
            while time.monotonic() - start < self.args.dynamic_timeout:
                now = time.monotonic()
                if now >= next_status:
                    status = self.ident_status()
                    next_status = now + self.args.poll_interval
                    if status[0] in (IDENT_DONE, IDENT_FAILED):
                        break
                else:
                    self.process(self.parser.feed(self.ser.read(4096)))

                peak = max(self.capture.stats.peak[:2])
                if peak > self.args.host_current_guard:
                    raise RuntimeError(
                        f"dynamic current {peak:.3f} A exceeded host guard "
                        f"{self.args.host_current_guard:.3f} A"
                    )
            else:
                raise TimeoutError("Rs/Ls dynamic excitation timeout")

            # Drain a short tail so the last telemetry block is accounted for.
            self.collect_for(0.05)
            return time.monotonic() - start, status
        finally:
            try:
                self.request(base.MSG_CONTROL, base.CTRL_DISABLE)
            except (TimeoutError, RuntimeError) as exc:
                print(f"DISABLE warning: {exc}", file=sys.stderr)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Automated static/dynamic validation of dual-ADC current sampling"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="static capture duration")
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--current-mean-limit", type=float, default=0.10,
                        help="maximum absolute zero-current mean for Ia/Ib [A]")
    parser.add_argument("--current-rms-limit", type=float, default=0.03,
                        help="maximum zero-current RMS noise for Ia/Ib [A]")
    parser.add_argument("--current-peak-limit", type=float, default=0.20,
                        help="maximum absolute zero-current peak for Ia/Ib [A]")
    parser.add_argument("--tail-threshold-a", type=float, default=0.20,
                        help="first static tail-count threshold [A]")
    parser.add_argument("--tail-threshold-b", type=float, default=0.30,
                        help="second static tail-count threshold [A]")
    parser.add_argument("--fast-rate-min", type=float, default=18000.0)
    parser.add_argument("--fast-rate-max", type=float, default=22000.0)
    parser.add_argument("--normal-rate-min", type=float, default=500.0)
    parser.add_argument("--max-fast-lost", type=int, default=0)
    parser.add_argument("--max-normal-lost", type=int, default=0)

    parser.add_argument("--dynamic", action="store_true",
                        help="run one guarded Rs/Ls excitation after static validation")
    parser.add_argument("--run", action="store_true",
                        help="required together with --dynamic to energize the motor")
    parser.add_argument("--current-limit", type=float,
                        help="dynamic commissioning current limit [A]")
    parser.add_argument("--dynamic-timeout", type=float, default=10.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--dynamic-min-activity", type=float, default=0.10,
                        help="minimum Ia and Ib peak proving dynamic activity [A]")

    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()

    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")
    for name in (
        "current_mean_limit", "current_rms_limit", "current_peak_limit",
        "tail_threshold_a", "tail_threshold_b", "fast_rate_min",
        "fast_rate_max", "normal_rate_min", "timeout", "dynamic_timeout",
        "poll_interval", "dynamic_min_activity",
    ):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.tail_threshold_a >= args.tail_threshold_b:
        parser.error("--tail-threshold-a must be less than --tail-threshold-b")
    if args.fast_rate_min >= args.fast_rate_max:
        parser.error("--fast-rate-min must be less than --fast-rate-max")
    if args.max_fast_lost < 0 or args.max_normal_lost < 0:
        parser.error("lost-frame limits must be non-negative")

    if args.dynamic:
        if not args.run:
            parser.error("--run is required with --dynamic")
        if args.current_limit is None or args.current_limit <= 0.0:
            parser.error("--current-limit must be positive with --dynamic")
        args.host_current_guard = HOST_CURRENT_GUARD_RATIO * args.current_limit
    else:
        args.host_current_guard = 0.0

    return args


def common_evaluate(cap, elapsed, args):
    failures = []
    if cap.stats.count == 0:
        failures.append("no FAST current samples")
    if not cap.vbus:
        failures.append("no Vbus samples")

    fast_rate = cap.fast_samples / elapsed if elapsed > 0.0 else 0.0
    normal_rate = cap.normal_frames / elapsed if elapsed > 0.0 else 0.0

    if not args.fast_rate_min <= fast_rate <= args.fast_rate_max:
        failures.append(
            f"FAST sample rate {fast_rate:.1f} Hz outside "
            f"{args.fast_rate_min:.1f} .. {args.fast_rate_max:.1f} Hz"
        )
    if normal_rate < args.normal_rate_min:
        failures.append(
            f"NORMAL update rate {normal_rate:.1f} Hz below "
            f"{args.normal_rate_min:.1f} Hz"
        )
    if cap.fast_lost > args.max_fast_lost:
        failures.append(f"FAST lost {cap.fast_lost} frames > {args.max_fast_lost}")
    if cap.normal_lost > args.max_normal_lost:
        failures.append(
            f"NORMAL lost {cap.normal_lost} frames > {args.max_normal_lost}"
        )

    if cap.vbus:
        mean_vbus = statistics.mean(cap.vbus)
        if not args.vbus_min <= mean_vbus <= args.vbus_max:
            failures.append(
                f"Vbus mean {mean_vbus:.3f} V outside "
                f"{args.vbus_min:.3f} .. {args.vbus_max:.3f} V"
            )

    return failures, fast_rate, normal_rate


def static_evaluate(cap, elapsed, args):
    failures, fast_rate, normal_rate = common_evaluate(cap, elapsed, args)
    if cap.stats.count:
        for index, name in enumerate(("Ia", "Ib")):
            mean = cap.stats.mean(index)
            rms = cap.stats.rms_noise(index)
            peak = cap.stats.peak[index]
            if abs(mean) > args.current_mean_limit:
                failures.append(
                    f"{name} mean {mean:+.4f} A exceeds "
                    f"{args.current_mean_limit:.4f} A"
                )
            if rms > args.current_rms_limit:
                failures.append(
                    f"{name} RMS noise {rms:.4f} A exceeds "
                    f"{args.current_rms_limit:.4f} A"
                )
            if peak > args.current_peak_limit:
                failures.append(
                    f"{name} peak {peak:.4f} A exceeds "
                    f"{args.current_peak_limit:.4f} A"
                )
    return failures, fast_rate, normal_rate


def dynamic_evaluate(cap, elapsed, ident_status, args):
    failures, fast_rate, normal_rate = common_evaluate(cap, elapsed, args)
    if cap.stats.count:
        for index, name in enumerate(("Ia", "Ib")):
            if cap.stats.peak[index] < args.dynamic_min_activity:
                failures.append(
                    f"{name} peak {cap.stats.peak[index]:.3f} A below dynamic "
                    f"activity threshold {args.dynamic_min_activity:.3f} A"
                )
        peak = max(cap.stats.peak[:2])
        if peak > args.host_current_guard:
            failures.append(
                f"dynamic peak {peak:.3f} A exceeds host guard "
                f"{args.host_current_guard:.3f} A"
            )

    if ident_status is None:
        failures.append("no Rs/Ls identification status")
    elif ident_status[0] == IDENT_FAILED:
        failures.append("Rs/Ls identification reported FAILED during excitation")
    elif ident_status[0] != IDENT_DONE:
        failures.append(f"unexpected Rs/Ls terminal state {ident_status[0]}")

    return failures, fast_rate, normal_rate


def print_capture(title, cap, elapsed, fast_rate, normal_rate, args, static):
    print(f"\n{title}")
    print(f"  duration={elapsed:.3f} s")
    print(
        f"  FAST: samples={cap.fast_samples} frames={cap.fast_frames} "
        f"lost={cap.fast_lost} rate={fast_rate:.1f} Hz"
    )
    print(
        f"  NORMAL: frames={cap.normal_frames} lost={cap.normal_lost} "
        f"rate={normal_rate:.1f} Hz"
    )

    if cap.stats.count:
        for index, name in enumerate(("Ia", "Ib", "Ic")):
            extra = ""
            if static and index < 2:
                extra = (
                    f" p99={cap.stats.p99(index):.5f} A"
                    f" p99.9={cap.stats.p999(index):.5f} A"
                    f" >{args.tail_threshold_a:.2f}A="
                    f"{cap.stats.above(index, args.tail_threshold_a)}"
                    f" >{args.tail_threshold_b:.2f}A="
                    f"{cap.stats.above(index, args.tail_threshold_b)}"
                )
            print(
                f"  {name}: mean={cap.stats.mean(index):+.5f} A "
                f"rms={cap.stats.rms_noise(index):.5f} A "
                f"peak={cap.stats.peak[index]:.5f} A{extra}"
            )
        print("  note: Ic is reconstructed from Ia/Ib, not an independent ADC check")

    if cap.vbus:
        print(
            f"  Vbus={statistics.mean(cap.vbus):.3f} V "
            f"({min(cap.vbus):.3f} .. {max(cap.vbus):.3f} V)"
        )


def report_result(name, failures):
    if failures:
        print(f"\n{name}: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return False
    print(f"\n{name}: PASS")
    return True


def main():
    args = parse_args()
    overall = True

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

            test = DualADCTest(ser, args)
            test.prepare()
            test.configure_plot()
            try:
                test.reset_capture()
                static_elapsed = test.collect_for(args.seconds)
                static_cap = test.capture
                static_failures, fast_rate, normal_rate = static_evaluate(
                    static_cap, static_elapsed, args
                )
                print_capture(
                    "Dual-ADC static result", static_cap, static_elapsed,
                    fast_rate, normal_rate, args, True
                )
                overall &= report_result("STATIC", static_failures)

                if args.dynamic:
                    if not overall:
                        print("\nDynamic phase skipped because static validation failed.")
                    else:
                        print(
                            f"\nStarting guarded dynamic phase: current-limit="
                            f"{args.current_limit:.3f} A, host-guard="
                            f"{args.host_current_guard:.3f} A"
                        )
                        dynamic_elapsed, ident_status = test.run_dynamic()
                        dynamic_cap = test.capture
                        failures, fast_rate, normal_rate = dynamic_evaluate(
                            dynamic_cap, dynamic_elapsed, ident_status, args
                        )
                        print_capture(
                            "Dual-ADC dynamic result", dynamic_cap,
                            dynamic_elapsed, fast_rate, normal_rate,
                            args, False
                        )
                        if ident_status is not None:
                            print(
                                f"  Rs/Ls terminal: state={ident_status[0]} "
                                f"valid={int(ident_status[1])} "
                                f"Rs={ident_status[2]:.7g} ohm "
                                f"Ls={ident_status[3] * 1.0e6:.4f} uH"
                            )
                            print("  note: identification result was NOT applied")
                        overall &= report_result("DYNAMIC", failures)
            finally:
                test.stop_plot()
                test.prepare()

        if not overall:
            raise SystemExit(1)
        print("\nOVERALL: PASS")

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
