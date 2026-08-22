#!/usr/bin/env python3
"""Capture NLOB/PLL Shadow signals for twice-electrical-frequency analysis."""

import argparse
import cmath
import csv
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import time

import sensorless_test as base


CUR_TS = 50.0e-6
BLOCK_SAMPLES = 20
FAST_FS_HZ = 1000.0
NORMAL_FS_HZ = 1000.0
FAST_CONFIG_ID = 5
NORMAL_CONFIG_ID = 6

FAST_VARS = (
    ("Theta_IF", 0x0014, 0.0002),
    ("Theta_obs", 0x0020, 0.0002),
    ("We_obs", 0x0021, 0.1),
    ("PLL_Err", 0x0022, 0.0001),
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
)

NORMAL_VARS = (
    ("Vbus", 0x0004),
    ("Flux_Err", 0x0023),
    ("Ic", 0x0003),
    ("Va_meas", 0x0120),
    ("Vb_meas", 0x0121),
    ("Vc_meas", 0x0122),
    ("Va_cmd_f", 0x0123),
    ("Vb_cmd_f", 0x0124),
    ("Vc_cmd_f", 0x0125),
    ("ErrA", 0x0126),
    ("ErrB", 0x0127),
    ("ErrC", 0x0128),
)

FAST_LINEAR = (2, 3, 4, 5)
FAST_ANGLE = (0, 1)
FAST_INDEX = {name: index for index, (name, _, _) in enumerate(FAST_VARS)}
NORMAL_INDEX = {name: index for index, (name, _) in enumerate(NORMAL_VARS)}
FAST_POS = {name: index + 2 for index, (name, _, _) in enumerate(FAST_VARS)}
NORMAL_POS = {name: index + 2 for index, (name, _) in enumerate(NORMAL_VARS)}


def angle_wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def git_info(repo):
    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip()
        status = subprocess.check_output(
            ["git", "status", "--short"], cwd=repo, text=True
        ).strip()
        return head, bool(status)
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True


def spectrum(values, sample_rate, max_frequency=100.0):
    count = len(values)
    if count < 4:
        return []

    mean = sum(values) / count
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * n / (count - 1))
              for n in range(count)]
    window_sum = sum(window)
    max_bin = min(count // 2, int(max_frequency * count / sample_rate))
    result = []

    for k in range(1, max_bin + 1):
        step = cmath.rect(1.0, -2.0 * math.pi * k / count)
        phase = 1.0 + 0.0j
        value_sum = 0.0 + 0.0j

        for value, weight in zip(values, window):
            value_sum += (value - mean) * weight * phase
            phase *= step

        result.append((k * sample_rate / count,
                       2.0 * abs(value_sum) / window_sum))

    return result


def spectrum_summary(points, target_frequency):
    if not points:
        return {}

    peak_frequency, peak_amplitude = max(points, key=lambda item: item[1])
    target_point = min(points, key=lambda item: abs(item[0] - target_frequency))
    return {
        "peak_frequency_hz": peak_frequency,
        "peak_amplitude": peak_amplitude,
        "target_2fe_frequency_hz": target_point[0],
        "target_2fe_amplitude": target_point[1],
    }


def linear_solve(matrix, vector):
    count = len(vector)
    work = [matrix[row][:] + [vector[row]] for row in range(count)]

    for column in range(count):
        pivot = max(range(column, count),
                    key=lambda row: abs(work[row][column]))
        if abs(work[pivot][column]) < 1.0e-12:
            raise RuntimeError("phase-voltage harmonic fit is singular")
        work[column], work[pivot] = work[pivot], work[column]

        scale = work[column][column]
        for item in range(column, count + 1):
            work[column][item] /= scale

        for row in range(count):
            if row == column:
                continue
            scale = work[row][column]
            for item in range(column, count + 1):
                work[row][item] -= scale * work[column][item]

    return [work[row][count] for row in range(count)]


def harmonic_fit(theta, values):
    harmonics = (1, 2, 6)
    columns = 1 + 2 * len(harmonics)
    normal = [[0.0] * columns for _ in range(columns)]
    target = [0.0] * columns

    for angle, value in zip(theta, values):
        row = [1.0]
        for harmonic in harmonics:
            row.extend((math.cos(harmonic * angle),
                        math.sin(harmonic * angle)))

        for lhs in range(columns):
            target[lhs] += row[lhs] * value
            for rhs in range(columns):
                normal[lhs][rhs] += row[lhs] * row[rhs]

    coefficient = linear_solve(normal, target)
    result = {"dc_v": coefficient[0]}

    for index, harmonic in enumerate(harmonics):
        cosine = coefficient[1 + 2 * index]
        sine = coefficient[2 + 2 * index]
        result[f"{harmonic}fe"] = {
            "amplitude_v": math.hypot(cosine, sine),
            "phase_deg": math.degrees(math.atan2(cosine, sine)),
        }

    return result


def sign_current_fit(current, error, current_min):
    selected = [(1.0 if phase_current > 0.0 else -1.0, voltage_error)
                for phase_current, voltage_error in zip(current, error)
                if abs(phase_current) >= current_min]
    if len(selected) < 4:
        return {"samples": len(selected)}

    sign_mean = sum(item[0] for item in selected) / len(selected)
    error_mean = sum(item[1] for item in selected) / len(selected)
    sign_var = sum((item[0] - sign_mean) ** 2 for item in selected)
    error_var = sum((item[1] - error_mean) ** 2 for item in selected)
    covariance = sum((item[0] - sign_mean) * (item[1] - error_mean)
                     for item in selected)
    gain = covariance / sign_var if sign_var > 0.0 else 0.0
    correlation = 0.0
    if sign_var > 0.0 and error_var > 0.0:
        correlation = covariance / math.sqrt(sign_var * error_var)

    return {
        "samples": len(selected),
        "current_min_a": current_min,
        "offset_v": error_mean - gain * sign_mean,
        "gain_v": gain,
        "correlation": correlation,
    }


def signal_summary(values):
    mean = sum(values) / len(values)
    rms = math.sqrt(sum(value * value for value in values) / len(values))
    return {
        "mean": mean,
        "rms": rms,
        "min": min(values),
        "max": max(values),
    }


class Observer2FeTest(base.SensorlessTest):
    def __init__(self, ser, args, head, dirty):
        super().__init__(ser, args)
        self.head = head
        self.dirty = dirty
        self.fast_rows = []
        self.normal_rows = []
        self.fast_motion_samples = 0
        self.normal_motion_samples = 0
        self.fast_block_count = 0
        self.fast_sum = [0.0] * len(FAST_VARS)
        self.fast_sin = [0.0] * len(FAST_ANGLE)
        self.fast_cos = [0.0] * len(FAST_ANGLE)
        self.fast_saturation = [0] * len(FAST_VARS)
        self.normal_frames = 0
        self.normal_lost = 0
        self.normal_last = None

    def configure_plot(self):
        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([
            base.NORMAL_GROUP,
            NORMAL_CONFIG_ID,
            len(NORMAL_VARS),
        ])
        normal_data += b"".join(
            struct.pack("<H", var_id) for _, var_id in NORMAL_VARS
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(
            base.MSG_PLOT,
            base.PLOT_START,
            bytes([base.FAST_MASK | base.NORMAL_MASK]),
        )

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
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
            start = sample * channel_count
            sample_raw = raw[start:start + channel_count]
            values = [
                sample_raw[index] * FAST_VARS[index][2]
                for index in range(channel_count)
            ]

            for index, value in enumerate(sample_raw):
                if value in (-32768, 32767):
                    self.fast_saturation[index] += 1

            if not self.motion_active:
                continue

            self.fast_motion_samples += 1
            ia = values[FAST_INDEX["Ia"]]
            ib = values[FAST_INDEX["Ib"]]
            ic_rebuilt = -ia - ib
            phase_abs = max(abs(ia), abs(ib), abs(ic_rebuilt))
            self.phase_peak = max(self.phase_peak, phase_abs)
            if phase_abs > self.args.phase_limit:
                self.tripped = True

            for index in FAST_LINEAR:
                self.fast_sum[index] += values[index]
            for angle_index, value_index in enumerate(FAST_ANGLE):
                self.fast_sin[angle_index] += math.sin(values[value_index])
                self.fast_cos[angle_index] += math.cos(values[value_index])

            self.fast_block_count += 1
            if self.fast_block_count == BLOCK_SAMPLES:
                self._finish_fast_block()

    def _finish_fast_block(self):
        row = [0.0] * len(FAST_VARS)
        for index in FAST_LINEAR:
            row[index] = self.fast_sum[index] / BLOCK_SAMPLES
        for angle_index, value_index in enumerate(FAST_ANGLE):
            row[value_index] = math.atan2(
                self.fast_sin[angle_index],
                self.fast_cos[angle_index],
            ) % (2.0 * math.pi)

        elapsed = self.fast_motion_samples * CUR_TS
        self.fast_rows.append((elapsed, self.stage_name(), *row))
        self.fast_block_count = 0
        self.fast_sum = [0.0] * len(FAST_VARS)
        self.fast_sin = [0.0] * len(FAST_ANGLE)
        self.fast_cos = [0.0] * len(FAST_ANGLE)

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

        values = struct.unpack_from(f"<{len(NORMAL_VARS)}f", payload, 4)
        self.vbus.append(values[NORMAL_INDEX["Vbus"]])

        if not self.motion_active:
            return

        self.normal_motion_samples += 1
        ic = values[NORMAL_INDEX["Ic"]]
        self.phase_peak = max(self.phase_peak, abs(ic))
        if abs(ic) > self.args.phase_limit:
            self.tripped = True

        elapsed = self.normal_motion_samples / NORMAL_FS_HZ
        self.normal_rows.append((elapsed, self.stage_name(), *values))

    def analyze(self, direction):
        fast_hold = [row for row in self.fast_rows if row[1] == "HOLD"]
        normal_hold = [row for row in self.normal_rows if row[1] == "HOLD"]
        if not fast_hold or not normal_hold:
            raise RuntimeError("HOLD data missing")

        fast_end = fast_hold[-1][0]
        normal_end = normal_hold[-1][0]
        fast = [row for row in fast_hold
                if row[0] >= fast_end - self.args.analysis_seconds]
        normal = [row for row in normal_hold
                  if row[0] >= normal_end - self.args.analysis_seconds]

        fast_signal = {
            "We_obs": [row[FAST_POS["We_obs"]] for row in fast],
            "PLL_Err": [row[FAST_POS["PLL_Err"]] for row in fast],
            "Delta_Theta": [
                angle_wrap(row[FAST_POS["Theta_obs"]] -
                           row[FAST_POS["Theta_IF"]])
                for row in fast
            ],
            "Ialpha": [row[FAST_POS["Ia"]] for row in fast],
            "Ibeta": [
                (row[FAST_POS["Ia"]] + 2.0 * row[FAST_POS["Ib"]]) /
                math.sqrt(3.0)
                for row in fast
            ],
        }
        fast_signal["I_Mag"] = [
            math.hypot(alpha, beta)
            for alpha, beta in zip(fast_signal["Ialpha"], fast_signal["Ibeta"])
        ]

        normal_signal = {
            "Flux_Err": [row[NORMAL_POS["Flux_Err"]] for row in normal],
            "ErrA": [row[NORMAL_POS["ErrA"]] for row in normal],
            "ErrB": [row[NORMAL_POS["ErrB"]] for row in normal],
            "ErrC": [row[NORMAL_POS["ErrC"]] for row in normal],
        }

        mean_we = sum(fast_signal["We_obs"]) / len(fast)
        fe = abs(mean_we) / (2.0 * math.pi)
        target_2fe = 2.0 * fe
        target = 120.0 if direction == 1 else -120.0
        command_fe = abs(target) / (2.0 * math.pi)
        command_2fe = 2.0 * command_fe
        spectra = {}
        spectrum_rows = []

        for name in ("We_obs", "PLL_Err", "Delta_Theta", "I_Mag"):
            points = spectrum(fast_signal[name], FAST_FS_HZ)
            spectra[name] = spectrum_summary(points, target_2fe)
            spectrum_rows.extend((name, frequency, amplitude)
                                 for frequency, amplitude in points)

        for name in ("Flux_Err",):
            points = spectrum(normal_signal[name], NORMAL_FS_HZ)
            spectra[name] = spectrum_summary(points, target_2fe)
            spectrum_rows.extend((name, frequency, amplitude)
                                 for frequency, amplitude in points)

        for name in ("ErrA", "ErrB", "ErrC"):
            points = spectrum(normal_signal[name], NORMAL_FS_HZ)
            spectra[name] = spectrum_summary(points, command_2fe)
            spectrum_rows.extend((name, frequency, amplitude)
                                 for frequency, amplitude in points)

        theta_if = []
        phase_current = {"A": [], "B": [], "C": []}
        phase_error = {"A": [], "B": [], "C": []}
        fast_index = 0
        alignment_max = 0.0
        alignment_offset = fast[-1][0] - normal[-1][0]

        for normal_row in normal:
            target_fast_time = normal_row[0] + alignment_offset
            while (fast_index + 1 < len(fast) and
                   abs(fast[fast_index + 1][0] - target_fast_time) <=
                   abs(fast[fast_index][0] - target_fast_time)):
                fast_index += 1

            fast_row = fast[fast_index]
            alignment_max = max(alignment_max,
                                abs(fast_row[0] - target_fast_time))
            ia = fast_row[FAST_POS["Ia"]]
            ib = fast_row[FAST_POS["Ib"]]

            theta_if.append(fast_row[FAST_POS["Theta_IF"]])
            phase_current["A"].append(ia)
            phase_current["B"].append(ib)
            phase_current["C"].append(normal_row[NORMAL_POS["Ic"]])
            phase_error["A"].append(normal_row[NORMAL_POS["ErrA"]])
            phase_error["B"].append(normal_row[NORMAL_POS["ErrB"]])
            phase_error["C"].append(normal_row[NORMAL_POS["ErrC"]])

        phase_voltage = {
            "samples": len(theta_if),
            "alignment_max_s": alignment_max,
            "alignment_offset_s": alignment_offset,
            "signals": {
                name: signal_summary(
                    [row[NORMAL_POS[name]] for row in normal]
                )
                for name in (
                    "Va_meas", "Vb_meas", "Vc_meas",
                    "Va_cmd_f", "Vb_cmd_f", "Vc_cmd_f",
                    "ErrA", "ErrB", "ErrC",
                )
            },
            "harmonics": {
                f"Err{phase}": harmonic_fit(theta_if, phase_error[phase])
                for phase in ("A", "B", "C")
            },
            "sign_current": {
                f"Err{phase}_vs_I{phase.lower()}": sign_current_fit(
                    phase_current[phase],
                    phase_error[phase],
                    self.args.sign_current_min,
                )
                for phase in ("A", "B", "C")
            },
        }

        flux_err_mean = sum(normal_signal["Flux_Err"]) / len(normal)
        result = {
            "head": self.head,
            "dirty": self.dirty,
            "direction": self.args.direction,
            "analysis_window_s": self.args.analysis_seconds,
            "observer_voltage_delay_samples": 1,
            "observer_voltage_delay_s": CUR_TS,
            "fast_samples": self.fast_samples,
            "fast_frames": self.fast_frames,
            "fast_seq_lost": self.fast_lost,
            "normal_frames": self.normal_frames,
            "normal_seq_lost": self.normal_lost,
            "phase_peak_a": self.phase_peak,
            "phase_limit_a": self.args.phase_limit,
            "fast_saturation": {
                FAST_VARS[index][0]: count
                for index, count in enumerate(self.fast_saturation)
            },
            "we_target_rad_s": target,
            "we_mean_rad_s": mean_we,
            "we_error_rad_s": mean_we - target,
            "fe_hz": fe,
            "target_2fe_hz": target_2fe,
            "command_fe_hz": command_fe,
            "command_2fe_hz": command_2fe,
            "flux_err_mean_percent_squared": (
                flux_err_mean / (self.args.flux_wb * self.args.flux_wb) * 100.0
            ),
            "spectra": spectra,
            "phase_voltage": phase_voltage,
            "checks": {
                "fast_frames_complete": self.fast_lost == 0,
                "normal_frames_complete": self.normal_lost == 0,
                "phase_current_safe": self.phase_peak <= self.args.phase_limit,
                "plot_not_saturated": not any(self.fast_saturation),
                "observer_direction": mean_we * target > 0.0,
            },
        }
        return result, spectrum_rows

    def save(self, result, spectrum_rows):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = f"observer_2fe_{self.args.direction}_{stamp}"
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        fast_path = output_dir / f"{stem}_fast.csv"
        normal_path = output_dir / f"{stem}_normal.csv"
        spectrum_path = output_dir / f"{stem}_spectrum.csv"
        json_path = output_dir / f"{stem}.json"

        with fast_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time_s", "stage", *(item[0] for item in FAST_VARS)))
            writer.writerows(self.fast_rows)

        with normal_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time_s", "stage", *(item[0] for item in NORMAL_VARS)))
            writer.writerows(self.normal_rows)

        with spectrum_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("signal", "frequency_hz", "amplitude"))
            writer.writerows(spectrum_rows)

        with json_path.open("w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
            handle.write("\n")

        print(f"FAST:     {fast_path}")
        print(f"NORMAL:   {normal_path}")
        print(f"SPECTRUM: {spectrum_path}")
        print(f"JSON:     {json_path}")

    def print_analysis(self, result):
        print("\n2fe analysis")
        print(
            f"We mean={result['we_mean_rad_s']:.4f} rad/s "
            f"error={result['we_error_rad_s']:+.4f} rad/s, "
            f"fe={result['fe_hz']:.3f} Hz "
            f"2fe={result['target_2fe_hz']:.3f} Hz"
        )
        print(
            f"FluxErr/Flux^2={result['flux_err_mean_percent_squared']:+.3f}%"
        )
        for name, item in result["spectra"].items():
            print(
                f"{name}: peak={item['peak_frequency_hz']:.3f} Hz "
                f"amp={item['peak_amplitude']:.7g}, "
                f"at 2fe={item['target_2fe_amplitude']:.7g}"
            )
        print(
            f"FAST lost={self.fast_lost}, NORMAL lost={self.normal_lost}, "
            f"phase peak={self.phase_peak:.3f} A"
        )
        print(
            f"Phase voltage samples={result['phase_voltage']['samples']} "
            f"alignment max={result['phase_voltage']['alignment_max_s'] * 1e3:.3f} ms"
        )
        for phase in ("A", "B", "C"):
            harmonic = result["phase_voltage"]["harmonics"][f"Err{phase}"]
            sign_fit = result["phase_voltage"]["sign_current"][
                f"Err{phase}_vs_I{phase.lower()}"
            ]
            sign_text = f"samples={sign_fit['samples']}"
            if "gain_v" in sign_fit:
                sign_text += (
                    f" K={sign_fit['gain_v']:+.5f} V "
                    f"corr={sign_fit['correlation']:+.3f}"
                )
            print(
                f"Err{phase}: DC={harmonic['dc_v']:+.5f} V, "
                f"1fe={harmonic['1fe']['amplitude_v']:.5f} V, "
                f"2fe={harmonic['2fe']['amplitude_v']:.5f} V, "
                f"6fe={harmonic['6fe']['amplitude_v']:.5f} V, "
                f"{sign_text}"
            )
        for name, passed in result["checks"].items():
            print(f"{name}: {'PASS' if passed else 'WARN'}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded NLOB/PLL Shadow 2fe diagnostic capture"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward")
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--analysis-seconds", type=float, default=2.0)
    parser.add_argument("--phase-limit", type=float, default=2.2)
    parser.add_argument("--sign-current-min", type=float, default=0.1)
    parser.add_argument("--flux-wb", type=float, default=0.0031891402)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.05)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output-dir", default="build/Release")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.hold_seconds <= 0.0:
        parser.error("--hold-seconds must be positive")
    if not 0.0 < args.analysis_seconds <= args.hold_seconds:
        parser.error("--analysis-seconds must be in (0, hold-seconds]")
    if args.phase_limit <= 0.0 or args.flux_wb <= 0.0:
        parser.error("phase limit and flux must be positive")
    if args.sign_current_min < 0.0:
        parser.error("sign-current-min must be non-negative")
    if args.vbus_seconds <= 0.0 or args.ready_timeout <= 0.0:
        parser.error("Vbus and ready timeouts must be positive")
    if args.poll_interval <= 0.0:
        parser.error("poll interval must be positive")
    return args


def main():
    args = parse_args()
    direction = 1 if args.direction == "forward" else 2
    repo = Path(__file__).resolve().parents[1]
    head, dirty = git_info(repo)

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

            test = Observer2FeTest(ser, args, head, dirty)
            error = None
            try:
                test.configure_plot()
                test.check_vbus()
                test.run(direction)
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop()

            if error is not None:
                raise error

            result, spectrum_rows = test.analyze(direction)
            test.print_analysis(result)
            test.save(result, spectrum_rows)

            if test.fast_lost or test.normal_lost:
                raise RuntimeError(
                    f"plot lost FAST={test.fast_lost} NORMAL={test.normal_lost}"
                )

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
