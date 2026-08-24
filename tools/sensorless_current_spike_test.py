#!/usr/bin/env python3
"""Capture a 20 kHz three-phase current window around a high-speed spike."""

import argparse
from collections import deque
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
SENSORLESS_RUN = 3
FAST_CONFIG_ID = 9
NORMAL_CONFIG_ID = 10

FAST_VARS = (
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
    ("Ic", 0x0003, 0.001),
    ("Id", 0x0010, 0.001),
    ("Iq", 0x0011, 0.001),
    ("Theta_e", 0x0014, 0.0002),
    ("We_obs", 0x0021, 0.1),
    ("PLL_Err", 0x0022, 0.0001),
)
FAST_INDEX = {name: index for index, (name, _, _) in enumerate(FAST_VARS)}
VBUS_ID = 0x0004


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def git_head(repo):
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class CurrentSpikeTest(base.SensorlessTest):
    def __init__(self, ser, args):
        super().__init__(ser, args)
        self.raw = deque(maxlen=args.pre_samples + 20)
        self.sample_index = 0
        self.trigger_index = None
        self.trigger_target = None
        self.capture_done = False
        self.latest_we = 0.0
        self.latest_pll = 0.0
        self.command_target = base.IF_WE_RAD_S

    def configure_plot(self):
        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(
            struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS
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
        block_triggered = False

        for sample in range(sample_count):
            start = sample * count
            values = tuple(
                raw[start + index] * FAST_VARS[index][2]
                for index in range(count)
            )
            ia = values[FAST_INDEX["Ia"]]
            ib = values[FAST_INDEX["Ib"]]
            ic = values[FAST_INDEX["Ic"]]
            d_axis = values[FAST_INDEX["Id"]]
            q_axis = values[FAST_INDEX["Iq"]]
            i_mag = math.hypot(d_axis, q_axis)
            i_sum = ia + ib + ic

            self.phase_peak = max(self.phase_peak, abs(ia), abs(ib), abs(ic))
            self.latest_we = values[FAST_INDEX["We_obs"]]
            self.latest_pll = values[FAST_INDEX["PLL_Err"]]
            self.raw.append((self.sample_index, *values, i_sum, i_mag))

            if self.trigger_index is None and i_mag > self.args.current_limit:
                self.trigger_index = self.sample_index
                self.trigger_target = self.command_target
                block_triggered = True

            self.sample_index += 1

        if block_triggered:
            self.capture_done = True
            self.tripped = True

    def process_normal(self, payload):
        if (len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or
                payload[3] != 1):
            return
        value, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(value)

    def speed_set(self, we_target):
        self.command_target = float(we_target)
        super().speed_set(self.command_target / self.args.pole_pairs)

    def read_stage(self):
        data = self.request(base.MSG_SENSORLESS, base.SENSORLESS_STATUS)
        if len(data) != 8:
            raise RuntimeError(f"invalid Sensorless status length: {len(data)}")

        active, ready, stage, if_stage = data[:4]
        self.stage = stage
        self.if_stage = if_stage
        self.ready = bool(ready)
        if not active:
            raise RuntimeError("Sensorless stopped during capture")
        return stage

    def pump(self, seconds, require_run=True):
        deadline = time.monotonic() + seconds
        next_status = time.monotonic()

        while time.monotonic() < deadline and not self.capture_done:
            now = time.monotonic()
            if now >= next_status:
                stage = self.read_stage()
                if require_run and stage != SENSORLESS_RUN:
                    raise RuntimeError(f"left SENSORLESS_RUN: stage={stage}")
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if abs(self.latest_pll) > self.args.pll_limit:
                raise RuntimeError(
                    f"PLL error {self.latest_pll:.4f} exceeded "
                    f"{self.args.pll_limit:.4f}"
                )
            if require_run and self.latest_we < 0.0:
                raise RuntimeError("observer direction became negative")

    def wait_run(self):
        deadline = time.monotonic() + self.args.ready_timeout
        next_status = time.monotonic()

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                stage = self.read_stage()
                if stage == SENSORLESS_RUN:
                    return
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))
            if self.capture_done:
                raise RuntimeError("current spike occurred before SENSORLESS_RUN")
        raise TimeoutError("Sensorless did not enter SENSORLESS_RUN")

    def run_forward(self):
        self.request(base.MSG_CONTROL, base.CTRL_MODE_SET,
                     bytes([base.MODE_SENSORLESS_SPEED]))
        self.speed_set(base.IF_WE_RAD_S)
        self.request(base.MSG_CONTROL, base.CTRL_ENABLE)
        self.request(base.MSG_CONTROL, base.CTRL_RUN)
        print("ALIGN -> I/F 120 rad/s -> Observer takeover")
        self.wait_run()
        print("SENSORLESS_RUN reached")

        target = int(base.IF_WE_RAD_S)
        while target < self.args.current_target and not self.capture_done:
            target = min(target + self.args.ramp_step,
                         self.args.current_target)
            self.speed_set(target)
            self.pump(self.args.ramp_interval)
            if target % 200 == 0:
                print(f"  target={target} rad/s, We_obs={self.latest_we:.1f}")

        if not self.capture_done:
            print(
                f"holding {self.args.current_target} rad/s for "
                f"{self.args.hold_seconds:.2f} s"
            )
            self.pump(self.args.hold_seconds)

    def stop_all(self):
        for msg_type, op, data in (
            (base.MSG_CONTROL, base.CTRL_STOP, b""),
            (base.MSG_CONTROL, base.CTRL_DISABLE, b""),
            (base.MSG_PLOT, base.PLOT_STOP,
             bytes([base.FAST_MASK | base.NORMAL_MASK])),
        ):
            try:
                self.request(msg_type, op, data)
            except (TimeoutError, RuntimeError) as exc:
                print(f"STOP warning: {exc}", file=sys.stderr)

    def analyze(self):
        rows = list(self.raw)
        if not rows:
            raise RuntimeError("no raw current samples")

        if self.trigger_index is None:
            center = rows[-1][0]
        else:
            center = self.trigger_index

        max_row = max(rows, key=lambda row: row[-1])
        above = [row for row in rows if row[-1] > self.args.current_limit]
        consecutive_max = 0
        consecutive = 0
        last_index = None
        for row in above:
            if last_index is not None and row[0] == last_index + 1:
                consecutive += 1
            else:
                consecutive = 1
            consecutive_max = max(consecutive_max, consecutive)
            last_index = row[0]

        pre = [row for row in rows
               if row[0] < center - 20 and row[0] >= center - 2000]
        if not pre:
            pre = rows[:max(1, len(rows) // 2)]

        names = [item[0] for item in FAST_VARS]
        offset = 1
        max_values = {
            names[index]: max_row[offset + index]
            for index in range(len(names))
        }
        sum_rms_pre = rms([row[-2] for row in pre])
        result = {
            "triggered": self.trigger_index is not None,
            "trigger_sample": self.trigger_index,
            "trigger_target_we_rad_s": self.trigger_target,
            "trigger_target_wm_rad_s": (
                self.trigger_target / self.args.pole_pairs
                if self.trigger_target is not None else None
            ),
            "current_limit_a": self.args.current_limit,
            "max_i_mag_a": max_row[-1],
            "max_i_mag_sample": max_row[0],
            "samples_above_limit": len(above),
            "max_consecutive_samples_above_limit": consecutive_max,
            "max_consecutive_time_us": consecutive_max * CUR_TS * 1.0e6,
            "values_at_max": max_values,
            "i_sum_at_max_a": max_row[-2],
            "i_sum_pre_rms_a": sum_rms_pre,
            "phase_abs_max_a": {
                name: max(abs(row[offset + FAST_INDEX[name]]) for row in rows)
                for name in ("Ia", "Ib", "Ic")
            },
            "vbus_mean_v": sum(self.vbus) / len(self.vbus),
            "fast_seq_lost": self.fast_lost,
            "raw_samples_saved": len(rows),
        }
        return result

    def save(self, result):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        stem = f"sensorless_current_spike_{stamp}"
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f"{stem}_raw.csv"
        json_path = output_dir / f"{stem}.json"
        rows = list(self.raw)
        center = self.trigger_index if self.trigger_index is not None else rows[-1][0]

        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("sample", "trigger_time_s",
                             *(item[0] for item in FAST_VARS),
                             "I_sum", "I_mag"))
            for row in rows:
                writer.writerow((row[0], (row[0] - center) * CUR_TS,
                                 *row[1:]))

        record = {
            "head": git_head(Path(__file__).resolve().parents[1]),
            "pole_pairs": self.args.pole_pairs,
            "result": result,
        }
        json_path.write_text(json.dumps(record, indent=2) + "\n",
                             encoding="utf-8")
        return csv_path, json_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture raw 20 kHz phase currents around a speed spike"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--pole-pairs", type=int, default=16)
    parser.add_argument("--current-target", type=int, default=2300,
                        help="electrical rad/s test point; command is converted to Wm")
    parser.add_argument("--current-limit", type=float, default=2.6)
    parser.add_argument("--ramp-step", type=int, default=5,
                        help="electrical rad/s test increment")
    parser.add_argument("--ramp-interval", type=float, default=0.02)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--pre-samples", type=int, default=4000)
    parser.add_argument("--pll-limit", type=float, default=0.08)
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--phase-limit", type=float, default=2.6)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-seconds", type=float, default=0.2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--output-dir", default="build/Release")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    if args.pole_pairs <= 0:
        parser.error("--pole-pairs must be positive")
    if not 120 <= args.current_target <= 3000:
        parser.error("current target must be in 120..3000 electrical rad/s")
    if (args.current_limit <= 0.0 or args.ramp_step <= 0 or
            args.ramp_interval <= 0.0 or args.hold_seconds <= 0.0 or
            args.pre_samples < 100):
        parser.error("capture and ramp parameters must be positive")
    return args


def main():
    args = parse_args()
    test = None
    error = None

    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = CurrentSpikeTest(ser, args)
            try:
                test.configure_plot()
                test.check_vbus()
                test.run_forward()
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop_all()

            result = test.analyze()
            paths = test.save(result)
            print(json.dumps(result, indent=2))
            print(f"RAW:  {paths[0]}")
            print(f"JSON: {paths[1]}")

            if error is not None:
                raise error

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
