#!/usr/bin/env python3
"""Build, optionally flash, identify Rs/Ls, validate, and apply over USB CDC.

The motor is energized only when --run is present.  The script keeps one USB
session for the complete hardware workflow and always attempts DISABLE before
leaving that session.

Example:
    python3 tools/rl_apply_test.py \
        --port /dev/ttyACM0 --run --apply --flash
"""

import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: python -m pip install pyserial",
          file=sys.stderr)
    raise SystemExit(2)


MAGIC = b"AXDR"
NODE_ID = 1

MSG_RESPONSE = 0x02
MSG_CONTROL = 0x03
MSG_PLOT = 0x04
MSG_IDENTIFICATION = 0x05
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18

CTRL_ENABLE = 0x01
CTRL_RUN = 0x02
CTRL_DISABLE = 0x04
CTRL_MODE_SET = 0x05

MODE_IDENT = 4

IDENT_MODE_SET = 0x01
IDENT_STATUS = 0x02
IDENT_APPLY = 0x04
IDENT_RS_LS = 0x01

IDENT_DONE = 2
IDENT_FAILED = 3

PLOT_CONFIG = 0x01
PLOT_START = 0x02
PLOT_STOP = 0x03

FAST_GROUP = 0
NORMAL_GROUP = 1
FAST_MASK = 1 << FAST_GROUP
NORMAL_MASK = 1 << NORMAL_GROUP
FAST_CONFIG_ID = 1
NORMAL_CONFIG_ID = 2

FAST_VARS = (
    ("Ia", 0x0001),
    ("Ib", 0x0002),
    ("Ic", 0x0003),
)
VBUS_ID = 0x0004

RS_LS_STAGE = {
    0: "IDLE",
    1: "PROBE_RAMP",
    2: "PROBE_MEASURE",
    3: "ALIGN",
    4: "RAMP",
    5: "SETTLE",
    6: "MEASURE_A",
    7: "MEASURE_B",
    8: "DONE",
    9: "FAILED",
}

STATUS_NAME = {
    0: "OK",
    1: "ERR_OP",
    2: "ERR_LENGTH",
    3: "ERR_VAR_ID",
    4: "ERR_READ_ONLY",
    5: "ERR_VALUE",
    6: "ERR_STATE",
    7: "ERR_CONFIG",
    8: "ERR_BANDWIDTH",
    9: "ERR_NOT_SUPPORTED",
}


def can_id(msg_type):
    return (msg_type << 6) | NODE_ID


def usb_frame(msg_type, payload):
    return MAGIC + struct.pack("<HB", can_id(msg_type), len(payload)) + payload


class StreamParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []

        while True:
            pos = self.buf.find(MAGIC)
            if pos < 0:
                if len(self.buf) > 3:
                    del self.buf[:-3]
                break

            if pos:
                del self.buf[:pos]

            if len(self.buf) < 7:
                break

            msg_id, length = struct.unpack_from("<HB", self.buf, 4)
            if msg_id > 0x07FF or length > 64:
                del self.buf[0]
                continue

            frame_len = 7 + length
            if len(self.buf) < frame_len:
                break

            payload = bytes(self.buf[7:frame_len])
            del self.buf[:frame_len]
            frames.append((msg_id, payload))

        return frames


class PhaseStats:
    def __init__(self):
        self.count = 0
        self.sum = [0.0, 0.0, 0.0]
        self.abs_max = [0.0, 0.0, 0.0]

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            self.sum[index] += value
            self.abs_max[index] = max(self.abs_max[index], abs(value))

    def means(self):
        if self.count == 0:
            return [0.0, 0.0, 0.0]
        return [value / self.count for value in self.sum]


class RLTest:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = StreamParser()
        self.txn = 1

        self.plot_started = False
        self.capture_zero = False
        self.ident_active = False
        self.tripped = False

        self.fast_frames = 0
        self.fast_samples = 0
        self.fast_lost = 0
        self.fast_last = None
        self.zero = PhaseStats()
        self.phase_peak = 0.0
        self.vbus = []
        self.stage_log = []

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
            base = sample * channel_count
            values = [raw[base + index] * 0.001
                      for index in range(channel_count)]

            if self.capture_zero:
                self.zero.add(values)

            if self.ident_active:
                phase_abs = max(abs(value) for value in values)
                self.phase_peak = max(self.phase_peak, phase_abs)
                if phase_abs > self.args.phase_limit:
                    self.tripped = True

    def process_normal(self, payload):
        if (len(payload) != 8 or payload[2] != NORMAL_CONFIG_ID or
                payload[3] != 1):
            return
        value, = struct.unpack_from("<f", payload, 4)
        self.vbus.append(value)

    def process(self, frames):
        responses = []
        for msg_id, payload in frames:
            msg_type = (msg_id >> 6) & 0x1F
            if msg_type == MSG_FAST_DATA:
                self.process_fast(payload)
            elif msg_type == MSG_NORMAL_DATA:
                self.process_normal(payload)
            elif msg_type == MSG_RESPONSE:
                responses.append(payload)
        return responses

    def pump(self):
        self.process(self.parser.feed(self.ser.read(4096)))

    def request(self, msg_type, op, data=b""):
        txn = self.txn
        payload = bytes([txn, op]) + data
        self.ser.write(usb_frame(msg_type, payload))
        self.ser.flush()

        deadline = time.monotonic() + self.args.timeout
        while time.monotonic() < deadline:
            rx = self.ser.read(4096)
            responses = self.process(self.parser.feed(rx))
            if self.ident_active and self.tripped:
                raise RuntimeError(
                    f"phase current exceeded {self.args.phase_limit:.3f} A"
                )

            for response in responses:
                if len(response) < 4:
                    continue
                rx_txn, req_msg, req_op, status = response[:4]
                if rx_txn != txn or req_msg != msg_type or req_op != op:
                    continue
                if status != 0:
                    name = STATUS_NAME.get(status, str(status))
                    raise RuntimeError(f"request {msg_type}/{op}: {name}")
                self.txn = (txn % 255) + 1
                return response[4:]

        raise TimeoutError(f"request {msg_type}/{op} timeout")

    def disable(self):
        self.ident_active = False
        self.request(MSG_CONTROL, CTRL_DISABLE)

    def configure_plot(self):
        fast_data = bytes([FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id)
                              for _, var_id in FAST_VARS)
        self.request(MSG_PLOT, PLOT_CONFIG, fast_data)

        normal_data = bytes([NORMAL_GROUP, NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", VBUS_ID)
        self.request(MSG_PLOT, PLOT_CONFIG, normal_data)
        self.request(MSG_PLOT, PLOT_START,
                     bytes([FAST_MASK | NORMAL_MASK]))
        self.plot_started = True

    def check_disabled_signals(self):
        self.zero = PhaseStats()
        self.vbus.clear()
        self.capture_zero = True

        deadline = time.monotonic() + self.args.baseline_seconds
        while time.monotonic() < deadline:
            self.pump()

        self.capture_zero = False

        if not self.vbus:
            raise RuntimeError("no Vbus samples")
        if self.zero.count == 0:
            raise RuntimeError("no phase-current samples")

        vbus_mean = sum(self.vbus) / len(self.vbus)
        means = self.zero.means()
        print(f"Vbus={vbus_mean:.3f} V "
              f"({min(self.vbus):.3f} .. {max(self.vbus):.3f} V)")
        print("Disabled current: " + ", ".join(
            f"{name} mean={means[index]:+.4f} A "
            f"abs_max={self.zero.abs_max[index]:.4f} A"
            for index, (name, _) in enumerate(FAST_VARS)
        ))

        if not self.args.vbus_min <= vbus_mean <= self.args.vbus_max:
            raise RuntimeError(
                f"Vbus {vbus_mean:.3f} V outside "
                f"{self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
            )
        if max(abs(value) for value in means) > self.args.zero_mean_limit:
            raise RuntimeError("disabled phase-current mean exceeds limit")
        if max(self.zero.abs_max) > self.args.zero_peak_limit:
            raise RuntimeError("disabled phase-current peak exceeds limit")

        return vbus_mean

    def ident_status(self):
        data = self.request(MSG_IDENTIFICATION, IDENT_STATUS)
        if len(data) != 12:
            raise RuntimeError(f"invalid Rs/Ls status length: {len(data)}")

        mode, state, stage, valid = data[:4]
        rs, ls = struct.unpack_from("<ff", data, 4)
        if mode != IDENT_RS_LS:
            raise RuntimeError(f"unexpected identification mode: {mode}")

        return {
            "state": state,
            "stage": stage,
            "valid": bool(valid),
            "rs_ohm": rs,
            "ls_h": ls,
        }

    def run_identification(self):
        self.request(MSG_CONTROL, CTRL_MODE_SET, bytes([MODE_IDENT]))
        self.request(MSG_IDENTIFICATION, IDENT_MODE_SET,
                     bytes([IDENT_RS_LS]))
        self.request(MSG_CONTROL, CTRL_ENABLE)

        start = time.monotonic()
        self.ident_active = True
        self.request(MSG_CONTROL, CTRL_RUN)
        print("t=0.000 s stage=START")

        deadline = start + self.args.ident_timeout
        next_status = start
        last = None
        result = None

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_status:
                result = self.ident_status()
                current = (result["state"], result["stage"])
                if current != last:
                    stage = RS_LS_STAGE.get(result["stage"],
                                            str(result["stage"]))
                    elapsed = now - start
                    print(f"t={elapsed:.3f} s stage={stage}")
                    self.stage_log.append({
                        "time_s": elapsed,
                        "state": result["state"],
                        "stage": stage,
                    })
                    last = current
                next_status = now + self.args.poll_interval
            else:
                self.pump()

            if self.tripped:
                raise RuntimeError(
                    f"phase current exceeded {self.args.phase_limit:.3f} A"
                )
            if result is not None and result["state"] == IDENT_DONE:
                return result
            if result is not None and result["state"] == IDENT_FAILED:
                raise RuntimeError("Rs/Ls identification failed")

        raise TimeoutError("Rs/Ls identification timeout")

    def validate_result(self, result):
        if not result["valid"]:
            raise RuntimeError("Rs/Ls result is not valid")
        if self.fast_lost != 0:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")

        rs_min = self.args.rs_ref * (1.0 - self.args.rs_tolerance)
        rs_max = self.args.rs_ref * (1.0 + self.args.rs_tolerance)
        ls_ref = self.args.ls_ref_uh * 1.0e-6
        ls_min = ls_ref * (1.0 - self.args.ls_tolerance)
        ls_max = ls_ref * (1.0 + self.args.ls_tolerance)

        if not rs_min <= result["rs_ohm"] <= rs_max:
            raise RuntimeError(
                f"Rs {result['rs_ohm']:.6g} ohm outside "
                f"{rs_min:.6g} .. {rs_max:.6g} ohm"
            )
        if not ls_min <= result["ls_h"] <= ls_max:
            raise RuntimeError(
                f"Ls {result['ls_h'] * 1.0e6:.4f} uH outside "
                f"{ls_min * 1.0e6:.4f} .. {ls_max * 1.0e6:.4f} uH"
            )

    def apply(self):
        self.request(MSG_IDENTIFICATION, IDENT_APPLY)

    def stop_plot(self):
        if self.plot_started:
            self.request(MSG_PLOT, PLOT_STOP,
                         bytes([FAST_MASK | NORMAL_MASK]))
            self.plot_started = False


def run_command(command, cwd, env=None):
    print("+ " + " ".join(str(item) for item in command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def repo_info(repo, allow_dirty):
    status = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"],
        cwd=repo, check=True, text=True, capture_output=True
    ).stdout.strip()
    if status and not allow_dirty:
        raise RuntimeError(
            "working tree is not clean; use --allow-dirty only for an "
            "intentional diagnostic build"
        )

    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, check=True,
        text=True, capture_output=True
    ).stdout.strip()
    branch = subprocess.run(
        ["git", "branch", "--show-current"], cwd=repo, check=True,
        text=True, capture_output=True
    ).stdout.strip()

    print(f"Branch: {branch or '(detached)'}")
    print(f"HEAD:   {head}")
    print(f"Tree:   {'dirty' if status else 'clean'}")
    if status:
        print(status)

    return branch, head, status


def cube_bundle_root():
    roots = []
    if os.environ.get("CUBE_BUNDLE_PATH"):
        roots.append(Path(os.environ["CUBE_BUNDLE_PATH"]))

    home = Path.home()
    roots.extend((
        home / "snap" / "code" / "current" / ".local" / "share" /
        "stm32cube" / "bundles",
        home / ".local" / "share" / "stm32cube" / "bundles",
    ))

    for root in roots:
        if root.is_dir():
            return root
    raise RuntimeError("STM32Cube extension bundle directory was not found")


def bundle_tool(root, bundle, executable):
    candidates = sorted(root.glob(f"{bundle}/*/bin/{executable}"),
                        reverse=True)
    if not candidates:
        raise RuntimeError(f"STM32Cube bundle tool was not found: {executable}")
    return candidates[0]


def build_firmware(repo, toolchain):
    if toolchain == "starm":
        root = cube_bundle_root()
        clang = bundle_tool(root, "st-arm-clang", "starm-clang")
        cmake = bundle_tool(root, "cmake", "cmake")
        ninja = bundle_tool(root, "ninja", "ninja")
        env = os.environ.copy()
        env["CUBE_BUNDLE_PATH"] = str(root)
        env["PATH"] = os.pathsep.join((
            str(clang.parent), str(cmake.parent), str(ninja.parent),
            env.get("PATH", ""),
        ))

        print(f"STM32Cube bundles: {root}")
        run_command([str(cmake), "--preset", "Release"], repo, env)
        run_command([str(cmake), "--build", "--preset", "Release"],
                    repo, env)
        return

    build_dir = repo / "build" / "gcc"
    run_command([
        "cmake", "-S", str(repo), "-B", str(build_dir), "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake",
    ], repo)
    run_command(["cmake", "--build", str(build_dir)], repo)


def flash_firmware(repo, elf):
    if not elf.is_file():
        raise RuntimeError(f"firmware not found: {elf}")
    program = f"program {{{elf}}} verify reset exit"
    run_command(["openocd", "-f", str(repo / "openocd.cfg"),
                 "-c", program], repo)


def open_serial(args):
    deadline = time.monotonic() + args.usb_wait
    last_error = None

    while time.monotonic() < deadline:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            return ser
        except (serial.SerialException, OSError) as exc:
            last_error = exc
            time.sleep(0.25)

    raise RuntimeError(f"USB CDC did not become ready: {last_error}")


def log_path(args, repo):
    if args.log:
        return Path(args.log).resolve()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    build_name = "Release" if args.toolchain == "starm" else "gcc"
    return repo / "build" / build_name / f"rl_apply_{stamp}.json"


def write_log(path, record):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")
    print(f"Log: {path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded Rs/Ls identification and conditional Apply"
    )
    parser.add_argument("--port", required=True, help="STM32 USB CDC port")
    parser.add_argument("--run", action="store_true",
                        help="required confirmation to energize the motor")
    parser.add_argument("--apply", action="store_true",
                        help="apply an accepted Rs/Ls result to RAM")
    parser.add_argument("--flash", action="store_true",
                        help="flash the Release ELF with OpenOCD after build")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--allow-dirty", action="store_true")
    parser.add_argument("--toolchain", choices=("starm", "gcc"),
                        default="starm",
                        help="Release toolchain; starm uses STM32Cube bundles")
    parser.add_argument("--elf",
                        help="override the selected toolchain's ELF path")
    parser.add_argument("--rs-ref", type=float, default=0.0767,
                        help="expected phase resistance in ohm")
    parser.add_argument("--ls-ref-uh", type=float, default=16.5,
                        help="expected phase inductance in uH")
    parser.add_argument("--rs-tolerance", type=float, default=0.20)
    parser.add_argument("--ls-tolerance", type=float, default=0.20)
    parser.add_argument("--phase-limit", type=float, default=1.9)
    parser.add_argument("--zero-mean-limit", type=float, default=0.20)
    parser.add_argument("--zero-peak-limit", type=float, default=0.50)
    parser.add_argument("--vbus-min", type=float, default=10.0)
    parser.add_argument("--vbus-max", type=float, default=20.0)
    parser.add_argument("--baseline-seconds", type=float, default=0.3)
    parser.add_argument("--ident-timeout", type=float, default=5.0)
    parser.add_argument("--poll-interval", type=float, default=0.02)
    parser.add_argument("--usb-wait", type=float, default=10.0)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--log")
    args = parser.parse_args()

    if not args.run:
        parser.error("--run is required to energize the motor")
    for name in ("rs_ref", "ls_ref_uh", "phase_limit",
                 "zero_mean_limit", "zero_peak_limit", "vbus_min",
                 "vbus_max", "baseline_seconds", "ident_timeout",
                 "poll_interval", "usb_wait", "timeout"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    for name in ("rs_tolerance", "ls_tolerance"):
        value = getattr(args, name)
        if not 0.0 < value < 1.0:
            parser.error(f"--{name.replace('_', '-')} must be in (0, 1)")
    if args.vbus_min >= args.vbus_max:
        parser.error("--vbus-min must be less than --vbus-max")

    return args


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    branch = ""
    head = ""
    status = ""
    result = None
    applied = False
    vbus_mean = None
    test = None
    error = None
    started = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    try:
        branch, head, status = repo_info(repo, args.allow_dirty)
        if not args.skip_build:
            build_firmware(repo, args.toolchain)

        build_name = "Release" if args.toolchain == "starm" else "gcc"
        elf = (Path(args.elf).resolve() if args.elf else
               repo / "build" / build_name / "AxDr_L_Motor.elf")
        print(f"Firmware: {elf}")
        if args.flash:
            flash_firmware(repo, elf)

        with open_serial(args) as ser:
            test = RLTest(ser, args)
            try:
                test.disable()
                print("DISABLE OK")
                test.configure_plot()
                vbus_mean = test.check_disabled_signals()
                result = test.run_identification()
                test.disable()
                print("DISABLE after identification OK")

                test.validate_result(result)
                print(f"Rs={result['rs_ohm']:.7g} ohm "
                      f"Ls={result['ls_h'] * 1.0e6:.5g} uH "
                      f"phase_peak={test.phase_peak:.3f} A")

                if args.apply:
                    test.apply()
                    applied = True
                    print("IDENT_APPLY OK")
                else:
                    print("Result accepted; Apply skipped")
            finally:
                if test is not None:
                    try:
                        test.disable()
                    except (TimeoutError, RuntimeError) as exc:
                        print(f"DISABLE warning: {exc}", file=sys.stderr)
                    try:
                        test.stop_plot()
                    except (TimeoutError, RuntimeError) as exc:
                        print(f"PLOT_STOP warning: {exc}", file=sys.stderr)

    except (subprocess.CalledProcessError, serial.SerialException, OSError,
            TimeoutError, RuntimeError) as exc:
        error = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr)

    record = {
        "started": started,
        "branch": branch,
        "head": head,
        "dirty": bool(status),
        "toolchain": args.toolchain,
        "flashed": args.flash,
        "port": args.port,
        "vbus_mean_v": vbus_mean,
        "fast_frames": test.fast_frames if test else 0,
        "fast_samples": test.fast_samples if test else 0,
        "fast_lost": test.fast_lost if test else 0,
        "phase_peak_a": test.phase_peak if test else 0.0,
        "stage_log": test.stage_log if test else [],
        "result": result,
        "applied": applied,
        "error": error,
    }
    write_log(log_path(args, repo), record)

    if error is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
