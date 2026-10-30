#!/usr/bin/env python3
"""Automatic sensored current/speed/position tests using USB Parameter and Plot.

python3 tools/servo_test.py --port /dev/ttyACM0 --run
Add --phase-search to find electrical phase first (firmware saves calibration).
No SWD connection is used. Existing motor, controller and motion limits are kept.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import statistics
import struct
import time

import numpy as np

import motor_usb as m


CHANNELS = (
    ("iq_ref_A", m.PARAM_REF_IQ), ("iq_A", m.PARAM_RUN_IQ),
    ("id_A", m.PARAM_RUN_ID), ("wm_ref_rad_s", m.PARAM_REF_WM),
    ("wm_rad_s", m.PARAM_RUN_WM), ("pos_ref_turn", m.PARAM_REF_POSITION),
    ("pos_turn", m.PARAM_RUN_POSITION), ("vbus_V", m.PARAM_ADC_VBUS),
)
CONFIG_ID = 73
RESTORE = {
    "PARAM_MOTOR_MODE": m.PARAM_U8, "PARAM_MOTION_PROFILE": m.PARAM_U8,
    "PARAM_TARGET_TORQUE": m.PARAM_FLOAT, "PARAM_TARGET_SPEED": m.PARAM_FLOAT,
    "PARAM_TARGET_POSITION": m.PARAM_POSITION,
}
CONDITIONS = {
    "PARAM_ENCODER_PROTOCOL": m.PARAM_U8, "PARAM_ENCODER_ABZ_PPR": m.PARAM_U32,
    "PARAM_MOTOR_PP": m.PARAM_U8, "PARAM_MOTOR_DIR": m.PARAM_I8,
    "PARAM_CAL_VALID": m.PARAM_U8, "PARAM_CAL_ENC_DIR": m.PARAM_I8,
    "PARAM_CTRL_CURRENT_CONTROLLER": m.PARAM_U8,
    "PARAM_CTRL_SPEED_CONTROLLER": m.PARAM_U8,
    **{name: m.PARAM_FLOAT for name in (
        "PARAM_CAL_THETA_OFF", "PARAM_MOTOR_FLUX", "PARAM_MOTOR_RS",
        "PARAM_MOTOR_LD", "PARAM_MOTOR_LQ", "PARAM_MOTOR_J",
        "PARAM_LIMIT_I_MAX", "PARAM_LIMIT_WM_MAX", "PARAM_MOTION_WM_ACC",
        "PARAM_MOTION_WM_DEC", "PARAM_MOTION_TE_RATE", "PARAM_CTRL_CURRENT_BW_HZ",
        "PARAM_CTRL_SPEED_KP_EFFECTIVE", "PARAM_CTRL_SPEED_KI_EFFECTIVE",
        "PARAM_CTRL_POSITION_KP", "PARAM_ENCODER_HARMONIC_C1",
        "PARAM_ENCODER_HARMONIC_S1", "PARAM_ENCODER_HARMONIC_C2",
        "PARAM_ENCODER_HARMONIC_S2",
    )},
}


def position_pair(rad):
    turn = math.floor(rad / math.tau)
    theta = struct.unpack("<f", struct.pack("<f", rad - turn * math.tau))[0]
    # A float32 remainder can round up at the turn boundary.
    if theta >= math.tau:
        turn, theta = turn + 1, 0.0
    return turn, theta


def position_rad(pair):
    return pair[0] * math.tau + pair[1]


def rms(values):
    return math.sqrt(statistics.fmean(x * x for x in values))


class ServoTest(m.MotorUSB):
    def __init__(self, ser, args, writer):
        super().__init__(ser, args)
        self.writer = writer
        self.stage = "setup"
        self.rows = []
        self.seq = None
        self.tick = 0
        self.lost = 0
        self.malformed = 0
        self.rate = 0
        self.t0 = time.monotonic()

    def process_fast(self, payload):
        pass

    def process_normal(self, payload):
        if len(payload) < 4:
            self.malformed += 1
            raise RuntimeError("short NORMAL frame")
        seq, config, count = struct.unpack_from("<HBB", payload)
        if config != CONFIG_ID or not self.rate:
            return
        data = m.canfd_payload(payload, 4 + 4 * len(CHANNELS))
        if count != len(CHANNELS) or data is None:
            self.malformed += 1
            raise RuntimeError("invalid NORMAL channel count/padding")
        values = struct.unpack_from("<8f", data, 4)
        if not all(math.isfinite(x) for x in values):
            self.malformed += 1
            raise RuntimeError("nonfinite motor telemetry")
        step = 1 if self.seq is None else (seq - self.seq) & 0xFFFF
        if not 0 < step < 0x8000:
            self.malformed += 1
            raise RuntimeError("duplicate or reordered NORMAL frame")
        self.lost += step - 1
        self.tick += step
        self.seq = seq
        # Normal_Seq counts transmitted frames, not dropped firmware samples.
        # nominal_s is valid sample time only when firmware_drop == 0.
        row = (self.tick / self.rate, time.monotonic() - self.t0, *values)
        self.rows.append(row)
        self.writer.writerow((self.stage, seq, *row))

    def check(self, expected=None):
        error = self.parameter_read(m.PARAM_EVENT_ERROR, m.PARAM_U32)
        trip = self.parameter_read(m.PARAM_EVENT_TRIP, m.PARAM_U32)
        fault = self.parameter_read(m.PARAM_ENCODER_FAULT, m.PARAM_U8)
        valid = self.parameter_read(m.PARAM_ENCODER_VALID, m.PARAM_U8)
        state = self.parameter_read(m.PARAM_MOTOR_STATE, m.PARAM_U8)
        if error or trip or fault or not valid:
            raise RuntimeError(f"motor fault: error={error:#x}, trip={trip:#x}, "
                               f"encoder_fault={fault}, valid={valid}")
        if expected is not None and state != expected:
            raise RuntimeError(f"motor state {state}, expected {expected}")

    def dwell(self, seconds, expected=2):
        deadline = time.monotonic() + seconds
        poll = time.monotonic()
        while time.monotonic() < deadline:
            self.process(self.parser.feed(self.ser.read(4096)))
            if time.monotonic() >= poll:
                self.check(expected)
                poll = time.monotonic() + 0.1

    def mode(self, mode, profile=0):
        self.parameter_action(m.ACTION_MOTOR_DISABLE)
        self.parameter_write(m.PARAM_MOTOR_MODE, m.PARAM_U8, mode)
        self.parameter_write(m.PARAM_MOTION_PROFILE, m.PARAM_U8, profile)
        self.parameter_write(m.PARAM_TARGET_TORQUE, m.PARAM_FLOAT, 0.0)
        self.speed_set(0.0)
        pos = self.parameter_read(m.PARAM_RUN_POSITION, m.PARAM_POSITION)
        self.parameter_write(m.PARAM_TARGET_POSITION, m.PARAM_POSITION, pos)
        self.parameter_action(m.ACTION_MOTOR_ENABLE)
        self.parameter_action(m.ACTION_MOTOR_RUN)
        self.dwell(0.2)

    def segment(self, name):
        self.stage = name
        self.rows = []
        print(name, flush=True)

    def metrics(self):
        if len(self.rows) < max(10, self.rate * 0.05):
            raise RuntimeError(f"insufficient telemetry in {self.stage}")
        tail = self.rows[-max(10, int(self.rate * 0.15)):]
        return {
            "stage": self.stage, "samples": len(self.rows),
            "iq_error_rms_A": rms([r[3] - r[2] for r in tail]),
            "iq_error_mean_A": statistics.fmean(r[3] - r[2] for r in tail),
            "iq_std_A": statistics.pstdev(r[3] for r in tail),
            "iq_mean_A": statistics.fmean(r[3] for r in tail),
            "iq_ref_mean_A": statistics.fmean(r[2] for r in tail),
            "id_rms_A": rms([r[4] for r in tail]),
            "wm_mean_rad_s": statistics.fmean(r[6] for r in tail),
            "wm_error_rms_rad_s": rms([r[6] - r[5] for r in tail]),
            "iq_peak_A": max(abs(r[3]) for r in self.rows),
        }

    def move(self, target, speed, timeout):
        self.speed_set(speed)
        self.parameter_write(m.PARAM_TARGET_POSITION, m.PARAM_POSITION,
                             position_pair(target))
        deadline = time.monotonic() + timeout
        settled = None
        while time.monotonic() < deadline:
            self.dwell(0.05)
            actual = position_rad(self.parameter_read(m.PARAM_RUN_POSITION, m.PARAM_POSITION))
            ref = position_rad(self.parameter_read(m.PARAM_REF_POSITION, m.PARAM_POSITION))
            wm = self.parameter_read(m.PARAM_RUN_WM, m.PARAM_FLOAT)
            if (abs(ref - target) < 0.001 and
                    abs(actual - target) <= self.args.position_tol and abs(wm) < 0.5):
                if settled is None:
                    settled = time.monotonic()
                if time.monotonic() - settled >= 0.25:
                    result = self.metrics()
                    result.update(target_rad=target, endpoint_error_rad=actual - target)
                    result["position_tracking_peak_rad"] = max(
                        abs(r[8] - r[7]) * math.tau for r in self.rows)
                    return result
            else:
                settled = None
        raise TimeoutError(f"{self.stage}: target={target:.6f}, actual={actual:.6f}, "
                           f"ref={ref:.6f} rad, Wm={wm:.6f} rad/s; did not settle")


def run_suite(c, args, report):
    p = report["conditions"]
    acc, dec = p["PARAM_MOTION_WM_ACC"], p["PARAM_MOTION_WM_DEC"]
    if min(acc, dec) <= 0:
        raise RuntimeError("suite requires positive configured acceleration/deceleration")
    speed_max = max(args.speed_rpm) * math.pi / 30
    if max(speed_max, args.position_speed) > p["PARAM_LIMIT_WM_MAX"]:
        raise RuntimeError("requested test speed exceeds configured limit")
    if args.iq >= p["PARAM_LIMIT_I_MAX"]:
        raise RuntimeError("requested torque test current exceeds configured limit")
    kt = 1.5 * p["PARAM_MOTOR_PP"] * p["PARAM_MOTOR_FLUX"]
    if kt <= 0:
        raise RuntimeError("invalid motor torque constant")

    def record(result, passed=True):
        result["pass"] = passed
        report["results"].append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)

    c.mode(0)
    ramp = p["PARAM_MOTION_TE_RATE"]
    duration = 0.35 + (2 * kt * args.iq / ramp if ramp > 0 else 0)
    for iq in (args.iq, -args.iq, 0):
        c.segment(f"current/{iq:+g}A")
        c.parameter_write(m.PARAM_TARGET_TORQUE, m.PARAM_FLOAT,
                          iq * kt * p["PARAM_MOTOR_DIR"])
        c.dwell(duration)
        result = c.metrics()
        result["target_iq_A"] = iq
        # Small unloaded motors operate near the sampling noise floor. Keep
        # current statistics separate from motion acceptance; no RMS cutoff.
        result["assessment"] = "measurement_only"
        record(result, passed=None)

    for profile in args.profiles:
        c.mode(1, profile)
        factor = 1.875 if profile == 2 else 1.0
        for rpm in args.speed_rpm:
            for target_rpm in (rpm, 0, -rpm, 0):
                target = target_rpm * math.pi / 30
                c.segment(f"profile{profile}/speed/{target_rpm:+g}rpm")
                c.speed_set(target)
                c.dwell(factor * (rpm * math.pi / 30) / min(acc, dec) + args.hold)
                result = c.metrics()
                result["target_rpm"] = target_rpm
                result["wm_target_error_rad_s"] = result["wm_mean_rad_s"] - target
                # In SPEED, Ref.Wm is the planner output. POSITION adds correction.
                slopes = [(b[5] - a[5]) / (b[0] - a[0])
                          for a, b in zip(c.rows, c.rows[1:])
                          if 0 < b[0] - a[0] < 1.5 / c.rate]
                result["nominal_ref_acc_peak_rad_s2"] = max(map(abs, slopes), default=0)
                tol = max(args.speed_tol, abs(target) * 0.05)
                record(result, abs(result["wm_target_error_rad_s"]) <= tol and
                       result["wm_error_rms_rad_s"] <= tol)

        c.mode(2, profile)
        origin = position_rad(c.parameter_read(m.PARAM_RUN_POSITION, m.PARAM_POSITION))
        span = args.turns * math.tau
        timeout = 5 + 4 * span / args.position_speed + 8 * args.position_speed / min(acc, dec)
        for repeat in range(args.repeats):
            for offset in (span, 0, -span, 0):
                c.segment(f"profile{profile}/position/{repeat + 1}/{offset / math.tau:+g}turn")
                record(c.move(origin + offset, args.position_speed, timeout))

        c.segment(f"profile{profile}/reverse-in-motion")
        c.speed_set(args.position_speed)
        c.parameter_write(m.PARAM_TARGET_POSITION, m.PARAM_POSITION, position_pair(origin + span))
        c.dwell(min(0.15, span / (4 * args.position_speed)))
        record(c.move(origin - span, args.position_speed, timeout))

        c.segment(f"profile{profile}/stop-run")
        c.parameter_write(m.PARAM_TARGET_POSITION, m.PARAM_POSITION, position_pair(origin + span))
        c.dwell(min(0.15, span / (4 * args.position_speed)))
        c.parameter_action(m.ACTION_MOTOR_STOP)
        end = time.monotonic() + timeout
        while c.parameter_read(m.PARAM_MOTOR_STATE, m.PARAM_U8) != 1:
            if time.monotonic() > end:
                raise TimeoutError("STOP did not reach ENABLED")
            c.dwell(0.05, expected=None)
        c.parameter_write(m.PARAM_TARGET_POSITION, m.PARAM_POSITION, position_pair(origin))
        c.parameter_action(m.ACTION_MOTOR_RUN)
        record(c.move(origin, args.position_speed, timeout))


def save_waveform(out, report):
    """Pack the streamed samples into the repository's NMIXX waveform format."""
    path = out / "samples.csv"
    if not path.exists():
        return
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return
    known = "firmware_drop_after" in report
    invalid = (report.get("firmware_drop", 0) or report.get("transport_lost", 0)
               or report.get("malformed_frames", 0))
    quality = "invalid" if invalid else "valid" if known else "unknown"
    reason = "sample gaps" if invalid else "" if known else "capture interrupted"
    filename = out / f"Servo-{time.strftime('%Y-%m-%d')}.waveform.nmixx"
    with filename.open("xb") as f:
        np.savez_compressed(
            f, kind=np.array(["waveform"]), group=np.array(["normal"]),
            sample_rate_hz=np.array([report["normal_rate_hz"]], dtype=np.float64),
            raw_samples=np.asarray([[float(r[n]) for n, _ in CHANNELS] for r in rows], dtype="<f4"),
            channel_id=np.array([pid for _, pid in CHANNELS], dtype=np.uint16),
            channel_name=np.array([n for n, _ in CHANNELS]),
            channel_unit=np.array(["A", "A", "A", "rad/s", "rad/s", "turn", "turn", "V"]),
            channel_scale=np.ones(len(CHANNELS)),
            frame_seq=np.array([int(r["seq"]) for r in rows], dtype=np.uint16),
            frame_sample_count=np.ones(len(rows), dtype=np.uint8),
            usb_seq_lost=np.array([report.get("transport_lost", 0)], dtype=np.int64),
            malformed_frames=np.array([report.get("malformed_frames", 0)], dtype=np.int64),
            firmware_drop_before=np.array([report.get("firmware_drop_before", -1)], dtype=np.int64),
            firmware_drop_after=np.array([report.get("firmware_drop_after", -1)], dtype=np.int64),
            quality=np.array([quality]), quality_reason=np.array([reason]),
            captured_at=np.array([report["captured_at"]]),
            metadata_json=np.array([json.dumps(report, ensure_ascii=False)]),
            stage=np.array([r["stage"] for r in rows]),
            host_s=np.array([float(r["host_s"]) for r in rows]),
        )
    path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--run", action="store_true", help="execute motor movements")
    parser.add_argument("--phase-search", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("test-data") / time.strftime("%Y%m%d_servo/%H%M%S"))
    parser.add_argument("--profiles", type=int, nargs="+", choices=(0, 1, 2), default=[0, 1, 2])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--turns", type=float, default=1.0)
    parser.add_argument("--position-speed", type=float, default=10.0, help="rad/s")
    parser.add_argument("--speed-rpm", type=float, nargs="+", default=[1000, 3000, 6000])
    parser.add_argument("--iq", type=float, default=0.2, help="torque test Iq target, A")
    parser.add_argument("--hold", type=float, default=0.5, help="speed plateau seconds")
    parser.add_argument("--position-tol", type=float, default=0.01, help="endpoint tolerance, rad")
    parser.add_argument("--speed-tol", type=float, default=2.0, help="minimum speed tolerance, rad/s")
    parser.add_argument("--timeout", type=float, default=1.0, help="USB response seconds")
    args = parser.parse_args()
    numbers = [args.repeats, args.turns, args.position_speed, args.iq, args.hold,
               args.position_tol, args.speed_tol, args.timeout, *args.speed_rpm]
    if any(not math.isfinite(x) or x <= 0 for x in numbers) or args.hold < 0.2:
        parser.error("numeric options must be positive and finite; hold must be >= 0.2 s")
    if not args.run:
        print("Plan: existing encoder and tuning; current +/-Iq; each profile: "
              "+speed/0/-speed/0, repeated +/-position returns, in-motion reversal, STOP/RUN.")
        print(vars(args))
        print("Add --run to execute; --phase-search optionally recalibrates first.")
        return 0

    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, interrupt)
    args.out.mkdir(parents=True, exist_ok=False)
    report = {"arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "results": [], "pass": False, "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    try:
        with m.serial.Serial(args.port, 115200, timeout=0.01, write_timeout=1,
                             exclusive=True) as ser, (args.out / "samples.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(("stage", "seq", "nominal_s", "host_s", *(n for n, _ in CHANNELS)))
            c = ServoTest(ser, args, writer)
            initial = {}
            owned = False
            drop0 = None
            try:
                if c.parameter_read(m.PARAM_MOTOR_STATE, m.PARAM_U8) != 0:
                    raise RuntimeError("motor must be DISABLED before starting suite")
                initial = {n: c.parameter_read(getattr(m, n), t) for n, t in RESTORE.items()}
                report["initial"] = initial
                report["conditions"] = {n: c.parameter_read(getattr(m, n), t) for n, t in CONDITIONS.items()}
                c.check(0)
                if report["conditions"]["PARAM_ENCODER_PROTOCOL"] not in (1, 2):
                    raise RuntimeError("select a sensored SPI or ABZ encoder first")
                owned = True
                c.request(m.MSG_PLOT, m.PLOT_STOP, bytes([3]))
                if args.phase_search:
                    c.parameter_write(m.PARAM_MOTOR_MODE, m.PARAM_U8, 6)
                    txn = c.parameter_action(m.ACTION_ENC_PHASE_START)
                    deadline = time.monotonic() + 30
                    while True:
                        c.dwell(0.05, expected=None)
                        status = c.action_complete_status(txn, m.ACTION_ENC_PHASE_START)
                        if status is not None:
                            if status:
                                raise RuntimeError(f"phase search failed: status={status}")
                            break
                        if time.monotonic() > deadline:
                            raise TimeoutError("phase search timeout")
                    c.parameter_action(m.ACTION_MOTOR_DISABLE)
                    report["phase"] = {n: c.parameter_read(getattr(m, n), t) for n, t in
                                       (("PARAM_CAL_ENC_DIR", m.PARAM_I8),
                                        ("PARAM_CAL_THETA_OFF", m.PARAM_FLOAT),
                                        ("PARAM_PHASE_THETA_OFF_ERROR", m.PARAM_FLOAT))}
                if not c.parameter_read(m.PARAM_CAL_VALID, m.PARAM_U8):
                    raise RuntimeError("phase calibration invalid; use --phase-search")
                caps = m.plot_capabilities(c.request)
                c.rate = caps["rates"]["normal"]
                report["normal_rate_hz"] = c.rate
                if (len(CHANNELS) > caps["max_channels"]["normal"] or
                        any(not caps["channels"].get(pid, (0,))[0] & m.NORMAL_MASK for _, pid in CHANNELS)):
                    raise RuntimeError("firmware lacks required NORMAL Plot channels")
                c.request(m.MSG_PLOT, m.PLOT_CONFIG, bytes([m.NORMAL_GROUP, CONFIG_ID, len(CHANNELS)]) +
                          b"".join(struct.pack("<H", pid) for _, pid in CHANNELS))
                drop0 = c.parameter_read(m.PARAM_PLOT_NORMAL_DROP, m.PARAM_U32)
                report["firmware_drop_before"] = drop0
                c.request(m.MSG_PLOT, m.PLOT_START, bytes([m.NORMAL_MASK]))
                run_suite(c, args, report)
                report["pass"] = all(result["pass"] is not False for result in report["results"])
            finally:
                if owned:
                    # Disable first, even if telemetry decoding/requests fail.
                    cleanup = []
                    try:
                        c.parameter_action(m.ACTION_MOTOR_DISABLE)
                        if c.parameter_read(m.PARAM_MOTOR_STATE, m.PARAM_U8) != 0:
                            raise RuntimeError("DISABLE did not reach state 0")
                    except Exception as exc:
                        cleanup.append(f"disable: {exc}")
                    try:
                        c.request(m.MSG_PLOT, m.PLOT_STOP, bytes([3]))
                        if drop0 is not None:
                            drop1 = c.parameter_read(m.PARAM_PLOT_NORMAL_DROP, m.PARAM_U32)
                            report["firmware_drop_after"] = drop1
                            report["firmware_drop"] = (drop1 - drop0) & 0xFFFFFFFF
                    except Exception as exc:
                        cleanup.append(f"plot stop: {exc}")
                    for name, value in initial.items():
                        try:
                            c.parameter_write(getattr(m, name), RESTORE[name], value)
                        except Exception as exc:
                            cleanup.append(f"restore {name}: {exc}")
                    report["transport_lost"] = c.lost
                    report["malformed_frames"] = c.malformed
                    if report.get("firmware_drop", 0) or c.lost:
                        report["pass"] = False
                        report["sampling_error"] = "gaps detected; nominal acceleration is not qualified"
                    if cleanup:
                        report["pass"] = False
                        report["cleanup_errors"] = cleanup
    except (Exception, KeyboardInterrupt) as exc:
        report["pass"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
    (args.out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    save_waveform(args.out, report)
    print(f"{'COMPLETE' if report['pass'] else 'FAIL'}: {args.out / 'report.json'}")
    if "error" in report:
        print(report["error"])
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
