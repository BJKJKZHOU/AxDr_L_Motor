#!/usr/bin/env python3
"""Run the bounded DUAL_LIMIT_TEST firmware in the position loop.

Requires the exact flashed ELF and OpenOCD. Uses software current guards with
the user-approved nominal 24 V supply. No flash or NVS writes. Position target:
+3 turns, command speed: 1 rad/s; firmware stops PWM on completion or a guard violation.
The added capture/excitation overhead is included in every reported latency.
"""

import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import time

import serial
import identification_test as base
import parameter_ids_generated as p
from encoder_motion_test import EncoderMotionTest
from ident_swd_trace import OpenOcdTcl
import lcd_foc_check as lcd


STAT_FIELDS = "Count I U Both Aw_D Aw_Q Both_FF Late Min Max Over Sum_Lo Sum_Hi".split()
SAMPLE_UINTS = "Seq Phase Flags Tick".split()
SAMPLE_FLOATS = ("Id_In Iq_In Id_Ref Iq_Ref Ud_In Uq_In Ud Uq Id Iq Wm Vbus "
                 "I_Lim U_Lim Id_Int_Pre Iq_Int_Pre Id_Int Iq_Int Ud_PI Uq_PI Ud_FF Uq_FF").split()
SYMBOLS = ("Limit_Run", "Limit_N", "Limit_Excite_N", "Limit_Trace_N", "Limit_Stat", "Limit_Trace",
           "Limit_Trip_Code", "Limit_Trip_Fbk", "Limit_Tail")


def addresses(elf):
    command = ["arm-none-eabi-gdb", "-q", "-batch", str(elf)]
    for name in SYMBOLS:
        command += ["-ex", f'printf "ADDR {name} %lu\\n", (unsigned long)&{name}']
    command += ["-ex", 'printf "SIZE %u %u\\n", sizeof(Limit_Stat[0]), sizeof(Limit_Trace[0])']
    output = subprocess.check_output(command, text=True)
    sizes = re.search(r"SIZE (\d+) (\d+)", output)
    if not sizes or tuple(map(int, sizes.groups())) != (312, 104):
        raise RuntimeError("ELF diagnostic layout mismatch")
    found = {name: int(value) for name, value in re.findall(r"ADDR (\w+) (\d+)", output)}
    if set(found) != set(SYMBOLS):
        raise RuntimeError("ELF lacks DUAL_LIMIT_TEST symbols")
    return found | lcd.addresses(elf)


def verify_samples(samples, model):
    """Check actual captured vector scaling, FF formula and PI state/output.

    This checks recorded samples, not all ISR cycles; the firmware counts every
    cycle independently. float32 rounding is allowed in the reconstruction.
    """
    failures = []
    for s in samples:
        def check(ok, reason):
            if not ok:
                failures.append({"seq": s["Seq"], "reason": reason})

        def close(a, b):
            return math.isclose(a, b, rel_tol=2e-5, abs_tol=2e-6)

        check(all(math.isfinite(s[k]) for k in SAMPLE_FLOATS), "nonfinite sample")
        for prefix, inputs, outputs, limit, bit in (
                ("I", ("Id_In", "Iq_In"), ("Id_Ref", "Iq_Ref"), "I_Lim", 1),
                ("U", ("Ud_In", "Uq_In"), ("Ud", "Uq"), "U_Lim", 2)):
            magnitude = math.hypot(*(s[k] for k in inputs))
            scale = min(1.0, s[limit] / magnitude) if magnitude else 1.0
            check(all(close(s[o], s[i] * scale) for i, o in zip(inputs, outputs)),
                  prefix + " vector output mismatch")
            changed = any(s[i] != s[o] for i, o in zip(inputs, outputs))
            check(bool(s["Flags"] & bit) == changed, prefix + " scaling flag mismatch")
        we = model["pp"] * s["Wm"]
        check(close(s["Ud_FF"], -we * model["lq"] * s["Iq"]), "d feedforward mismatch")
        check(close(s["Uq_FF"], we * (model["ld"] * s["Id"] + model["flux"])),
              "q feedforward mismatch")
        for axis, bit in (("d", 4), ("q", 8)):
            err = s[f"I{axis}_Ref"] - s[f"I{axis}"]
            previous = s[f"I{axis}_Int_Pre"]
            lim = s["U_Lim"]
            candidate = max(-lim, min(lim, previous + model[f"ki_{axis}"] * err * .00005))
            output = model[f"kp_{axis}"] * err + candidate
            rollback = ((output > lim and candidate > previous) or
                        (output < -lim and candidate < previous))
            expected = previous if rollback else candidate
            check(close(s[f"I{axis}_Int"], expected), axis + " PI integral mismatch")
            check(close(s[f"U{axis}_PI"], max(-lim, min(lim, model[f"kp_{axis}"] * err + expected))),
                  axis + " PI output mismatch")
            if s["Flags"] & bit:
                check(s[f"I{axis}_Int"] == previous, axis + " rollback did not hold integral")
    return failures


def capture(tcl, addr, model):
    read = lambda name, count=1: tcl.words(addr[name], count)
    raw = read("Limit_Stat", 3 * 78)
    stats = []
    for i in range(3):
        words = raw[i * 78:(i + 1) * 78]
        peak2 = struct.unpack("<f", struct.pack("<I", words[77]))[0]
        stats.append(dict(zip(STAT_FIELDS, words[:13])) |
                     {"hist_counts": words[13:77], "feedback_peak_a": math.sqrt(peak2)})
    count = read("Limit_Trace_N")[0]
    if count > 256:
        raise RuntimeError("invalid trace length")
    samples = []
    for start in range(0, count, 8):
        words = tcl.words(addr["Limit_Trace"] + start * 104, min(8, count - start) * 26)
        for offset in range(0, len(words), 26):
            row = words[offset:offset + 26]
            floats = struct.unpack("<22f", struct.pack("<22I", *row[4:]))
            samples.append(dict(zip(SAMPLE_UINTS, row[:4])) | dict(zip(SAMPLE_FLOATS, floats)))
    tail = []
    for start in range(0, 16, 8):
        words = tcl.words(addr["Limit_Tail"] + start * 104, 8 * 26)
        for offset in range(0, len(words), 26):
            row = words[offset:offset + 26]
            floats = struct.unpack("<22f", struct.pack("<22I", *row[4:]))
            tail.append(dict(zip(SAMPLE_UINTS, row[:4])) | dict(zip(SAMPLE_FLOATS, floats)))
    tail_count = min(16, read("Limit_N")[0])
    tail = sorted(tail, key=lambda s: s["Seq"])[-tail_count:] if tail_count else []
    trip = struct.unpack("<5f", struct.pack("<5I", *read("Limit_Trip_Fbk", 5)))
    clock = read("clock")[0]
    psc = tcl.words(0x40012C28, 1)[0]
    fast = dict(zip(lcd.FAST_FIELDS, read("fast", len(lcd.FAST_FIELDS))))
    hist = read("ccr_hist", 64)
    violations = verify_samples(samples + tail, model)
    for phase, s in enumerate(stats):
        if sum(s["hist_counts"]) + s["Late"] != s["Both"]:
            violations.append({"phase": phase, "reason": "joint histogram/count mismatch"})
        if not (s["Both"] <= min(s["I"], s["U"]) <= s["Count"]):
            violations.append({"phase": phase, "reason": "scaling counts inconsistent"})
    if sum(hist) + fast["ARR_CCR_Late"] != fast["ARR_CCR_Count"]:
        violations.append({"reason": "whole-loop histogram/count mismatch"})
    scale = (psc + 1) * 1e6 / clock
    return {
        "run": read("Limit_Run")[0], "cycles": read("Limit_N")[0],
        "clock_hz": clock, "tim1_psc": psc, "hist_ticks_per_bin": 43,
        "note": "Reference vector <=0.3 A; feedback shutdown threshold 0.6 A. "
                "Test-only voltage ceiling 0.15 V during excitation; capture overhead included. "
                "PI rollback is reconstructed and checked against the actual integral. "
                "Final histogram bin is open-ended; exact maximum is stored separately.",
        "stats": stats, "samples": samples, "model": model, "fast": fast,
        "tail": tail, "guard_code": read("Limit_Trip_Code")[0],
        "guard_feedback": dict(zip(("Ialpha", "Ibeta", "Wm", "Vbus", "I_Lim"), trip)),
        "arr_ccr_hist_counts": hist, "arr_ccr_max_us": fast["ARR_CCR_Tick_Max"] * scale,
        "arr_ccr_over_12_5us": fast["ARR_CCR_Over"],
        "both_max_us": stats[1]["Max"] * scale if stats[1]["Both"] else None,
        "sample_check_failures": violations,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-STMicroelectronics_STM32_USB_Device_000000000001-if00")
    parser.add_argument("--supply-volts", type=float, required=True)
    parser.add_argument("--seconds", type=float, default=10.0, help="excitation time, 0.1..10 s")
    parser.add_argument("--run", action="store_true", help="authorize the bounded position motion")
    args = parser.parse_args()
    if not args.run or not (20 <= args.supply_volts <= 27 and .1 <= args.seconds <= 10):
        parser.error("requires --run, user-approved 20..27 V supply, and 0.1..10 s excitation")
    excite_n = round(args.seconds * 20000)
    args.timeout = 1.0
    addr = addresses(args.elf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tcl = OpenOcdTcl()

    def write(name, values):
        response = tcl.command(f'write_memory {addr[name]} 32 {{{" ".join(map(str, values))}}}')
        if response:
            raise RuntimeError(response)

    result, saved, previous_lcd = {}, {}, None
    try:
        tcl.ensure_running()
        with serial.Serial(args.port, 115200, timeout=.003, write_timeout=1) as ser:
            client = EncoderMotionTest(ser, args)
            read = client.parameter_read
            if read(p.PARAM_MOTOR_STATE, base.PARAM_U8) != 0:
                raise RuntimeError("motor must initially be DISABLED")
            vbus = read(p.PARAM_ADC_VBUS, base.PARAM_FLOAT)
            if not (20 <= vbus <= 27 and abs(vbus - args.supply_volts) <= 1):
                raise RuntimeError(f"actual Vbus {vbus:.3f} V does not match approved supply")
            client.protection_check()
            client.encoder_check()
            model = {k: read(i, ty) for k, i, ty in (
                ("pp", p.PARAM_MOTOR_PP, base.PARAM_U8),
                ("ld", p.PARAM_MOTOR_LD, base.PARAM_FLOAT),
                ("lq", p.PARAM_MOTOR_LQ, base.PARAM_FLOAT),
                ("flux", p.PARAM_MOTOR_FLUX, base.PARAM_FLOAT),
                ("kp_d", p.PARAM_CTRL_ID_KP, base.PARAM_FLOAT),
                ("ki_d", p.PARAM_CTRL_ID_KI, base.PARAM_FLOAT),
                ("kp_q", p.PARAM_CTRL_IQ_KP, base.PARAM_FLOAT),
                ("ki_q", p.PARAM_CTRL_IQ_KI, base.PARAM_FLOAT))}
            settings = ((p.PARAM_MOTOR_MODE, base.PARAM_U8, 2),
                        (p.PARAM_LIMIT_I_MAX, base.PARAM_FLOAT, 0.3),
                        (p.PARAM_LIMIT_WM_MAX, base.PARAM_FLOAT, 2.0),
                        (p.PARAM_TARGET_SPEED, base.PARAM_FLOAT, 1.0),
                        (p.PARAM_CTRL_CURRENT_FF_ENABLE, base.PARAM_U8, 1))
            for param, ty, _ in settings:
                saved[param] = (ty, read(param, ty))
            saved_position = client.position_read(p.PARAM_TARGET_POSITION)
            origin = client.position_read(p.PARAM_RUN_POSITION)
            result.update(supply_volts=args.supply_volts, hardware_current_limit=False,
                          excitation_seconds=excite_n / 20000,
                          vbus_before=vbus, origin=origin)
            try:
                # Nothing is armed while setting limits or clearing diagnostics.
                write("Limit_Run", [0])
                write("Limit_N", [0])
                write("Limit_Excite_N", [excite_n])
                write("Limit_Trace_N", [0])
                write("Limit_Trip_Code", [0])
                write("Limit_Trip_Fbk", [0] * 5)
                write("Limit_Stat", [0] * (3 * 78))
                for param, ty, value in settings:
                    client.parameter_write(param, ty, value)
                client.position_write(p.PARAM_TARGET_POSITION, *origin)
                previous_lcd = tcl.words(addr["test"], 1)[0]
                write("test", [0])
                write("count", [0])
                write("ready", [0])
                time.sleep(.15)
                client.start()
                write("Limit_Run", [1])
                target = client.scalar_position(client.position_scalar(origin) + 6 * math.pi)
                client.position_write(p.PARAM_TARGET_POSITION, *target)
                deadline = time.monotonic() + args.seconds + 3
                while tcl.words(addr["Limit_Run"], 1)[0] == 1:
                    if time.monotonic() >= deadline:
                        raise TimeoutError("firmware experiment did not finish")
                    client.protection_check()
                    time.sleep(.05)
            except BaseException as exc:
                result["error"] = str(exc)
                raise
            finally:
                # Latch PWM off before serial cleanup; never halt an active MCU.
                try:
                    if tcl.words(addr["Limit_Run"], 1)[0] == 1:
                        write("Limit_Run", [3])
                    time.sleep(.01)
                finally:
                    client.stop_disable()
                if read(p.PARAM_MOTOR_STATE, base.PARAM_U8) != 0:
                    raise RuntimeError("motor did not disable; keep PWM latch set")
                try:
                    result.update(capture(tcl, addr, model))
                    result["position_after"] = client.position_read(p.PARAM_RUN_POSITION)
                    result["error_flags"] = read(p.PARAM_EVENT_ERROR, base.PARAM_U32)
                    result["trip_flags"] = read(p.PARAM_EVENT_TRIP, base.PARAM_U32)
                finally:
                    args.output.write_text(json.dumps(result, indent=2) + "\n")
                    # Restore parameters while DISABLED, never persist settings.
                    for param, (ty, value) in saved.items():
                        client.parameter_write(param, ty, value)
                    client.position_write(p.PARAM_TARGET_POSITION, *saved_position)
                    if previous_lcd is not None:
                        write("test", [previous_lcd])
                    # Keep the completed/aborted PWM latch until reset/reflash.
    finally:
        tcl.close()
    active = result["stats"][1]
    joint_samples = [s for s in result["samples"] if s["Phase"] == 1 and s["Flags"] & 3 == 3]
    observed_ff = any(s["Ud_FF"] != 0 and s["Uq_FF"] != 0 for s in joint_samples)
    observed_rollback = any(s["Flags"] & 12 for s in joint_samples)
    print(json.dumps({k: result[k] for k in ("run", "cycles", "arr_ccr_max_us",
                     "arr_ccr_over_12_5us", "both_max_us", "sample_check_failures")}, indent=2))
    if (result["run"] != 2 or result["cycles"] != excite_n + 20000 or
            [s["Count"] for s in result["stats"]] != [10000, excite_n, 10000] or
            not active["Both"] or not active["Both_FF"] or not joint_samples or
            not observed_ff or not observed_rollback or
            not (active["Aw_D"] or active["Aw_Q"]) or result["sample_check_failures"] or
            result["error_flags"] or result["trip_flags"] or
            any(result["fast"][k] for k in ("ARR_CCR_Late", "Deadline_Miss", "Enc_Late", "Enc_Miss"))):
        raise SystemExit("Coverage/control checks failed; inspect saved capture")
    if result["arr_ccr_max_us"] >= 12.5:
        raise SystemExit("Instrumented whole-loop maximum is not below 12.5 us")


if __name__ == "__main__":
    main()
