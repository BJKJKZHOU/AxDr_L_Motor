#!/usr/bin/env python3
"""Read optional MATH_TIME probes during an already-running motor test.

Use the matching ELF and OpenOCD. No motor commands or flash operations are
performed. Probe times include call/timestamp overhead and interrupt preemption;
ESO includes its nested angle probe. This is not an uninstrumented deadline test.
"""

import argparse
import json
from pathlib import Path
import re
import subprocess
import time

import lcd_foc_check as lcd
from ident_swd_trace import OpenOcdTcl


NAMES = ("wrap_e", "wrap_eso", "sincos", "limit_i", "limit_u", "pid_d",
         "pid_q", "eso", "svpwm", "empty")
FIELDS = ("Cyc", "Min", "Max", "Count", "Sum_Lo", "Sum_Hi", "Alt", "Alt_Count")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be between 1 and 60")
    command = ["arm-none-eabi-gdb", "-q", "-batch", str(args.elf)]
    for name in ("Math_Time", "Math_Run"):
        command += ["-ex", f'printf "ADDR {name} %lu\\n", (unsigned long)&{name}']
    output = subprocess.check_output(command, text=True)
    addr = {name: int(value) for name, value in
            re.findall(r"ADDR (\w+) (\d+)", output)}
    if set(addr) != {"Math_Time", "Math_Run"}:
        raise RuntimeError("ELF lacks MATH_TIME diagnostics")
    tcl = OpenOcdTcl()

    def run(value):
        response = tcl.command(f'write_memory {addr["Math_Run"]} 32 {{{value}}}')
        if response:
            raise RuntimeError(response)

    try:
        tcl.ensure_running()
        run(0)
        time.sleep(.01)
        # Cyc/Alt remain ISR-owned scratch; only accumulated fields are cleared.
        for i in range(len(NAMES)):
            base = addr["Math_Time"] + i * len(FIELDS) * 4
            for offset, value in ((4, 0xFFFFFFFF), (8, 0), (12, 0),
                                  (16, 0), (20, 0), (28, 0)):
                response = tcl.command(f"write_memory {base + offset} 32 {{{value}}}")
                if response:
                    raise RuntimeError(response)
        begin = time.monotonic()
        run(1)
        args.mode, args.keep_mode = "normal", False
        foc = lcd.measure(args)
        run(0)
        seconds = time.monotonic() - begin
        time.sleep(.01)  # Let any ISR already accumulating finish before reading.
        raw = tcl.words(addr["Math_Time"], len(NAMES) * len(FIELDS))
        result = {"math_seconds": seconds, "clock_hz": foc["clock_hz"],
                  "note": "Math window includes LCD timing warmup/readout; times are raw inclusive measurements.",
                  "math": {}, "foc": foc}
        for i, name in enumerate(NAMES):
            row = dict(zip(FIELDS, raw[i * len(FIELDS):(i + 1) * len(FIELDS)]))
            count = row["Count"]
            ticks = (row["Sum_Hi"] << 32) | row["Sum_Lo"]
            scale = 1e6 / foc["clock_hz"]
            result["math"][name] = {
                "count": count,
                "min_cycles": row["Min"] if count else None,
                "max_cycles": row["Max"] if count else None,
                "min_us": row["Min"] * scale if count else None,
                "mean_us": ticks * scale / count if count else None,
                "max_us": row["Max"] * scale if count else None,
                "alt_count": row["Alt_Count"],
            }
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "foc"}, indent=2))
        print("FOC", json.dumps({k: foc[k] for k in
              ("fast_max_us", "adc_isr_max_us", "arr_ccr_max_us", "arr_ccr_late")}))
        if any(foc["live"][key] for key in ("Deadline_Miss", "Enc_Miss", "Enc_Late")):
            raise SystemExit("Diagnostic build has FOC timing/encoder errors; inspect saved results")
    finally:
        try:
            run(0)
        finally:
            tcl.close()


if __name__ == "__main__":
    main()
