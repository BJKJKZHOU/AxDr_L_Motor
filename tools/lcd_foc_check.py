#!/usr/bin/env python3
"""Measure LCD transfers, Panel stack and FOC timing without halting the MCU.

Start OpenOCD separately. Use the exact ELF flashed to the board. This tool
changes only LCD_Test and the existing Fast_Snap diagnostic window; it does
not enable the motor. The previous display mode is restored on exit unless
--keep-mode is requested for visual inspection of the corner pattern.
"""

import argparse
import json
from pathlib import Path
import re
import subprocess
import time

from ident_swd_trace import OpenOcdTcl, ROOT


MODES = {"normal": 0, "pause": 1, "full": 2, "pattern": 3}
EXPRESSIONS = {
    "lcd": "LCD_Time",
    "test": "LCD_Test",
    "fast": "Fast_Time",
    "snap": "Fast_Snap",
    "count": "Fast_Time.Snap_Cnt",
    "ready": "Fast_Snap_Ready",
    "stack": "Panel_Thread.tx_thread_stack_start",
    "end": "Panel_Thread.tx_thread_stack_end",
    "high": "Panel_Thread.tx_thread_stack_highest_ptr",
    "size": "Panel_Thread.tx_thread_stack_size",
    "clock": "SystemCoreClock",
    "hdmatx": "hspi3.hdmatx",
    "hdma": "hdma_spi3_tx",
    "state": "Motor_State",
    "ccr_hist": "Fast_CCR_Hist",
}
CCR_HIST_SIZE = 64
CCR_HIST_TICKS = 43

FAST_FIELDS = (
    "T0", "Snap_Cnt", "TIM_ISR_Cyc", "TIM_ISR_Max", "SPI_RX_Cnt",
    "SPI_1_Cyc", "SPI_1_ISR_Cyc", "SPI_1_Flag", "SPI_1_CNDTR",
    "SPI_2_Cyc", "SPI_2_ISR_Cyc", "SPI_2_Flag", "SPI_2_CNDTR",
    "Enc_Cyc", "ADC_Cyc", "ADC_Run_Cyc", "ADC_Run_Max", "ADC_ISR_Cyc",
    "ADC_ISR_Max", "Fast_Cyc", "Fast_Max", "Enc_Late", "Enc_Miss",
    "Deadline_Miss", "ADC_IRQ_T0", "ADC_PWM_T0",
    "ESO_Cyc", "ESO_Max", "PI_Cyc", "PI_Max", "SVPWM_Cyc",
    "SVPWM_Max", "ADC_CCR_Cyc", "ADC_CCR_Max",
    "ARR_CCR_Tick", "ARR_CCR_Tick_Max", "ARR_CCR_Count",
    "ARR_CCR_Late", "ARR_CCR_Min", "ARR_CCR_Over",
    "ARR_CCR_Sum_Lo", "ARR_CCR_Sum_Hi", "ARR_CCR_Worst_Seq",
    "ARR_CCR_Worst_ADC", "ARR_CCR_Worst_ESO",
    "ARR_CCR_Worst_PI", "ARR_CCR_Worst_SVPWM",
    "ARR_CCR_Win_N", "ARR_CCR_Win_Max", "ARR_CCR_Win_Over",
    "ARR_CCR_Win_Total", "ARR_CCR_Win_Bad", "ARR_CCR_Win_Last",
)


def ccr_percentile_us(hist, percentile, clock, psc):
    total = sum(hist)
    if not total:
        return None
    target = max(1, int((total * percentile + 99.999999) // 100))
    cumulative = 0
    for i, count in enumerate(hist):
        cumulative += count
        if cumulative >= target:
            if i == len(hist) - 1:
                return None  # The last bin is open-ended.
            return (i + 1) * CCR_HIST_TICKS * (psc + 1) * 1e6 / clock
    return None


def addresses(elf):
    command = ["arm-none-eabi-gdb", "-q", "-batch", str(elf)]
    for name, expression in EXPRESSIONS.items():
        command += ["-ex", f'printf "ADDR {name} %lu\\n", '
                    f'(unsigned long)&{expression}']
    result = subprocess.run(command, text=True, capture_output=True, check=True)
    found = {name: int(value) for name, value in
             re.findall(r"ADDR (\w+) (\d+)", result.stdout)}
    if found.keys() != EXPRESSIONS.keys():
        raise RuntimeError("ELF is missing LCD/FOC diagnostic symbols")
    return found


def measure(args):
    addr = addresses(args.elf)
    tcl = OpenOcdTcl()
    previous = None

    def read(name, count=1):
        return tcl.words(addr[name], count)

    def write(name, value):
        response = tcl.command(f"write_memory {addr[name]} 32 {{{value}}}")
        if response:
            raise RuntimeError(response)

    try:
        tcl.ensure_running()
        size, start, end = read("size")[0], read("stack")[0], read("end")[0]
        if not (0 < size <= 8192 and 0x20000000 <= start <= end < 0x20018000):
            raise RuntimeError("Panel stack is not initialized; check boot and ELF")
        previous = read("test")[0]
        write("test", MODES[args.mode])
        time.sleep(0.25)  # Finish any pending frame before opening the window.
        write("count", 0)
        write("ready", 0)
        time.sleep(0.15)  # Existing ISR clears maxima at cycle 2000 (100 ms).
        first = read("lcd", 6)
        begin = time.monotonic()
        state_before = int(tcl.command(f'read_memory {addr["state"]} 8 1'), 0)
        time.sleep(args.seconds)
        last = read("lcd", 6)
        seconds = time.monotonic() - begin
        tcl.ensure_running()
        clock = read("clock")[0]
        live = dict(zip(FAST_FIELDS, read("fast", len(FAST_FIELDS))))
        snap = dict(zip(FAST_FIELDS, read("snap", len(FAST_FIELDS))))
        ccr_hist = read("ccr_hist", CCR_HIST_SIZE)
        stack = tcl.command(f"read_memory {start} 8 {size}").split()
        if len(stack) != size:
            raise RuntimeError("incomplete stack read")
        unused = 0
        for byte in stack:
            if int(byte, 0) != 0xEF:
                break
            unused += 1
        # STM32G474: DMA2 clock bit 1; DMA2 channel1 IRQ 56, priority 8.
        dma_clock = bool(tcl.words(0x40021048, 1)[0] & 2)
        dma_irq = bool(tcl.words(0xE000E104, 1)[0] & (1 << 24))
        irq_prio = tcl.words(0xE000E438, 1)[0] & 0xFF
        dma_request = tcl.words(0x40020820, 1)[0] & 0x7F
        # This project clocks TIM1 and the CPU at the same 170 MHz.
        psc, arr = tcl.words(0x40012C28, 2)
        result = {
            "mode": args.mode, "seconds": seconds, "clock_hz": clock,
            "motor_state_before": state_before,
            "motor_state_after": int(tcl.command(f'read_memory {addr["state"]} 8 1'), 0),
            "frames": (last[0] - first[0]) & 0xFFFFFFFF,
            "fps": ((last[0] - first[0]) & 0xFFFFFFFF) / seconds,
            "bytes_per_s": ((last[1] - first[1]) & 0xFFFFFFFF) / seconds,
            "lcd_before": first, "lcd_after": last,
            "panel_stack_bytes": size, "panel_stack_used": size - unused,
            "panel_stack_high_used": end - read("high")[0] + 1,
            "dma_link_ok": read("hdmatx")[0] == addr["hdma"],
            "dma2_clock": dma_clock, "dma2_irq": dma_irq,
            "dma2_irq_priority": irq_prio >> 4,
            "spi3_tx_request": dma_request,
            "pwm_period_us": 2 * arr * (psc + 1) * 1e6 / clock,
            "snap_ready": read("ready")[0], "snap": snap, "live": live,
            "fast_max_us": live["Fast_Max"] * 1e6 / clock,
            "adc_isr_max_us": live["ADC_ISR_Max"] * 1e6 / clock,
            "eso_max_us": live["ESO_Max"] * 1e6 / clock,
            "pi_max_us": live["PI_Max"] * 1e6 / clock,
            "svpwm_max_us": live["SVPWM_Max"] * 1e6 / clock,
            "adc_ccr_max_us": live["ADC_CCR_Max"] * 1e6 / clock,
            # This board runs TIM1 at HCLK; account for the TIM1 prescaler.
            "arr_ccr_max_us": (live["ARR_CCR_Tick_Max"] * (psc + 1)
                               * 1e6 / clock),
            "arr_ccr_count": live["ARR_CCR_Count"],
            "arr_ccr_late": live["ARR_CCR_Late"],
            "arr_ccr_min_us": (live["ARR_CCR_Min"] * (psc + 1) * 1e6 / clock
                               if live["ARR_CCR_Min"] != 0xFFFFFFFF else None),
            "arr_ccr_mean_us": ((((live["ARR_CCR_Sum_Hi"] << 32)
                                  | live["ARR_CCR_Sum_Lo"]) * (psc + 1)
                                 * 1e6 / clock) / max(
                                     1, live["ARR_CCR_Count"] - live["ARR_CCR_Late"])),
            "arr_ccr_over_12_5us": live["ARR_CCR_Over"],
            "arr_ccr_over_12_5us_pct": (live["ARR_CCR_Over"] * 100
                                        / max(1, live["ARR_CCR_Count"] -
                                              live["ARR_CCR_Late"])),
            "arr_ccr_p50_upper_us": ccr_percentile_us(
                ccr_hist, 50, clock, psc),
            "arr_ccr_p95_upper_us": ccr_percentile_us(
                ccr_hist, 95, clock, psc),
            "arr_ccr_p99_upper_us": ccr_percentile_us(
                ccr_hist, 99, clock, psc),
            "arr_ccr_p999_upper_us": ccr_percentile_us(
                ccr_hist, 99.9, clock, psc),
            "arr_ccr_hist_ticks_per_bin": CCR_HIST_TICKS,
            "arr_ccr_hist_counts": ccr_hist,
            "arr_ccr_hist_samples": sum(ccr_hist),
            "arr_ccr_worst_sample_index": live["ARR_CCR_Worst_Seq"],
            "arr_ccr_worst_sample_stages_us": {
                "adc_irq_to_ccr": live["ARR_CCR_Worst_ADC"] * 1e6 / clock,
                "mechanical_eso": live["ARR_CCR_Worst_ESO"] * 1e6 / clock,
                "current_pi": live["ARR_CCR_Worst_PI"] * 1e6 / clock,
                "svpwm": live["ARR_CCR_Worst_SVPWM"] * 1e6 / clock,
            },
            "arr_ccr_windows_50ms_completed": live["ARR_CCR_Win_Total"],
            "arr_ccr_windows_50ms_over_limit": live["ARR_CCR_Win_Bad"],
            "arr_ccr_last_window_max_us": (live["ARR_CCR_Win_Last"]
                                            * (psc + 1) * 1e6 / clock),
            "arr_ccr_within_12_5us": (live["ARR_CCR_Count"] > 0
                                      and live["ARR_CCR_Late"] == 0
                                      and live["ARR_CCR_Tick_Max"] * (psc + 1)
                                      * 1e6 / clock <= 12.5),
        }
        return result
    finally:
        if previous is not None and not args.keep_mode:
            write("test", previous)
        tcl.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Release/AxDr_L_Motor.elf")
    parser.add_argument("--mode", choices=MODES, default="normal")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--keep-mode", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--arr-ccr-limit-us", type=float,
                        help="require PWM-active ARR->CCR3 timing within this limit")
    args = parser.parse_args()
    if not 1.0 <= args.seconds <= 60.0:
        parser.error("--seconds must be between 1 and 60")
    if args.arr_ccr_limit_us is not None and args.arr_ccr_limit_us <= 0.0:
        parser.error("--arr-ccr-limit-us must be positive")
    result = measure(args)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if not (result["dma_link_ok"] and result["dma2_clock"] and result["dma2_irq"]
            and result["dma2_irq_priority"] == 8 and result["spi3_tx_request"] == 15
            and result["snap_ready"] == 1):
        raise SystemExit("DMA/FOC initialization check failed")
    if any(result["live"][name] for name in ("Deadline_Miss", "Enc_Miss", "Enc_Late")):
        raise SystemExit("FOC timing or encoder errors detected")
    if result["fast_max_us"] >= result["pwm_period_us"]:
        raise SystemExit("FOC maximum exceeds the PWM period")
    if args.mode == "full" and result["frames"] == 0:
        raise SystemExit("no full-frame DMA transfers completed")
    if args.arr_ccr_limit_us is not None:
        if result["arr_ccr_count"] == 0:
            raise SystemExit("No PWM-active CCR samples; enable the motor first")
        if result["arr_ccr_late"]:
            raise SystemExit("CCR write missed the intended ARR descending half")
        if result["arr_ccr_max_us"] > args.arr_ccr_limit_us:
            raise SystemExit("ARR->CCR3 maximum exceeds the requested limit")


if __name__ == "__main__":
    main()
