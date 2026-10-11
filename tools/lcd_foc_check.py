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
}
FAST_FIELDS = (
    "T0", "Snap_Cnt", "TIM_ISR_Cyc", "TIM_ISR_Max", "SPI_RX_Cnt",
    "SPI_1_Cyc", "SPI_1_ISR_Cyc", "SPI_1_Flag", "SPI_1_CNDTR",
    "SPI_2_Cyc", "SPI_2_ISR_Cyc", "SPI_2_Flag", "SPI_2_CNDTR",
    "Enc_Cyc", "ADC_Cyc", "ADC_Run_Cyc", "ADC_Run_Max", "ADC_ISR_Cyc",
    "ADC_ISR_Max", "Fast_Cyc", "Fast_Max", "Enc_Late", "Enc_Miss",
    "Deadline_Miss",
)


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
    args = parser.parse_args()
    if not 1.0 <= args.seconds <= 60.0:
        parser.error("--seconds must be between 1 and 60")
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


if __name__ == "__main__":
    main()
