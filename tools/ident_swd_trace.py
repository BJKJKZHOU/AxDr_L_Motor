#!/usr/bin/env python3
"""Sample identification RAM through SWD without halting the target.

The optional command after ``--`` starts only after a running-target, PWM-off,
and Vbus preflight.  OpenOCD Tcl ``read_memory`` is the sole target access.
Samples are not atomic across fields; use them for trends and state transitions,
not 20 kHz current-loop waveform reconstruction.
"""

import argparse
import csv
from datetime import datetime, timezone
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TIM1_BDTR = 0x40012C44
MOE = 1 << 15
TCL_END = b"\x1a"

# Source is either an unambiguous ELF symbol or an offline GDB address
# expression.  Neither nm nor GDB connects to the MCU.
SIGNALS = {
    "ident_mode": ("nm:Ident_Mode", "u8", "main"),
    "ident_state": ("nm:Ident_State", "u8", "main"),
    "fail_reason": ("nm:Ident_Fail_Reason", "u8", "main"),
    "motor_state": ("nm:Motor_State", "u8", "main"),
    "flux_state": ("gdb:'Identification/Flux.c'::State", "u8", "main"),
    "flux_valid": ("gdb:'Identification/Flux.c'::Result.Valid", "u8", "main"),
    "flux_wb": ("gdb:'Identification/Flux.c'::Result.Flux_Wb", "f32", "main"),
    "if_mode": ("gdb:Flux_IF.State.Mode", "u8", "main"),
    "if_we": ("gdb:Flux_IF.State.We", "f32", "main"),
    "if_theta_e": ("gdb:Flux_IF.State.Theta_e", "f32", "main"),
    "if_iq": ("gdb:Flux_IF.State.Iq", "f32", "main"),
    "we_base": ("gdb:'Identification/Flux.c'::Start_Para.We_Base", "f32", "main"),
    "we_target": ("nm:We_Target", "f32", "main"),
    "we_obs": ("gdb:'Identification/Flux.c'::We_Obs_F", "f32", "main"),
    "pll_we": ("gdb:Ident_PLL.State.We", "f32", "main"),
    "pll_err": ("gdb:Ident_PLL.State.Err", "f32", "main"),
    "pll_theta": ("gdb:Ident_PLL.State.Theta", "f32", "main"),
    "handover_we_err_f": ("gdb:'Identification/Flux.c'::Handover.We_Err_F", "f32", "main"),
    "handover_we_err2_f": ("gdb:'Identification/Flux.c'::Handover.We_Err2_F", "f32", "main"),
    "handover_pll_err2_f": ("gdb:'Identification/Flux.c'::Handover.PLL_Err2_F", "f32", "main"),
    "handover_theta_err_f": ("gdb:'Identification/Flux.c'::Handover.Theta_Err_F", "f32", "main"),
    "handover_theta_err2_f": ("gdb:'Identification/Flux.c'::Handover.Theta_Err2_F", "f32", "main"),
    "handover_speed_valid": ("gdb:'Identification/Flux.c'::Handover.Speed_Valid", "u8", "main"),
    "handover_theta_valid": ("gdb:'Identification/Flux.c'::Handover.Theta_Valid", "u8", "main"),
    "est_flux": ("gdb:Flux_Estimator.State.Flux", "f32", "main"),
    "emf_ratio": ("nm:Emf_Ratio_F", "f32", "main"),
    "emf_valid": ("nm:Emf_Valid", "u8", "main"),
    "obs_active": ("nm:Obs_Active", "u8", "main"),
    "pll_active": ("nm:PLL_Active", "u8", "main"),
    "obs_control": ("nm:Obs_Control", "u8", "main"),
    "work_reached": ("nm:Work_Point_Reached", "u8", "main"),
    "motion_armed": ("nm:Motion_Lost_Armed", "u8", "main"),
    "motion_lost_cnt": ("nm:Motion_Lost_Cnt", "u32", "main"),
    "handover_ready_cnt": ("nm:Handover_Ready_Cnt", "u32", "main"),
    "blend_cnt": ("gdb:'Identification/Flux.c'::Handover.Blend_Cnt", "u32", "main"),
    "vbus_v": ("gdb:ADC.Vbus_V", "f32", "main"),
    "ia_a": ("gdb:ADC.Ia_A", "f32", "main"),
    "ib_a": ("gdb:ADC.Ib_A", "f32", "main"),
    "ic_a": ("gdb:ADC.Ic_A", "f32", "main"),
    "id_a": ("gdb:Motor_Run.Id", "f32", "main"),
    "iq_a": ("gdb:Motor_Run.Iq", "f32", "main"),
    "ud_v": ("gdb:Motor_Run.Ud", "f32", "main"),
    "uq_v": ("gdb:Motor_Run.Uq", "f32", "main"),
    "resp_rd": ("nm:Resp_Rd", "u8", "aux"),
    "resp_wr": ("nm:Resp_Wr", "u8", "aux"),
    "plot_fast_run": ("gdb:Plot_Group[0].Run", "u8", "aux"),
    "plot_normal_run": ("gdb:Plot_Group[1].Run", "u8", "aux"),
    "plot_fast_drop": ("nm:Plot_Fast_Drop", "u32", "aux"),
    "plot_normal_drop": ("nm:Plot_Normal_Drop", "u32", "aux"),
    "usb_tx_runs": ("gdb:USB_Tx_Thread_Obj.tx_thread_run_count", "u32", "aux"),
    "usb_tx_state": ("gdb:USB_Tx_Thread_Obj.tx_thread_state", "u32", "aux"),
    "usb_rx_runs": ("gdb:ux_device_app_thread.tx_thread_run_count", "u32", "aux"),
    "usb_rx_state": ("gdb:ux_device_app_thread.tx_thread_state", "u32", "aux"),
    "cdc_ptr": ("nm:Cdc_Acm", "u32", "aux"),
    "ident_action_pending": ("nm:Ident_Action_Pending", "u8", "aux"),
}


def symbol_addresses(elf):
    output = subprocess.check_output(["nm", "-an", str(elf)], text=True)
    symbols = {}
    for line in output.splitlines():
        match = re.fullmatch(r"([0-9a-fA-F]+)\s+\w\s+(\S+)", line)
        if match:
            symbols.setdefault(match.group(2), []).append(int(match.group(1), 16))

    addresses = {}
    expressions = {}
    for name, (source, _, _) in SIGNALS.items():
        kind, value = source.split(":", 1)
        if kind == "nm":
            matches = symbols.get(value, [])
            if len(matches) != 1:
                raise RuntimeError(f"ELF symbol {value!r} has {len(matches)} matches")
            addresses[name] = matches[0]
        else:
            expressions[name] = value

    if expressions:
        command = ["arm-none-eabi-gdb", "-q", "-batch", str(elf)]
        for name, expression in expressions.items():
            command += [
                "-ex",
                f'printf "SWD_ADDR {name} 0x%lx\\n", (unsigned long)&{expression}',
            ]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        for name, raw in re.findall(r"SWD_ADDR (\w+) 0x([0-9a-fA-F]+)", result.stdout):
            addresses[name] = int(raw, 16)
        missing = expressions.keys() - addresses.keys()
        if missing:
            raise RuntimeError(f"GDB could not locate {sorted(missing)}: {result.stderr}")

    if any(not 0x20000000 <= addr < 0x20020000 for addr in addresses.values()):
        raise RuntimeError("one or more resolved signal addresses are outside STM32 RAM")
    return addresses


class OpenOcdTcl:
    def __init__(self, host="127.0.0.1", port=6666):
        self.sock = socket.create_connection((host, port), timeout=1.0)
        self.sock.settimeout(3.0)
        self.pending = b""

    def close(self):
        self.sock.close()

    def command(self, command):
        self.sock.sendall(command.encode("ascii") + TCL_END)
        while TCL_END not in self.pending:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("OpenOCD Tcl connection closed")
            self.pending += chunk
        result, self.pending = self.pending.split(TCL_END, 1)
        return result.decode("ascii", errors="replace").strip()

    def ensure_running(self):
        response = self.command("targets")
        if not re.search(r"stm32g4x\.cpu\s+running\b", response):
            raise RuntimeError(f"target is not running; refusing to sample: {response}")

    def words(self, address, count):
        response = self.command(f"read_memory 0x{address:08x} 32 {count}")
        values = response.split()
        if len(values) != count:
            raise RuntimeError(f"SWD read 0x{address:08x}: {response}")
        return [int(value, 0) for value in values]


def regions(names, addresses, max_gap=128, max_bytes=1024):
    words = sorted({addresses[name] & ~3 for name in names})
    result = []
    for address in words:
        if result and address - result[-1][1] <= max_gap and address - result[-1][0] + 4 <= max_bytes:
            result[-1] = (result[-1][0], address)
        else:
            result.append((address, address))
    return [(first, (last - first) // 4 + 1) for first, last in result]


def sample(tcl, names, addresses, spans):
    memory = {}
    for first, count in spans:
        for index, value in enumerate(tcl.words(first, count)):
            memory[first + index * 4] = value

    result = {}
    for name in names:
        address = addresses[name]
        raw = memory[address & ~3]
        kind = SIGNALS[name][1]
        if kind == "u8":
            result[name] = (raw >> (8 * (address & 3))) & 0xFF
        elif kind == "u32":
            result[name] = raw
        else:
            result[name] = struct.unpack("<f", struct.pack("<I", raw))[0]
    return result


def openocd_connect(config, output):
    try:
        return OpenOcdTcl(), None, None
    except OSError:
        pass

    log_path = output.with_suffix(".openocd.log")
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["openocd", "-f", str(config)],
        cwd=ROOT,
        stdout=log_file,
        stderr=subprocess.STDOUT,
    )
    for _ in range(50):
        if process.poll() is not None:
            break
        try:
            return OpenOcdTcl(), process, log_file
        except OSError:
            time.sleep(0.1)
    process.terminate()
    process.wait(timeout=3)
    log_file.close()
    raise RuntimeError(f"OpenOCD did not start; see {log_path}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Release/AxDr_L_Motor.elf")
    parser.add_argument("--openocd-config", type=Path, default=ROOT / "openocd.cfg")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rate", type=float, default=50.0, help="SWD samples/s, maximum 100")
    parser.add_argument("--aux-every", type=int, default=5, help="USB/Plot fields every N samples")
    parser.add_argument("--duration", type=float, help="maximum seconds; default 120 or 900 with command")
    parser.add_argument("--tail", type=float, default=2.0, help="seconds after command exits")
    parser.add_argument("--min-vbus", type=float, default=8.0, help="command preflight minimum Vbus")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="optional command after --")
    args = parser.parse_args()
    if args.command and args.command[0] == "--":
        args.command.pop(0)
    if not 0.0 < args.rate <= 100.0 or args.aux_every < 1:
        parser.error("--rate must be in (0, 100], and --aux-every must be positive")
    if args.tail < 0.0 or args.min_vbus < 0.0:
        parser.error("--tail and --min-vbus must be non-negative")
    if args.duration is None:
        args.duration = 900.0 if args.command else 120.0
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = ROOT / "build/Release" / f"ident_swd_trace_{stamp}.csv"
    args.output = args.output.expanduser().resolve()
    return args


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    addresses = symbol_addresses(args.elf)
    main_names = [name for name in SIGNALS if SIGNALS[name][2] == "main"]
    aux_names = [name for name in SIGNALS if SIGNALS[name][2] == "aux"]
    main_spans = regions(main_names, addresses)
    aux_spans = regions(aux_names, addresses)
    tcl = None
    openocd = None
    openocd_log = None
    child = None
    rows = 0
    error = None

    try:
        tcl, openocd, openocd_log = openocd_connect(args.openocd_config, args.output)
        tcl.ensure_running()
        initial = sample(tcl, main_names, addresses, main_spans)
        bdtr = tcl.words(TIM1_BDTR, 1)[0]
        print(
            f"SWD running; Vbus={initial['vbus_v']:.3f} V, "
            f"motor_state={initial['motor_state']}, MOE={bool(bdtr & MOE)}"
        )
        if args.command:
            if initial["motor_state"] != 0 or (bdtr & MOE):
                raise RuntimeError("motor is not DISABLED/PWM-off; refusing to launch command")
            if initial["vbus_v"] < args.min_vbus:
                raise RuntimeError("Vbus below preflight minimum; refusing to launch command")

        columns = ["utc", "time_s", "swd_ms", *SIGNALS, "tim1_moe"]
        with args.output.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=columns)
            writer.writeheader()
            print(f"Trace: {args.output}")
            if args.command:
                print("Launching:", " ".join(args.command), flush=True)
                child = subprocess.Popen(args.command, cwd=ROOT)

            start = time.monotonic()
            next_sample = start
            next_state_check = start + 1.0
            next_report = start + 5.0
            child_done = None
            aux = {name: "" for name in aux_names}
            moe = int(bool(bdtr & MOE))
            period = 1.0 / args.rate

            while True:
                now = time.monotonic()
                if child and child_done is None and child.poll() is not None:
                    child_done = now
                if now - start >= args.duration or (child_done and now - child_done >= args.tail):
                    break
                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.01))
                    continue
                if now >= next_state_check:
                    tcl.ensure_running()
                    next_state_check = now + 1.0

                read_start = time.monotonic()
                main_values = sample(tcl, main_names, addresses, main_spans)
                if rows % args.aux_every == 0:
                    aux = sample(tcl, aux_names, addresses, aux_spans)
                    moe = int(bool(tcl.words(TIM1_BDTR, 1)[0] & MOE))
                read_end = time.monotonic()
                writer.writerow({
                    "utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    "time_s": f"{read_start - start:.6f}",
                    "swd_ms": f"{1000.0 * (read_end - read_start):.3f}",
                    **main_values,
                    **aux,
                    "tim1_moe": moe,
                })
                rows += 1
                if read_end >= next_report:
                    csv_file.flush()
                    print(
                        f"t={read_end - start:.1f}s state={main_values['flux_state']} "
                        f"We_IF={main_values['if_we']:.1f} "
                        f"We_obs={main_values['we_obs']:.1f} "
                        f"EMF={main_values['emf_ratio']:.3f} "
                        f"lost={main_values['motion_lost_cnt']} "
                        f"plot_drop={aux['plot_fast_drop']}/{aux['plot_normal_drop']}",
                        flush=True,
                    )
                    next_report = read_end + 5.0
                next_sample = max(next_sample + period, read_end)

            csv_file.flush()
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        error = exc
        print(f"Trace error: {exc}", file=sys.stderr)
    except KeyboardInterrupt:
        error = KeyboardInterrupt()
        print("Trace interrupted", file=sys.stderr)
        if child and child.poll() is None:
            child.send_signal(2)
    finally:
        if child and child.poll() is None:
            print("Waiting for identification command to finish its own cleanup...", flush=True)
            child.wait()
        if tcl:
            tcl.close()
        if openocd:
            openocd.terminate()
            openocd.wait(timeout=3)
        if openocd_log:
            openocd_log.close()

    print(f"Captured {rows} samples: {args.output}")
    if error is not None:
        return 1
    return child.returncode if child else 0


if __name__ == "__main__":
    raise SystemExit(main())
