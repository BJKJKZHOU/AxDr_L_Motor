#!/usr/bin/env python3
"""Plot AxDr phase-current feedback against time, with a sample-level zoom.

Dependencies: numpy, matplotlib, pyserial (USB), OpenOCD + nm/llvm-nm
(live capture integrity check). The ELF must match the running firmware.

Examples:
  python tools/current_waveform.py --port /dev/ttyACM0 --elf flashed.elf --labels LADRC
  python tools/current_waveform.py --input pi.i16 ladrc.i16 --labels PI LADRC
  python tools/current_waveform.py --input capture.npz --zoom-ms 2

Legacy .i16 captures use their adjacent .json column metadata and Tick counter.
Live acquisition uses the existing FAST Plot protocol and checks the firmware
drop counter through SWD. Motor commands, parameters and firmware are unchanged.
Each FOC cycle measures two phases and reconstructs the remaining phase.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np

from fft import CHANNELS, PlotClient, channel_selection, decode_samples

ROOT = Path(__file__).resolve().parents[1]
PHASES = ("ia", "ib", "ic")
COLORS = ("#2563eb", "#db6b16", "#169b79")


def load_capture(path, fs):
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as z:
            if int(z["format_version"].item()) != 1:
                raise ValueError(f"unsupported waveform archive: {path}")
            return dict(t=z["time_s"], values=z["values"],
                        names=z["channel_names"].tolist(), pair=z["phase_pair"],
                        fs=float(z["sample_rate_hz"].item()),
                        label=str(z["label"].item()),
                        meta=json.loads(str(z["metadata_json"].item())))

    raw_path = path.with_suffix(".i16") if path.suffix == ".json" else path
    meta = json.loads(raw_path.with_suffix(".json").read_text())
    columns = meta["columns"]
    raw = np.fromfile(raw_path, dtype="<i2")
    if not len(raw) or len(raw) % len(columns):
        raise ValueError(f"incomplete sample row: {raw_path}")
    raw = raw.reshape(-1, len(columns)).astype(np.float64)
    index = {col[0]: n for n, col in enumerate(columns)}
    if "Tick" not in index:
        raise ValueError("legacy capture requires a Tick column for its time axis")
    ticks = raw[:, index["Tick"]].astype(np.int64)
    step = np.mod(np.diff(ticks), 65536)
    if np.any(step == 0):
        raise ValueError("duplicate Tick values in capture")
    t = np.r_[0, np.cumsum(step)] / fs
    ids = {info[0]: name for name, info in CHANNELS.items()}
    names, values = [], []
    for n, (_, param_id, scale) in enumerate(columns):
        if param_id in ids:
            names.append(ids[param_id])
            values.append(raw[:, n] * scale)
    pair = raw[:, index["Pair"]].astype(np.int8) if "Pair" in index else np.full(len(t), -1)
    return dict(t=t, values=np.asarray(values).T, names=names, pair=pair,
                fs=fs, label=meta.get("mode", raw_path.stem).upper(), meta=meta)


def capture_usb(args, out):
    # Read only the upload-drop counter, never current values, via SWD.
    # FAST packet sequence numbers alone do not expose discarded firmware blocks.
    from ident_swd_trace import openocd_connect

    nm = shutil.which("arm-none-eabi-nm") or shutil.which("llvm-nm") or shutil.which("nm")
    if nm is None:
        raise RuntimeError("live capture needs arm-none-eabi-nm, llvm-nm or nm")
    symbols = subprocess.check_output([nm, "--defined-only", str(args.elf)], text=True)
    match = re.search(r"^([0-9a-fA-F]+)\s+\w\s+Plot_Fast_Drop$", symbols, re.MULTILINE)
    if not match:
        raise ValueError("ELF has no Plot_Fast_Drop symbol")
    address = int(match[1], 16)
    tcl, proc, log = openocd_connect(args.openocd_config, out / "capture")
    client = None
    try:
        tcl.command("poll")
        tcl.ensure_running()
        client = PlotClient(args.port, args.baud)
        client.ser.timeout = .001
        for attempt in range(args.attempts):
            before = tcl.words(address, 1)[0]
            try:
                raw = client.capture("fast", args.channels, args.duration)
            except RuntimeError as exc:
                if not str(exc).startswith("incomplete Plot") or attempt + 1 == args.attempts:
                    raise
                continue
            after = tcl.words(address, 1)[0]
            if after == before:
                break
            if attempt + 1 == args.attempts:
                raise RuntimeError("FAST upload dropped samples; use a shorter --duration")
    finally:
        if client is not None:
            client.close()
        tcl.close()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=3)
        if log is not None:
            log.close()
    values = decode_samples(raw, "fast", args.channels).T
    return dict(t=np.arange(len(values)) / args.fs, values=values,
                names=list(args.channels), pair=np.full(len(values), -1, dtype=np.int8),
                fs=args.fs, label="Current",
                meta=dict(source="USB FAST Plot", port=args.port,
                          captured_at=datetime.now().isoformat(),
                          firmware_elf=str(args.elf.resolve()),
                          firmware_drop_delta=after - before))


def validate_capture(cap):
    t, values, names, pair = (cap[k] for k in ("t", "values", "names", "pair"))
    if (t.ndim != 1 or len(t) < 2 or values.shape != (len(t), len(names))
            or pair.shape != t.shape or len(set(names)) != len(names)
            or not np.all(np.isfinite(t)) or not np.all(np.isfinite(values))
            or not np.all(np.diff(t) > 0) or not np.isfinite(cap["fs"]) or cap["fs"] <= 0):
        raise ValueError("invalid waveform samples or time axis")
    if not np.all(np.isin(pair, [-1, 0, 1, 2])):
        raise ValueError("invalid phase pair: expected AB=0, AC=1, BC=2")
    missing = [name for name in PHASES if name not in names]
    if len(missing) > 1:
        raise ValueError("select at least two phase-current channels: ia, ib, ic")
    if missing:
        measured = [names.index(name) for name in PHASES if name in names]
        cap["values"] = np.column_stack((values, -values[:, measured].sum(axis=1)))
        cap["names"] = names + missing


def continuous_runs(cap):
    cuts = np.flatnonzero(np.diff(cap["t"]) > 1.5 / cap["fs"]) + 1
    edges = np.r_[0, cuts, len(cap["t"])]
    return list(zip(edges[:-1], edges[1:]))


def zoom_indices(cap, zoom_ms, start_ms):
    t = cap["t"]
    if start_ms is not None:
        start = int(np.searchsorted(t, start_ms / 1000))
        end = int(np.searchsorted(t, (start_ms + zoom_ms) / 1000, side="right"))
    else:
        first, last = max(continuous_runs(cap), key=lambda r: r[1] - r[0])
        count = min(last - first, max(2, int(round(zoom_ms * cap["fs"] / 1000)) + 1))
        start = first + (last - first - count) // 2
        end = start + count
    if end - start < 2:
        raise ValueError(f"zoom contains fewer than two samples: {cap['label']}")
    return start, end


def draw_phase(ax, cap, phase, start, end, origin, zoom):
    t = cap["t"][start:end]
    y = cap["values"][start:end, cap["names"].index(phase)]
    # A missing interval is a break in the line, not a new invented sample.
    breaks = np.flatnonzero(np.diff(t) > 1.5 / cap["fs"]) + 1
    x = np.insert((t - origin) * 1000, breaks, np.nan)
    y = np.insert(y, breaks, np.nan)
    color = COLORS[PHASES.index(phase)]
    style = dict(color=color, linewidth=1.15 if zoom else .95, marker=None)
    ax.plot(x, y, label=f"I{phase[1]}", **style)
    ax.axhline(0, color="#64748b", linewidth=.5, alpha=.5)
    ax.grid(alpha=.20, linewidth=.6)
    ax.set_ylabel("Phase current (A)")
    ax.tick_params(labelsize=9)
    ax.spines[["top", "right"]].set_visible(False)


def render(caps, windows, out):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.titleweight": "bold", "savefig.dpi": 170})
    peak = max(float(np.max(np.abs(c["values"][:, [c["names"].index(p) for p in PHASES]])))
               for c in caps)
    full_limit = max(.01, peak * 1.08)
    peak_zoom = max(float(np.max(np.abs(c["values"][a:b, [c["names"].index(p) for p in PHASES]])))
                    for c, (a, b) in zip(caps, windows))
    zoom_limit = max(.01, peak_zoom * 1.12)
    footer = "Phase-current feedback | Two phases measured, third reconstructed each FOC cycle"
    for cap, (start, end) in zip(caps, windows):
        for suffix, first, last, limit in (("", 0, len(cap["t"]), full_limit),
                                           ("-zoom", start, end, zoom_limit)):
            fig, ax = plt.subplots(figsize=(13, 5))
            fig.subplots_adjust(left=.08, right=.98, top=.81, bottom=.16)
            fig.suptitle(f"{cap['label']} | Three-phase current", fontsize=17, y=.97)
            fig.text(.5, .89,
                     f"Fs = {cap['fs']:g} Hz   |   Ts = {1e6/cap['fs']:g} us   |   {last-first:,} samples",
                     ha="center", color="#475569")
            for phase in PHASES:
                draw_phase(ax, cap, phase, first, last, 0, bool(suffix))
            ax.set_ylim(-limit, limit)
            ax.set_xlim(cap["t"][first] * 1000, cap["t"][last-1] * 1000)
            ax.set_xlabel("Time (ms)")
            ax.legend(loc="upper right", ncol=3, framealpha=.9)
            fig.text(.5, .025, footer, ha="center", fontsize=8, color="#475569")
            fig.savefig(out / f"{cap['stem']}{suffix}.png")
            plt.close(fig)

    if len(caps) > 1:
        fig, axes = plt.subplots(1, len(caps), figsize=(6 * len(caps), 5), squeeze=False, sharey=True)
        fig.subplots_adjust(left=.075, right=.98, top=.79, bottom=.17, wspace=.20)
        fig.suptitle("Three-phase current comparison", fontsize=18, y=.97)
        fig.text(.5, .915, "Same current and time scales | Sample-level zoom", ha="center", color="#475569")
        duration = max((c["t"][b-1] - c["t"][a]) * 1000 for c, (a, b) in zip(caps, windows))
        for col, (cap, (start, end)) in enumerate(zip(caps, windows)):
            ax = axes[0, col]
            for phase in PHASES:
                draw_phase(ax, cap, phase, start, end, cap["t"][start], True)
            ax.set_ylim(-zoom_limit, zoom_limit)
            ax.set_xlim(0, duration)
            ax.set_title(cap["label"])
            ax.set_xlabel("Time from zoom start (ms)")
            ax.legend(loc="upper right", ncol=3, framealpha=.9)
        fig.text(.5, .025, footer, ha="center", fontsize=8, color="#475569")
        fig.savefig(out / "comparison.png")
        plt.close(fig)


def save_capture(cap, out):
    units = [CHANNELS[name][1] for name in cap["names"]]
    np.savez_compressed(out / f"{cap['stem']}.npz", format_version=1,
                        time_s=cap["t"], values=cap["values"], channel_names=cap["names"],
                        channel_units=units, phase_pair=cap["pair"], sample_rate_hz=cap["fs"],
                        label=cap["label"], metadata_json=json.dumps(cap["meta"]))
    header = ["time_s"] + [f"{name}_{unit}" for name, unit in zip(cap["names"], units)] + ["phase_pair"]
    np.savetxt(out / f"{cap['stem']}.csv", np.column_stack((cap["t"], cap["values"], cap["pair"])),
               delimiter=",", header=",".join(header), comments="", fmt="%.9g")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, nargs="+", help="legacy .i16/.json or waveform .npz")
    source.add_argument("--port", help="USB CDC device, e.g. /dev/ttyACM0")
    ap.add_argument("--labels", nargs="+", help="one plot label per capture")
    ap.add_argument("--out", type=Path, default=ROOT / "test-data" / "current-waveform" / datetime.now().strftime("%Y%m%d-%H%M%S"))
    ap.add_argument("--zoom-ms", type=float, default=5.0)
    ap.add_argument("--zoom-start-ms", type=float, help="default: center of longest continuous interval")
    ap.add_argument("--fs", type=float, default=20000.0, help="legacy/live sample rate in Hz; NPZ uses its saved rate")
    ap.add_argument("--duration", type=float, default=.01, help="USB burst duration in seconds (default: 0.01)")
    ap.add_argument("--attempts", type=int, default=10, help="maximum USB burst attempts")
    ap.add_argument("--channels", nargs="+", default=["ia", "ib"], help="USB FAST channels; missing phase is reconstructed")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--elf", type=Path, default=ROOT / "build/Release/AxDr_L_Motor.elf",
                    help="ELF matching the running firmware, for the upload counter address")
    ap.add_argument("--openocd-config", type=Path, default=ROOT / "openocd.cfg")
    args = ap.parse_args()
    if not 1 <= args.attempts <= 100:
        ap.error("attempts must be between 1 and 100")
    for value in (args.fs, args.duration, args.zoom_ms):
        if not np.isfinite(value) or value <= 0:
            ap.error("sample rate, duration and zoom must be positive finite numbers")
    if args.zoom_start_ms is not None and (not np.isfinite(args.zoom_start_ms) or args.zoom_start_ms < 0):
        ap.error("zoom start must be a nonnegative finite number")
    count = len(args.input) if args.input else 1
    if args.labels is not None and (len(args.labels) != count or any(not s.strip() for s in args.labels)):
        ap.error("provide one nonempty label per capture")
    if args.out.exists() and any(args.out.iterdir()):
        ap.error(f"output directory is not empty: {args.out}")
    args.channels = channel_selection("fast", args.channels)
    if args.port and len(set(args.channels) & set(PHASES)) < 2:
        ap.error("USB capture needs at least two phase-current channels")
    args.out.mkdir(parents=True, exist_ok=True)
    caps = [load_capture(p, args.fs) for p in args.input] if args.input else [capture_usb(args, args.out)]
    for n, cap in enumerate(caps):
        validate_capture(cap)
        if args.labels:
            cap["label"] = args.labels[n]
        slug = re.sub(r"[^A-Za-z0-9_-]", "_", cap["label"]).strip("_") or "current"
        cap["stem"] = f"{n+1:02d}-{slug}"
    windows = [zoom_indices(c, args.zoom_ms, args.zoom_start_ms) for c in caps]
    for cap in caps:
        save_capture(cap, args.out)
    render(caps, windows, args.out)
    for cap in caps:
        print(f"{cap['label']}: {len(cap['t']):,} samples -> {args.out / (cap['stem'] + '.png')}")
    if len(caps) > 1:
        print("Comparison:", args.out / "comparison.png")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError) as exc:
        print(f"current_waveform: {exc}", file=sys.stderr)
        raise SystemExit(1)
