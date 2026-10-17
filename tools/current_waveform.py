#!/usr/bin/env python3
"""Render existing AxDr waveform archives; hardware capture belongs to fft.py.

Only reads *.waveform.nmixx or legacy .i16/.npz input. Does not open USB,
SWD or OpenOCD. Unknown/missing timing must never be interpreted as a
continuous 20 kHz waveform or used for electrical-angle sine fitting.

Examples:
    python3 tools/current_waveform.py --input test-data/run/Current-PI.waveform.nmixx
    python3 tools/current_waveform.py --input pi.waveform.nmixx ladrc.waveform.nmixx --labels PI LADRC
"""

import argparse
from datetime import date
import json
from pathlib import Path
import re
import sys

import numpy as np

from fft import CHANNELS

PHASES = ("ia", "ib", "ic")
COLORS = ("#2563eb", "#db6b16", "#169b79")


def load_capture(path, legacy_fs):
    if path.name.endswith(".waveform.nmixx"):
        with np.load(path, allow_pickle=False) as z:
            if (int(z["format_version"][0]) != 1 or
                    str(z["kind"][0]) != "waveform"):
                raise ValueError(f"unsupported waveform archive: {path}")
            group = str(z["group"][0])
            raw = z["raw_samples"]
            fs = float(z["sample_rate_hz"][0])
            names = list(map(str, z["channel_name"]))
            scale = z["channel_scale"]
            seq = z["frame_seq"].astype(np.int64)
            counts = z["frame_sample_count"].astype(np.int64)
            quality = str(z["quality"][0])
            reason = str(z["quality_reason"][0])
            if (group != "fast" or raw.ndim != 2 or
                    raw.dtype.kind != "i" or raw.dtype.itemsize != 2 or
                    len(raw) < 2 or len(names) != raw.shape[1] or
                    len(scale) != len(names) or len(seq) != len(counts) or
                    int(counts.sum()) != len(raw) or
                    quality not in ("valid", "unknown", "invalid") or
                    not np.isfinite(fs) or fs <= 0 or
                    not np.isfinite(scale).all() or
                    len(set(names)) != len(names)):
                raise ValueError(f"malformed FAST waveform archive: {path}")
            if (quality == "valid" and
                    (np.any(((np.diff(seq) & 0xFFFF) != 1)) or
                     int(z["usb_seq_lost"][0]) != 0 or
                     int(z["malformed_frames"][0]) != 0 or
                     int(z["firmware_drop_before"][0]) < 0 or
                     int(z["firmware_drop_after"][0]) < 0 or
                     int(z["firmware_drop_before"][0]) != int(z["firmware_drop_after"][0]))):
                raise ValueError(f"archive falsely claims valid continuous timing: {path}")
            values = raw.astype(np.float64) * scale
            meta = json.loads(str(z["metadata_json"][0]))
            pair = np.full(len(raw), -1, dtype=np.int8)
            return dict(values=values, names=names, pair=pair, fs=fs,
                        quality=quality, reason=reason, meta=meta,
                        label=meta.get("controller") or path.name.split(".waveform")[0],
                        t=np.arange(len(raw), dtype=np.float64) / fs)

    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as z:
            if int(z["format_version"].item()) != 1:
                raise ValueError("unsupported legacy NPZ")
            values = z["values"]
            t = z["time_s"]
            names = list(map(str, z["channel_names"]))
            pair = z["phase_pair"]
            meta = json.loads(str(z["metadata_json"].item()))
            fs = float(z["sample_rate_hz"].item())
            label = str(z["label"].item())
    else:
        raw_path = path.with_suffix(".i16") if path.suffix == ".json" else path
        meta = json.loads(raw_path.with_suffix(".json").read_text(encoding="utf-8"))
        columns = meta["columns"]
        raw = np.fromfile(raw_path, dtype="<i2")
        if not len(raw) or len(raw) % len(columns):
            raise ValueError(f"incomplete legacy sample row: {raw_path}")
        raw = raw.reshape(-1, len(columns)).astype(np.float64)
        index = {col[0]: n for n, col in enumerate(columns)}
        if "Tick" not in index:
            raise ValueError("legacy .i16 requires Tick for the time axis")
        step = np.diff(raw[:, index["Tick"]].astype(np.int64)) & 0xFFFF
        if np.any(step == 0):
            raise ValueError("duplicate legacy Tick")
        t = np.r_[0, np.cumsum(step)] / legacy_fs
        ids = {info[0]: name for name, info in CHANNELS.items()}
        names, parts = [], []
        for n, (_, param_id, scale) in enumerate(columns):
            if param_id in ids:
                names.append(ids[param_id])
                parts.append(raw[:, n] * scale)
        values = np.asarray(parts).T
        fs = legacy_fs
        pair = raw[:, index["Pair"]].astype(np.int8) if "Pair" in index else np.full(len(t), -1)
        label = meta.get("mode", raw_path.stem)

    if (values.ndim != 2 or values.shape != (len(t), len(names)) or
            pair.shape != t.shape or not np.isfinite(values).all() or
            not np.isfinite(t).all() or np.any(np.diff(t) <= 0)):
        raise ValueError(f"invalid legacy capture: {path}")
    return dict(values=values, names=names, pair=pair, fs=fs,
                quality="unknown", reason="legacy format: firmware drops not verified",
                meta=meta, label=label, t=t)


def validate(cap):
    if any(p not in cap["names"] for p in PHASES):
        present = [p for p in PHASES if p in cap["names"]]
        missing = [p for p in PHASES if p not in cap["names"]]
        if len(present) != 2 or len(missing) != 1:
            raise ValueError("waveform needs two or three of ia, ib, ic")
        cap["values"] = np.column_stack((cap["values"],
                                        -cap["values"][:, [cap["names"].index(p) for p in present]].sum(axis=1)))
        cap["names"].extend(missing)
    if len(cap["t"]) < 2 or not np.isfinite(cap["values"]).all():
        raise ValueError("empty/non-finite waveform")
    return cap


def zoom_range(cap, zoom_ms, start_ms):
    n = len(cap["t"])
    length = min(n, max(2, int(round(zoom_ms * cap["fs"] / 1000))))
    if start_ms is None:
        start = max(0, (n - length) // 2)
    else:
        if cap["quality"] != "valid":
            raise ValueError("zoom start in milliseconds requires verified continuous timing")
        start = int(round(start_ms * cap["fs"] / 1000))
    if start < 0 or start + length > n:
        raise ValueError("zoom range is outside saved samples")
    return start, start + length


def draw_series(ax, cap, first, last, point_style=False):
    x = (cap["t"][first:last] * 1000 if cap["quality"] == "valid"
         else np.arange(first, last))
    for phase, color in zip(PHASES, COLORS):
        y = cap["values"][first:last, cap["names"].index(phase)]
        ax.plot(x, y, color=color, label=phase.upper(),
                linewidth=.9 if cap["quality"] == "valid" else 0,
                marker="." if point_style or cap["quality"] != "valid" else None,
                markersize=2.6, alpha=.88)
    ax.grid(which="major", alpha=.3)
    ax.minorticks_on()
    ax.grid(which="minor", alpha=.12)
    ax.set_ylabel("Phase-current feedback (A)")
    ax.set_xlabel("Time (ms)" if cap["quality"] == "valid"
                  else "Received sample index (timing unverified)")
    ax.legend(ncol=3, loc="upper right")


def plot_waveform(cap, out, zoom_ms, start_ms):
    import matplotlib.pyplot as plt

    first, last = zoom_range(cap, zoom_ms, start_ms)
    fig, axes = plt.subplots(2, 1, figsize=(11.69, 8.27), dpi=180)
    fig.subplots_adjust(left=.09, right=.98, bottom=.08, top=.83, hspace=.38)
    title = f"{cap['label']} | Three-phase current"
    fig.suptitle(title, fontsize=15, y=.96)
    fig.text(.5, .895,
             f"Fs={cap['fs']:g} Hz   N={len(cap['t'])}   Quality={cap['quality']}",
             ha="center", fontsize=10)
    draw_series(axes[0], cap, 0, len(cap["t"]))
    axes[0].set_title("Full received capture", loc="left")
    draw_series(axes[1], cap, first, last, point_style=True)
    axes[1].set_title("Sample-level zoom", loc="left")
    fig.text(.5, .022, cap["reason"] or
             "Two phases are measured per FOC cycle; the third is reconstructed",
             ha="center", fontsize=8)
    path = out / f"{cap['stem']}.waveform.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def fundamental(cap):
    """Fit DC and electrical-angle first harmonic; residual is NOT ADC noise."""
    if cap["quality"] != "valid" or "theta_e" not in cap["names"]:
        return None
    theta = cap["values"][:, cap["names"].index("theta_e")]
    unwrapped = np.unwrap(theta)
    if np.ptp(unwrapped) < 4 * np.pi:
        return None
    design = np.column_stack((np.ones(len(theta)), np.cos(theta), np.sin(theta)))
    if np.linalg.cond(design) > 1e5:
        return None
    fitted = {}
    for phase in PHASES:
        y = cap["values"][:, cap["names"].index(phase)]
        coeff = np.linalg.lstsq(design, y, rcond=None)[0]
        rms = float(np.hypot(coeff[1], coeff[2]) / np.sqrt(2))
        residual = float(np.sqrt(np.mean((y - design @ coeff) ** 2)))
        fitted[phase] = (coeff, rms, residual)
    return theta % (2 * np.pi), fitted


def plot_phase(cap, out):
    fit = fundamental(cap)
    if fit is None:
        return None
    import matplotlib.pyplot as plt

    angle, fitted = fit
    fig, ax = plt.subplots(figsize=(11.69, 8.27), dpi=180)
    fig.subplots_adjust(left=.10, right=.98, top=.84, bottom=.13)
    smooth = np.linspace(0, 2 * np.pi, 400)
    stats = []
    for phase, color in zip(PHASES, COLORS):
        coeff, rms, residual = fitted[phase]
        y = cap["values"][:, cap["names"].index(phase)]
        ax.scatter(np.rad2deg(angle), y, s=3, alpha=.16, color=color)
        ax.plot(np.rad2deg(smooth),
                coeff[0] + coeff[1] * np.cos(smooth) + coeff[2] * np.sin(smooth),
                lw=1.5, color=color, label=f"{phase.upper()} (fund. {rms:.3f} A)")
        stats.append(f"{phase.upper()} residual={residual:.3f} A")
    ax.set_xlim(0, 360)
    ax.set_xlabel("FOC electrical angle (deg)")
    ax.set_ylabel("Phase-current feedback (A)")
    ax.grid(alpha=.25)
    ax.legend(loc="upper right")
    fig.suptitle(f"{cap['label']} | Electrical-angle overlay and fundamental", y=.96, fontsize=14)
    fig.text(.5, .897, "Light points: measured feedback over multiple periods. Lines: first-harmonic fit",
             ha="center", fontsize=9)
    fig.text(.5, .034, " | ".join(stats) + "    Residual includes harmonics, noise and model error.",
             ha="center", fontsize=9)
    path = out / f"{cap['stem']}.phase.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def plot_comparison(caps, out, zoom_ms, start_ms):
    if len(caps) < 2:
        return None
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    windows = [zoom_range(c, zoom_ms, start_ms) for c in caps]
    fig, axes = plt.subplots(1, len(caps), figsize=(11.69, 8.27), dpi=180,
                             sharey=True, squeeze=False)
    fig.subplots_adjust(left=.08, right=.98, bottom=.15, top=.85, wspace=.13)
    for ax, cap, (first, last) in zip(axes[0], caps, windows):
        x = np.arange(last - first) / cap["fs"] * 1000 if cap["quality"] == "valid" else np.arange(last - first)
        for phase, color in zip(PHASES, COLORS):
            y = cap["values"][first:last, cap["names"].index(phase)]
            ax.plot(x, y, color=color, lw=.9 if cap["quality"] == "valid" else 0,
                    marker=".", markersize=2, label=phase.upper())
        ax.set_title(f"{cap['label']} ({cap['quality']})", fontsize=11)
        ax.set_xlabel("Relative time (ms)" if cap["quality"] == "valid" else "Received sample index")
        ax.grid(alpha=.22)
        ax.xaxis.set_major_locator(MaxNLocator(6))
    axes[0, 0].set_ylabel("Phase-current feedback (A)")
    axes[0, -1].legend(ncol=3)
    fig.suptitle("Phase-current comparison | common amplitude scale", fontsize=15, y=.95)
    path = out / "comparison.waveform.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, nargs="+", required=True,
                        help="waveform.nmixx, or legacy .i16/.json/.npz")
    parser.add_argument("--labels", nargs="+", help="optional label per input")
    parser.add_argument("--out", type=Path,
                        default=Path("test-data") / f"{date.today():%Y%m%d}_current_waveform" / "plots")
    parser.add_argument("--zoom-ms", type=float, default=5.0)
    parser.add_argument("--zoom-start-ms", type=float)
    parser.add_argument("--fs", type=float, default=20000.0, help="only for legacy .i16")
    args = parser.parse_args()
    if (not np.isfinite(args.fs) or args.fs <= 0 or
            not np.isfinite(args.zoom_ms) or args.zoom_ms <= 0 or
            (args.zoom_start_ms is not None and
             (not np.isfinite(args.zoom_start_ms) or args.zoom_start_ms < 0))):
        parser.error("invalid sample rate or zoom")
    if args.labels is not None and len(args.labels) != len(args.input):
        parser.error("--labels must match --input count")
    caps = []
    for i, path in enumerate(args.input):
        cap = validate(load_capture(path, args.fs))
        if args.labels:
            cap["label"] = args.labels[i]
        cap["stem"] = f"{i+1:02d}-{re.sub(r'[^A-Za-z0-9_-]', '_', cap['label'])[:80]}"
        caps.append(cap)
    targets = [args.out / f"{c['stem']}.waveform.png" for c in caps]
    targets += [args.out / f"{c['stem']}.phase.png" for c in caps if fundamental(c) is not None]
    if len(caps) > 1:
        targets.append(args.out / "comparison.waveform.png")
    if any(p.exists() for p in targets):
        parser.error("output files exist; choose another --out (archives are never overwritten)")
    args.out.mkdir(parents=True, exist_ok=True)
    for cap in caps:
        print("Waveform:", plot_waveform(cap, args.out, args.zoom_ms, args.zoom_start_ms))
        phase = plot_phase(cap, args.out)
        if phase is not None:
            print("Electrical angle:", phase)
        else:
            print(f"Electrical-angle fit unavailable ({cap['quality']}; need verified data and theta_e)")
    comparison = plot_comparison(caps, args.out, args.zoom_ms, args.zoom_start_ms)
    if comparison is not None:
        print("Comparison:", comparison)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print(f"current_waveform: {exc}", file=sys.stderr)
        raise SystemExit(1)
