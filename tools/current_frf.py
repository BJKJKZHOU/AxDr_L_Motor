#!/usr/bin/env python3
"""Capture and replot sensored Id/Iq/Speed closed-loop Bode responses.

Requires numpy; pyserial for hardware capture, matplotlib for plotting.
The motor must be manually set to TORQUE/SPEED + ENABLED. Never sends Enable,
Run or persistent Save. Frequency points can be manual or integer-Hz logarithmic.
"""

import argparse
from datetime import date
import math
from pathlib import Path
import struct
import textwrap
import time

import numpy as np


def responses(ref, fbk, output):
    """Return complex T, S, C, G, L, H_U from measured same-clock phasors."""
    ref, fbk, output = complex(ref), complex(fbk), complex(output)
    error = ref - fbk
    if abs(ref) < 1e-12:
        raise ValueError("reference phasor is zero")
    if abs(error) < 1e-12 or abs(output) < 1e-12:
        raise ValueError("error or voltage phasor is too small for L/G measurement")
    return {
        "T": fbk / ref,
        "S": error / ref,
        "C": output / error,
        "G": fbk / output,
        "L": fbk / error,
        "HU": output / ref,
    }


def bode_arrays(freq_hz, *signals, phase_anchor=None):
    f = np.asarray(freq_hz, dtype=float)
    if f.ndim != 1 or len(f) < 2 or not np.all(np.isfinite(f)) or np.any(f <= 0):
        raise ValueError("need at least two positive finite frequencies")
    if any(len(s) != len(f) for s in signals):
        raise ValueError("frequency and response lengths differ")
    order = np.argsort(f)
    f = f[order]
    if np.any(np.diff(f) <= 0):
        raise ValueError("frequency points must be unique")
    outputs = []
    for signal in signals:
        h = np.asarray(signal, dtype=np.complex128)[order]
        if h.shape != f.shape:
            raise ValueError("response must be one-dimensional")
        gain = 20 * np.log10(np.maximum(np.abs(h), 1e-20))
        gain[~np.isfinite(h)] = np.nan
        phase = np.full(len(f), np.nan)
        valid = np.flatnonzero(np.isfinite(h) & (abs(h) > 1e-20))
        # A singular T/(1-T) leaves a gap, not a fabricated finite response.
        for section in np.split(valid, np.flatnonzero(np.diff(valid) > 1) + 1):
            if not len(section):
                continue
            angles = np.angle(h[section], deg=True)
            unwrapped = np.rad2deg(np.unwrap(np.deg2rad(angles)))
            anchor = (np.argmin(abs(np.log(f[section] / phase_anchor)))
                      if phase_anchor is not None else 0)
            unwrapped -= 360 * np.round((unwrapped[anchor] - angles[anchor]) / 360)
            phase[section] = unwrapped
        outputs.append((gain, phase))
    return f, outputs


def _crossings(freq, values, target, direction=None):
    """Log-frequency interpolate every measured threshold crossing."""
    out = []
    for i in range(1, len(freq)):
        a, b = values[i - 1] - target, values[i] - target
        if not np.isfinite(a + b) or b == a or b == 0:
            continue
        # An exact interior crossing is counted once, only if bracketed.
        left = values[i - 2] - target if a == 0 and i > 1 else a
        if not np.isfinite(left) or left * b >= 0:
            continue
        if direction == "down" and not (left > 0 > b):
            continue
        if direction == "up" and not (left < 0 < b):
            continue
        frac = -a / (b - a)
        if 0 <= frac <= 1:
            log_f = np.log(freq[i - 1]) + frac * (np.log(freq[i]) - np.log(freq[i - 1]))
            out.append((float(np.exp(log_f)), i - 1, float(frac)))
    return out


def bode_metrics(freq_hz, open_loop, closed_loop, low_frequency_points=3,
                 *, quality="unknown", quality_reason="", point_valid=None):
    """Conditional unit-negative-feedback estimates, never a stability proof.

    PM is the signed local angle to the negative real axis; GM considers every
    odd-180-degree crossing. Neither depends on the initial unwrap branch.
    Sampling guards cannot rule out hidden turns between measured frequencies.
    """
    if quality not in ("unknown", "valid", "invalid"):
        raise ValueError("quality must be unknown, valid or invalid")
    if low_frequency_points < 1:
        raise ValueError("low_frequency_points must be positive")
    f, ((l_db, l_phase), (t_db, _)) = bode_arrays(freq_hz, open_loop, closed_loop)
    order = np.argsort(freq_hz)
    reliable = np.ones(len(f), dtype=bool)
    if point_valid is not None:
        flags = np.asarray(point_valid)
        if flags.shape != f.shape or not np.all(np.isin(flags, [0, 1])):
            raise ValueError("invalid point quality mask")
        reliable = flags[order].astype(bool)
    low_db = float(np.mean(t_db[:min(low_frequency_points, len(t_db))]))
    threshold = low_db - 3.0
    cutoffs = _crossings(f, t_db, threshold, direction="down")
    unity = _crossings(f, l_db, 0.0)
    phase_crossings = []
    finite_phase = l_phase[np.isfinite(l_phase)]
    if len(finite_phase):
        first = math.ceil((finite_phase.min() + 180) / 360)
        last = math.floor((finite_phase.max() + 180) / 360)
        for branch in range(first, last + 1):
            phase_crossings.extend(_crossings(f, l_phase, -180 + 360 * branch))
    phase_margin = [((float(l_phase[i] + frac * (l_phase[i + 1] - l_phase[i]))
                      % 360) - 180, freq) for freq, i, frac in unity]
    gain_margin = [(-float(l_db[i] + frac * (l_db[i + 1] - l_db[i])), freq)
                   for freq, i, frac in phase_crossings]
    # Both rising and falling crossings matter; retain the smallest distance
    # to instability, including its sign (as in classical SISO margins).
    pm = min(phase_margin, key=lambda item: abs(item[0])) if phase_margin else None
    gm = min(gain_margin, key=lambda item: abs(item[0])) if gain_margin else None
    reason = (quality_reason or f"measurement quality {quality}") if quality != "valid" else ""

    def crossing_reason(crossings, phase):
        if reason:
            return reason
        if not crossings:
            return "no bracketed crossing in sweep"
        for _, i, frac in crossings:
            # Include the other side of an exact interior crossing as well.
            start = i - 1 if frac == 0 else i
            section = slice(start, i + 2)
            if not np.all(reliable[section]):
                return "unreliable samples near crossing"
            if not np.all(np.isfinite(phase[section])):
                return "undefined phase near crossing"
            if (np.any(np.diff(np.log2(f[section])) > 1.000001) or
                np.any(abs(np.diff(phase[section])) >= 120)):
                return "crossing insufficiently resolved"
        return ""

    bw_reason = crossing_reason(cutoffs, np.zeros(len(f)))
    if not reason and (not np.isfinite(low_db) or
                       not np.all(reliable[:low_frequency_points])):
        bw_reason = "unreliable low-frequency reference"
    pm_reason = crossing_reason(unity, l_phase)
    gm_reason = crossing_reason(phase_crossings, l_phase)
    if not pm_reason and pm is not None and abs(pm[0]) >= 179:
        pm_reason = "phase-margin sign ambiguous near positive real axis"
    if bw_reason:
        cutoffs = []
    if pm_reason:
        pm = None
    if gm_reason:
        gm = None
    return {
        "low_frequency_reference_db": low_db,
        "relative_minus_3db_threshold_db": threshold,
        "relative_minus_3db_hz": cutoffs[0][0] if cutoffs else None,
        "relative_minus_3db_all_hz": [item[0] for item in cutoffs],
        "gain_crossover_hz": pm[1] if pm else None,
        "phase_margin_deg": pm[0] if pm else None,
        "phase_crossing_hz": gm[1] if gm else None,
        "gain_margin_db": gm[0] if gm else None,
        "min_frequency_hz": float(f[0]),
        "max_frequency_hz": float(f[-1]),
        "quality": quality,
        "quality_reason": quality_reason,
        "bandwidth_reason": bw_reason,
        "phase_margin_reason": pm_reason,
        "gain_margin_reason": gm_reason,
    }


def plot_bode_a4(freq_hz, open_loop, sensitivity, closed_loop, title, destination,
                 *, quality="unknown", quality_reason="", point_valid=None):
    """Deterministic landscape-A4 Bode layout, magnitude above phase (PNG)."""
    import matplotlib as mpl
    from matplotlib import pyplot as plt
    from matplotlib.ticker import LogLocator, NullFormatter, MultipleLocator

    loop = np.asarray(open_loop)
    finite = np.flatnonzero(np.isfinite(loop) & (abs(loop) > 1e-20))
    anchor = (np.asarray(freq_hz)[finite[np.argmin(abs(np.log(abs(loop[finite]))))]]
              if len(finite) else None)
    f, spectra = bode_arrays(freq_hz, open_loop, sensitivity, closed_loop,
                            phase_anchor=anchor)
    metrics = bode_metrics(freq_hz, open_loop, closed_loop, quality=quality,
                           quality_reason=quality_reason, point_valid=point_valid)
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)

    gains = np.concatenate([item[0] for item in spectra])
    phases = np.concatenate([item[1] for item in spectra])
    gains = gains[np.isfinite(gains)]
    phases = phases[np.isfinite(phases)]
    if not len(gains) or not len(phases):
        raise ValueError("no finite Bode response to plot")

    # Quantized bounds avoid different axes for near-identical test runs.
    threshold = metrics["relative_minus_3db_threshold_db"]
    if not np.isfinite(threshold):
        threshold = float(gains.min())
    gain_min = 5.0 * np.floor((min(gains.min(), threshold) - 3.0) / 5.0)
    gain_max = 5.0 * np.ceil((max(gains.max(), threshold) + 3.0) / 5.0)
    phase_min = 30.0 * np.floor((phases.min() - 10.0) / 30.0)
    phase_max = 30.0 * np.ceil((phases.max() + 10.0) / 30.0)

    style = {"font.family": "DejaVu Sans", "font.size": 10,
             "axes.titlesize": 10, "axes.labelsize": 10,
             "xtick.labelsize": 9, "ytick.labelsize": 9,
             "figure.facecolor": "white", "axes.facecolor": "white",
             "savefig.facecolor": "white"}
    with mpl.rc_context(style):
        fig, (mag, phase) = plt.subplots(
            2, 1, sharex=True, figsize=(11.69, 8.27), dpi=180,
            gridspec_kw={"height_ratios": [1, 1]},
        )
        for (gain_db, phase_deg), label, color in zip(
            spectra, ("Inferred L = T/(1-T)", "Sensitivity S", "Closed Loop T"),
            ("#367EBC", "#D86B43", "#DCA832"),
        ):
            mag.semilogx(f, gain_db, label=label, lw=1.25, color=color)
            phase.semilogx(f, phase_deg, label=label, lw=1.25, color=color)

        for axis in (mag, phase):
            axis.set_xlim(float(f[0]), float(f[-1]))
            axis.xaxis.set_major_locator(LogLocator(base=10, numticks=15))
            axis.xaxis.set_minor_locator(
                LogLocator(base=10, subs=np.arange(2, 10), numticks=100))
            axis.xaxis.set_minor_formatter(NullFormatter())
            axis.grid(which="major", color="0.6", lw=0.60, alpha=0.40)
            axis.grid(which="minor", color="0.6", ls=":", lw=0.45, alpha=0.45)
            axis.tick_params(which="major", direction="in", top=True,
                             right=True, length=6)
            axis.tick_params(which="minor", direction="in", top=True,
                             right=True, length=3)

        mag.set_ylim(gain_min, gain_max)
        phase.set_ylim(phase_min, phase_max)
        mag.yaxis.set_major_locator(MultipleLocator(5))
        mag.yaxis.set_minor_locator(MultipleLocator(1))
        phase.yaxis.set_major_locator(MultipleLocator(30))
        phase.yaxis.set_minor_locator(MultipleLocator(5))
        for axis in (mag, phase):
            axis.grid(which="minor", axis="y", color="0.65",
                      ls=":", lw=0.45, alpha=0.40)

        mag.set_ylabel("Magnitude (dB)")
        phase.set_ylabel("Phase (deg)")
        phase.set_xlabel("Frequency (Hz)")
        mag.legend(loc="lower left", fontsize=9, framealpha=0.92)

        if not metrics["bandwidth_reason"]:
            mag.axhline(threshold, color="0.40", ls="--", lw=0.9, alpha=0.75)
        bw = metrics["relative_minus_3db_hz"]
        if bw is not None:
            for axis in (mag, phase):
                axis.axvline(bw, color="#D86B43", ls="--",
                             lw=1.05, alpha=0.9)
            cutoff_text = f"-3 dB = {bw:.1f} Hz"
        else:
            cutoff_text = "-3 dB = N/A"

        pm = metrics["phase_margin_deg"]
        gm = metrics["gain_margin_db"]
        fc = metrics["gain_crossover_hz"]
        detail = f"PM = {pm:.1f} deg" if pm is not None else "PM = N/A"
        detail += "   |   " + (f"GM = {gm:.1f} dB" if gm is not None else "GM = N/A")
        detail += "   |   " + (f"fc = {fc:.1f} Hz" if fc is not None else "fc = N/A")

        fig.suptitle(f"{title}   |   {cutoff_text}", y=0.980, fontsize=12)
        fig.text(0.5, 0.948, detail, ha="center", va="top",
                 fontsize=10, color="0.35")
        reasons = list(dict.fromkeys(metrics[key] for key in
                       ("bandwidth_reason", "phase_margin_reason", "gain_margin_reason")
                       if metrics[key]))
        note = "Unit negative feedback assumed; L/margins inferred, not measured open-loop."
        if reasons:
            note += " N/A: " + "; ".join(reasons)
        fig.text(.5, .025, textwrap.fill(note, 135), ha="center", va="center",
                 fontsize=8, color="0.35")
        fig.subplots_adjust(left=0.075, right=0.985, bottom=0.087,
                            top=0.895, hspace=0.12)
        fig.savefig(output, dpi=180, metadata={"Software": "NMIXX"})
        plt.close(fig)
    return metrics

LOOPS = {"id": 0, "iq": 1, "speed": 2}
NAMES = ("Current Id", "Current Iq", "Speed")

def save_bode(path, freq_hz, ref, fbk, out, loop_id, bandwidth_hz,
              ff_enabled, tune_source, kp, ki, workpoint=None, pole_pairs=None,
              *, quality="unknown", quality_reason="", point_valid=None,
              fit_residual=None, fit_drift=None):
    """Store measured same-clock phasors in the common NMIXX v2 container."""
    freq = np.asarray(freq_hz, dtype=np.float64)
    phasors = [np.asarray(x, dtype=np.complex128) for x in (ref, fbk, out)]
    if freq.ndim != 1 or len(freq) < 2 or any(x.shape != freq.shape for x in phasors):
        raise ValueError("frequency/phasor shapes do not match")
    extra = {}
    if quality not in ("unknown", "valid", "invalid"):
        raise ValueError("invalid measurement quality")
    extra["quality"] = np.array([quality])
    extra["quality_reason"] = np.array([quality_reason])
    if point_valid is not None:
        flags = np.asarray(point_valid)
        if flags.shape != freq.shape or not np.all(np.isin(flags, [0, 1])):
            raise ValueError("invalid point quality mask")
        extra["point_valid"] = flags.astype(np.uint8)[None, :]
    for key, values in (("fit_residual", fit_residual), ("fit_drift", fit_drift)):
        if values is not None:
            values = np.asarray(values, dtype=float)
            if values.shape != (len(freq), 3) or not np.all(np.isfinite(values)) or np.any(values < 0):
                raise ValueError(f"invalid {key}")
            extra[key] = values[None, :, :]
    if workpoint is not None:
        if not pole_pairs or pole_pairs < 1:
            raise ValueError("invalid pole pairs for speed workpoint")
        for key, values in workpoint.items():
            values = np.asarray(values, dtype=np.float64)
            if values.shape != freq.shape:
                raise ValueError(f"workpoint {key} shape does not match frequencies")
            extra[key] = values[None, :]
        extra["pole_pairs"] = np.array([pole_pairs], dtype=np.int32)
    with Path(path).open("wb") as handle:
        np.savez_compressed(
            handle, format_version=np.array([2], dtype=np.int32),
            freq_hz=freq,
            loop_id=np.array([loop_id], dtype=np.int32),
            bandwidth_hz=np.array([bandwidth_hz], dtype=np.float64),
            ff_enabled=np.array([ff_enabled], dtype=np.int32),
            tune_source=np.array([tune_source], dtype=np.int32),
            kp=np.array([kp], dtype=np.float64),
            ki=np.array([ki], dtype=np.float64),
            ref=phasors[0][None, :], fbk=phasors[1][None, :],
            out=phasors[2][None, :],
            **extra,
        )


def replot(source, output):
    """Recompute each archived condition from measured complex phasors."""
    with np.load(source, allow_pickle=False) as archive:
        if int(archive["format_version"][0]) != 2:
            raise ValueError("unsupported NMIXX data version")
        freq = np.asarray(archive["freq_hz"], dtype=np.float64)
        loop_id = int(archive["loop_id"][0])
        if loop_id not in LOOPS.values():
            raise ValueError("unknown NMIXX loop")
        bw = np.asarray(archive["bandwidth_hz"], dtype=np.float64)
        ff = np.asarray(archive["ff_enabled"], dtype=np.int32)
        tune = np.asarray(archive["tune_source"], dtype=np.int32)
        kp = np.asarray(archive["kp"], dtype=np.float64)
        ki = np.asarray(archive["ki"], dtype=np.float64)
        reference = np.asarray(archive["ref"], dtype=np.complex128)
        feedback = np.asarray(archive["fbk"], dtype=np.complex128)
        output_signal = np.asarray(archive["out"], dtype=np.complex128)
        shape = (len(bw), len(freq))
        quality = np.asarray(archive.get("quality", np.full(len(bw), "unknown")))
        reason = np.asarray(archive.get("quality_reason", np.full(len(bw), "measurement quality unknown (legacy archive)")))
        point_valid = np.asarray(archive.get("point_valid", np.ones(shape, dtype=np.uint8)))
        if (quality.shape != (len(bw),) or reason.shape != (len(bw),) or
            quality.dtype.kind != "U" or reason.dtype.kind != "U" or
            not np.all(np.isin(quality, ["unknown", "valid", "invalid"])) or
            point_valid.shape != shape or not np.all(np.isin(point_valid, [0, 1]))):
            raise ValueError("invalid NMIXX quality metadata")
        if (freq.ndim != 1 or len(freq) < 2 or
            any(v.shape != (len(bw),) for v in (ff, tune, kp, ki)) or
            any(v.shape != shape for v in (reference, feedback, output_signal)) or
            any(not np.all(np.isfinite(v)) for v in (reference, feedback, output_signal)) or
            np.any(abs(reference) < 1e-12)):
            raise ValueError("invalid NMIXX phasors or array dimensions")
        T_all = feedback / reference

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    metrics = {}
    for i, bandwidth in enumerate(bw):
        T = T_all[i]
        S = 1 - T
        L = np.divide(T, S, out=np.full(T.shape, complex(np.nan, np.nan)),
                      where=abs(S) >= 1e-10)
        key = f"{NAMES[loop_id].replace(' ', '_')}_BW{bandwidth:g}_FF{int(ff[i])}"
        if key in metrics:
            raise ValueError(f"duplicate NMIXX condition: {key}")
        if tune[i] == 1:
            label = f"{NAMES[loop_id]} (Manual Kp {kp[i]:.4g}, Ki {ki[i]:.4g})"
        else:
            label = f"{NAMES[loop_id]} (PI setting {bandwidth:g} Hz)"
        if loop_id != LOOPS["speed"]:
            label += f", FF {int(ff[i])}"
        metrics[key] = plot_bode_a4(
            freq, L, S, T, label, output / f"bode_{key}.png",
            quality=str(quality[i]), quality_reason=str(reason[i]),
            point_valid=point_valid[i],
        )
    return metrics

try:
    import serial
except ImportError:
    serial = None  # Offline --result plotting does not need serial access.

from parameter_ids_generated import (
    PARAM_MOTOR_MODE, PARAM_MOTOR_STATE,
    PARAM_RUN_ID, PARAM_SIGNAL_ID_REF, PARAM_SIGNAL_ID_PI_OUT,
    PARAM_SIGNAL_OUT, PARAM_SIGNAL_FREQ_HZ, PARAM_SIGNAL_AMP_A,
    PARAM_SIGNAL_TIME_S, PARAM_CTRL_CURRENT_FF_ENABLE, PARAM_CTRL_CURRENT_BW_HZ,
    PARAM_SIGNAL_TARGET, PARAM_SIGNAL_SPEED_AMP,
    PARAM_SIGNAL_IQ_REF, PARAM_SIGNAL_IQ_PI_OUT,
    PARAM_SIGNAL_SPEED_REF, PARAM_SIGNAL_SPEED_FBK,
    PARAM_SIGNAL_SPEED_IQ_OUT, PARAM_SIGNAL_SPEED_INJ,
    PARAM_CTRL_SPEED_BW_HZ, PARAM_CTRL_SPEED_SOURCE,
    PARAM_CTRL_CURRENT_SOURCE, PARAM_LIMIT_I_EFFECTIVE,
    PARAM_CTRL_SPEED_KP_EFFECTIVE,
    PARAM_CTRL_SPEED_KI_EFFECTIVE, PARAM_RUN_IQ,
    PARAM_CTRL_ID_KP, PARAM_CTRL_ID_KI, PARAM_CTRL_IQ_KP, PARAM_CTRL_IQ_KI,
    PARAM_MOTOR_RS, PARAM_MOTOR_LD, PARAM_MOTOR_LQ,
    PARAM_MOTOR_PP, PARAM_REF_WM, PARAM_RUN_WM, PARAM_TARGET_SPEED,
    ACTION_SIGNAL_START, ACTION_SIGNAL_ABORT,
)

MAGIC = b"AXDR"
NODE_ID = 1
MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_PARAMETER = 0x07
MSG_FAST_DATA = 0x18
PARAM_READ = 1
PARAM_WRITE = 2
PARAM_FLOAT = 2
PARAM_ACTION = 6
PLOT_CONFIG = 1
PLOT_START = 2
PLOT_STOP = 3
FAST_RATE_HZ = 20000.0
FAST_CONFIG_ID = 21
CHANNELS = {
    "id": (
        ("Injection", PARAM_SIGNAL_OUT, 0.001),
        ("Ref", PARAM_SIGNAL_ID_REF, 0.001),
        ("Fbk", PARAM_RUN_ID, 0.001),
        ("Out", PARAM_SIGNAL_ID_PI_OUT, 0.001),
    ),
    "iq": (
        ("Injection", PARAM_SIGNAL_OUT, 0.001),
        ("Ref", PARAM_SIGNAL_IQ_REF, 0.001),
        ("Fbk", PARAM_RUN_IQ, 0.001),
        ("Out", PARAM_SIGNAL_IQ_PI_OUT, 0.001),
    ),
    "speed": (
        ("Injection", PARAM_SIGNAL_SPEED_INJ, 0.02),
        ("Ref", PARAM_SIGNAL_SPEED_REF, 0.05),
        ("Fbk", PARAM_SIGNAL_SPEED_FBK, 0.05),
        ("Out", PARAM_SIGNAL_SPEED_IQ_OUT, 0.001),
    ),
}
CANFD_LEN = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)


def wire(msg_type, payload):
    length = next(n for n in CANFD_LEN if n >= len(payload))
    return MAGIC + struct.pack("<HB", (msg_type << 6) | NODE_ID, length) + payload.ljust(length, b"\x00")


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 7:
                break
            msg_id, size = struct.unpack_from("<HB", self.buffer, 4)
            if msg_id > 0x07FF or size not in CANFD_LEN:
                del self.buffer[0]
                continue
            end = 7 + size
            if len(self.buffer) < end:
                break
            frames.append((msg_id >> 6, bytes(self.buffer[7:end])))
            del self.buffer[:end]
        return frames


class Client:
    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError("pyserial is required for hardware capture")
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=1.0)
        self.parser = Parser()
        self.txn = 0
        self.fast = bytearray()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = False
        self.channels = CHANNELS["id"]

    def close(self):
        self.ser.close()

    def poll(self):
        responses = []
        for msg_type, payload in self.parser.feed(self.ser.read(4096)):
            if msg_type == MSG_FAST_DATA and self.collect:
                if len(payload) < 4:
                    self.malformed += 1
                    continue
                seq, = struct.unpack_from("<H", payload)
                n, config_id = payload[3], payload[2]
                needed = 4 + n * len(self.channels) * 2
                if config_id != FAST_CONFIG_ID or n == 0 or len(payload) < needed:
                    self.malformed += 1
                    continue
                if self.last_seq is not None:
                    self.dropped += (seq - self.last_seq - 1) & 0xFFFF
                self.last_seq = seq
                self.fast.extend(payload[4:needed])
            elif msg_type == MSG_RESPONSE:
                responses.append(payload)
        return responses

    def request(self, msg_type, operation, body=b"", timeout=2.0):
        self.txn = (self.txn % 255) + 1
        txn = self.txn
        self.ser.write(wire(msg_type, bytes([txn, operation]) + body))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for payload in self.poll():
                if len(payload) < 4:
                    continue
                if payload[0] != txn or payload[1] != msg_type or payload[2] != operation:
                    continue
                if payload[3] != 0:
                    raise RuntimeError(f"request {msg_type:02X}/{operation:02X} status {payload[3]}")
                return payload[4:]
        raise TimeoutError(f"request {msg_type:02X}/{operation:02X} timed out")

    def read_u8(self, pid):
        payload = self.request(MSG_PARAMETER, PARAM_READ, struct.pack("<H", pid))
        if len(payload) < 4 or struct.unpack_from("<H", payload)[0] != pid or payload[2] != 0:
            raise RuntimeError(f"unexpected value reply for 0x{pid:04X}")
        return payload[3]

    def read_float(self, pid):
        payload = self.request(MSG_PARAMETER, PARAM_READ, struct.pack("<H", pid))
        if len(payload) < 7 or struct.unpack_from("<H", payload)[0] != pid or payload[2] != PARAM_FLOAT:
            raise RuntimeError(f"unexpected float reply for 0x{pid:04X}")
        return struct.unpack_from("<f", payload, 3)[0]

    def set_u8(self, pid, value):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HBB", pid, 0, value))

    def set_float(self, pid, value):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HBf", pid, PARAM_FLOAT, value))

    def action(self, pid):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HB", pid, PARAM_ACTION))

    def plot_start(self, channels):
        self.channels = channels
        vars_payload = bytes([0, FAST_CONFIG_ID, len(channels)])
        vars_payload += b"".join(struct.pack("<H", pid) for _, pid, _ in channels)
        self.request(MSG_PLOT, PLOT_CONFIG, vars_payload)
        self.fast.clear()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = True
        self.request(MSG_PLOT, PLOT_START, b"\x01")

    def plot_stop(self):
        self.request(MSG_PLOT, PLOT_STOP, b"\x01")
        self.collect = False


def sinusoid_fit(signal, freq, times):
    # Include DC as a separate fit parameter to avoid leakage from offsets.
    w = 2.0 * math.pi * freq
    basis = np.column_stack((np.sin(w * times), np.cos(w * times), np.ones(len(times))))
    coef = np.linalg.lstsq(basis, signal, rcond=None)[0]
    residue = signal - basis @ coef
    # Phasor in cosine convention; ratios do not depend on convention.
    return (complex(coef[1], -coef[0]),
            float(np.sqrt(np.mean(residue ** 2))), float(coef[2]))


def analyze(raw, freq, amp, settle_cycles, channels):
    """Fit the injection and same-clock reference/feedback/output phasors."""
    data = np.frombuffer(bytes(raw), dtype="<i2")
    if len(data) % len(channels) != 0:
        raise RuntimeError("FAST sample buffer is not whole records")
    raw_matrix = data.reshape(-1, len(channels))
    samples = raw_matrix.astype(np.float64) * np.array([c[2] for c in channels])
    inj = samples[:, 0]
    active = np.flatnonzero(np.abs(inj) > max(channels[0][2] * 2, amp * 0.02))
    if len(active) < 50:
        raise RuntimeError("sine excitation missing from FAST Plot")
    start = int(active[0]) + int(round(settle_cycles * FAST_RATE_HZ / freq))
    end = int(active[-1]) + 1
    if end - start < max(100, int(round(2.0 * FAST_RATE_HZ / freq))):
        raise RuntimeError("too few settled cycles; increase --duration")
    interval = samples[start:end]
    t = np.arange(len(interval), dtype=np.float64) / FAST_RATE_HZ
    ref, ref_err, ref_dc = sinusoid_fit(interval[:, 1], freq, t)
    fbk, fbk_err, fbk_dc = sinusoid_fit(interval[:, 2], freq, t)
    out, out_err, out_dc = sinusoid_fit(interval[:, 3], freq, t)
    if abs(ref) < max(channels[1][2] * 5, amp * 0.2):
        raise RuntimeError("measured reference amplitude too small")
    frf = responses(ref, fbk, out)
    drift = []
    split = len(interval) // 2
    for col, phasor in enumerate((ref, fbk, out), 1):
        first = sinusoid_fit(interval[:split, col], freq, t[:split])[0]
        second = sinusoid_fit(interval[split:, col], freq, t[split:])[0]
        drift.append(abs(first - second) / max(abs(phasor), channels[col][2]))
    return {
        "freq": freq,
        "ref": ref,
        "fbk": fbk,
        "out": out,
        "transfer": frf,
        "ref_amplitude": abs(ref),
        "fit_residual": (ref_err, fbk_err, out_err),
        "fit_drift": tuple(drift),
        "mean": (ref_dc, fbk_dc, out_dc),
        "samples": len(interval),
    }


def sweep_quality(results, linear_verified=False):
    """Reject poor fits; a passed fit does not establish physical linearity.

    The 20% half-record phasor drift is a per-point interpolation guard.
    Large off-tone feedback (>3x reference peak) invalidates the whole sweep,
    without claiming to distinguish noise, oscillation, or nonlinearity.
    """
    valid = np.array([max(r["fit_drift"][:2]) <= .2 for r in results], dtype=bool)
    for r in results:
        if r["fit_residual"][0] > .1 * abs(r["ref"]):
            return "invalid", f"poor reference fit at {r['freq']:g} Hz", valid
        if r["fit_residual"][1] > 3 * abs(r["ref"]):
            return "invalid", f"large off-tone feedback at {r['freq']:g} Hz", valid
    if not linear_verified:
        return "unknown", "linear operating conditions not verified", valid
    return "valid", "linear conditions confirmed by operator; fit guards applied", valid


def log_sweep(start, stop, points_per_decade):
    """Rounded integer-Hz log grid, always including both endpoints."""
    if start < 1 or stop <= start or points_per_decade < 1:
        raise ValueError("log sweep requires 0 < start < stop and points-per-decade >= 1")
    intervals = max(1, math.ceil(points_per_decade * math.log10(stop / start)))
    freqs = np.rint(np.logspace(math.log10(start), math.log10(stop),
                               intervals + 1)).astype(np.int64)
    freqs[0], freqs[-1] = start, stop
    return np.unique(freqs).astype(float).tolist()


def main():
    p = argparse.ArgumentParser(description="AxDr sensored Id/Iq/Speed FRF")
    p.add_argument("--port", help="USB CDC port, e.g. /dev/ttyACM0 or COM5")
    p.add_argument("--result", type=Path, help="replot a *.bode.nmixx archive without hardware")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--loop", choices=tuple(LOOPS), default="id")
    p.add_argument("--freq", type=float, nargs="+",
                   help="manual frequency points in Hz (alternative to --start/--stop)")
    p.add_argument("--start", type=int, help="log sweep start frequency, integer Hz")
    p.add_argument("--stop", type=int, help="log sweep end frequency, integer Hz")
    p.add_argument("--points-per-decade", type=int, default=10,
                   help="log sweep density before integer rounding (default: 10)")
    p.add_argument("--amp", type=float, default=0.1,
                   help="peak excitation: A for Id/Iq, mechanical rad/s for Speed")
    p.add_argument("--duration", type=float, default=2.0,
                   help="minimum seconds per point (auto extended for low frequency)")
    p.add_argument("--settle-cycles", type=float, default=4.0)
    p.add_argument("--out", type=Path, default=Path("test-data/current_frf"))
    p.add_argument("--plot", action="store_true", help="write A4 bode.png")
    p.add_argument("--linear-verified", action="store_true",
                   help="capture only: operator confirms linear, unsaturated, non-oscillatory conditions; fit checks still apply")
    args = p.parse_args()
    if args.result is not None:
        if (args.port or args.freq is not None or args.start is not None or
            args.stop is not None or args.linear_verified):
            p.error("--result is offline; do not pass capture options or override archived quality")
        metrics = replot(args.result, args.out)
        for name, values in metrics.items():
            summary = ", ".join(f"{label}={values[key] if values[key] is not None else 'N/A'}"
                                for label, key in (("-3dB", "relative_minus_3db_hz"),
                                                   ("PM", "phase_margin_deg"),
                                                   ("GM", "gain_margin_db")))
            print(f"{name}: {summary}")
            for key in ("bandwidth_reason", "phase_margin_reason", "gain_margin_reason"):
                if values[key]:
                    print(f"  {key}: {values[key]}")
        return

    if not args.port:
        p.error("hardware capture requires --port")
    max_freq = 999 if args.loop == "speed" else 9999
    if not 1 <= args.points_per_decade <= 1000:
        p.error("--points-per-decade must be between 1 and 1000")
    if args.freq is not None:
        if args.start is not None or args.stop is not None:
            p.error("--freq cannot be combined with --start/--stop")
        freqs = args.freq
    else:
        if args.start is None or args.stop is None:
            p.error("specify --start and --stop, or use --freq for manual points")
        if not 1 <= args.start < args.stop <= max_freq:
            p.error(f"log sweep requires 1 <= start < stop <= {max_freq} Hz")
        freqs = log_sweep(args.start, args.stop, args.points_per_decade)

    if (len(freqs) < 2 or len(set(freqs)) != len(freqs) or
        any(not math.isfinite(f) or f <= 0 or f > max_freq for f in freqs) or
        not math.isfinite(args.amp) or args.amp <= 0 or
        not math.isfinite(args.duration) or not 0 < args.duration <= 60 or
        not math.isfinite(args.settle_cycles) or args.settle_cycles < 0):
        p.error("invalid frequency points, amplitude, duration or settle cycles")
    print(f"{len(freqs)} sweep points (Hz): " +
          ", ".join(f"{f:g}" for f in freqs))

    # The type is part of the compound suffix, not the entire filename.
    stem = ("Current-D", "Current-Q", "Speed")[LOOPS[args.loop]]
    data_path = args.out / f"{stem}-{date.today().isoformat()}.bode.nmixx"
    args.out.mkdir(parents=True, exist_ok=True)
    if data_path.exists():
        p.error(f"archive already exists; use another --out directory: {data_path}")
    client = Client(args.port, args.baud)
    results = []
    plotting = False
    channels = CHANNELS[args.loop]
    loop_id = LOOPS[args.loop]
    try:
        expected_mode = 1 if args.loop == "speed" else 0
        state = client.read_u8(PARAM_MOTOR_STATE)
        if (client.read_u8(PARAM_MOTOR_MODE) != expected_mode or
            (state not in (1, 2) if args.loop == "speed" else state != 1)):
            raise RuntimeError("set TORQUE/ENABLED for Id/Iq or SPEED/ENABLED/RUN for speed")
        pole_pairs = None
        client.set_u8(PARAM_SIGNAL_TARGET, loop_id)
        if client.read_u8(PARAM_SIGNAL_TARGET) != loop_id:
            raise RuntimeError("FRF target readback mismatch")
        ff = client.read_u8(PARAM_CTRL_CURRENT_FF_ENABLE)
        if args.loop == "speed":
            pole_pairs = client.read_u8(PARAM_MOTOR_PP)
            if pole_pairs < 1:
                raise RuntimeError("invalid motor pole pairs")
            if state == 2:
                target = client.read_float(PARAM_TARGET_SPEED)
                work_ref = client.read_float(PARAM_REF_WM)
                work_fbk = client.read_float(PARAM_RUN_WM)
                print(f"SPEED RUN workpoint (user coordinates): target={target:.4g}, "
                      f"ref={work_ref:.4g}, fbk={work_fbk:.4g} rad/s")
                print("The motor must have settled at the requested speed before sweeping.")
            bw = client.read_float(PARAM_CTRL_SPEED_BW_HZ)
            tune_source = client.read_u8(PARAM_CTRL_SPEED_SOURCE)
            kp = client.read_float(PARAM_CTRL_SPEED_KP_EFFECTIVE)
            ki = client.read_float(PARAM_CTRL_SPEED_KI_EFFECTIVE)
            amp_param = PARAM_SIGNAL_SPEED_AMP
            current_limit_a = client.read_float(PARAM_LIMIT_I_EFFECTIVE)
        else:
            bw = client.read_float(PARAM_CTRL_CURRENT_BW_HZ)
            tune_source = client.read_u8(PARAM_CTRL_CURRENT_SOURCE)
            if tune_source == 1:
                kp_pid = PARAM_CTRL_ID_KP if args.loop == "id" else PARAM_CTRL_IQ_KP
                ki_pid = PARAM_CTRL_ID_KI if args.loop == "id" else PARAM_CTRL_IQ_KI
                kp = client.read_float(kp_pid)
                ki = client.read_float(ki_pid)
            else:
                ldq_pid = PARAM_MOTOR_LD if args.loop == "id" else PARAM_MOTOR_LQ
                kp = client.read_float(ldq_pid) * 2 * math.pi * bw
                ki = client.read_float(PARAM_MOTOR_RS) * 2 * math.pi * bw
            amp_param = PARAM_SIGNAL_AMP_A

        for i, freq in enumerate(freqs):
            if client.read_u8(PARAM_MOTOR_STATE) != state:
                raise RuntimeError("motor changed state during sweep")
            duration = max(args.duration, (args.settle_cycles + 3.0) / freq)
            if duration > 60:
                raise RuntimeError("frequency requires more than 60 s per point")
            client.set_float(PARAM_SIGNAL_FREQ_HZ, freq)
            client.set_float(amp_param, args.amp)
            client.set_float(PARAM_SIGNAL_TIME_S, duration)
            client.plot_start(channels)
            plotting = True
            try:
                client.action(ACTION_SIGNAL_START)
                end = time.monotonic() + duration + 0.25
                while time.monotonic() < end:
                    client.poll()
                client.action(ACTION_SIGNAL_ABORT)
                client.plot_stop()
                plotting = False
            finally:
                client.action(ACTION_SIGNAL_ABORT)
            if client.dropped or client.malformed:
                raise RuntimeError(f"FAST data incomplete: seq_lost={client.dropped}, "
                                   f"malformed={client.malformed}")
            raw_values = np.frombuffer(bytes(client.fast), dtype="<i2").reshape(-1, len(channels))
            if np.any(np.abs(raw_values.astype(np.int32)) >= 32767):
                raise RuntimeError(f"{freq:g} Hz: FAST int16 Plot data clipped")
            if client.read_u8(PARAM_MOTOR_STATE) != state:
                raise RuntimeError("motor changed state during sweep")
            result = analyze(client.fast, freq, args.amp, args.settle_cycles, channels)
            if args.loop == "speed":
                iq_samples = np.frombuffer(bytes(client.fast), dtype="<i2").reshape(-1, 4)[:, 3]
                if np.max(np.abs(iq_samples.astype(float) * channels[3][2])) >= 0.98 * current_limit_a:
                    raise RuntimeError(f"{freq:g} Hz: speed PI Iq command approached current limit")
            raw_path = args.out / f"point_{i:03d}.raw"
            raw_path.write_bytes(client.fast)
            results.append(result)
            T = result["transfer"]["T"]
            print(f"{freq:g} Hz: gain={20*np.log10(abs(T)):+.2f} dB, "
                  f"phase={np.angle(T, deg=True):+.1f} deg; samples={result['samples']}")

        freqs = np.array([r["freq"] for r in results])
        ref = np.array([r["ref"] for r in results])
        fbk = np.array([r["fbk"] for r in results])
        out = np.array([r["out"] for r in results])
        workpoint = None
        if args.loop == "speed":
            means = np.array([r["mean"] for r in results], dtype=np.float64)
            workpoint = {
                "wm_ref_mean_rad_s": means[:, 0] / pole_pairs,
                "wm_fbk_mean_rad_s": means[:, 1] / pole_pairs,
                "iq_ref_mean_a": means[:, 2],
            }
        quality, quality_reason, point_valid = sweep_quality(results, args.linear_verified)
        save_bode(data_path, freqs, ref, fbk, out,
                  loop_id, bw, ff, tune_source, kp, ki,
                  workpoint=workpoint, pole_pairs=pole_pairs, quality=quality,
                  quality_reason=quality_reason, point_valid=point_valid,
                  fit_residual=[r["fit_residual"] for r in results],
                  fit_drift=[r["fit_drift"] for r in results])

        if len(results) >= 2:
            T = fbk / ref
            S = 1 - T
            L = np.divide(T, S, out=np.full(T.shape, complex(np.nan, np.nan)),
                          where=abs(S) >= 1e-10)
            if tune_source == 1:
                title = f"{NAMES[loop_id]} (Manual Kp {kp:.4g}, Ki {ki:.4g})"
            else:
                title = f"{NAMES[loop_id]} (PI setting {bw:g} Hz)"
            if args.loop != "speed":
                title += f", FF {ff}"
            if args.plot:
                metrics = plot_bode_a4(freqs, L, S, T, title, args.out / "bode.png",
                                       quality=quality, quality_reason=quality_reason,
                                       point_valid=point_valid)
            else:
                metrics = bode_metrics(freqs, L, T, quality=quality,
                                       quality_reason=quality_reason, point_valid=point_valid)
            print(", ".join(f"{label}={metrics[key] if metrics[key] is not None else 'N/A'}"
                            for label, key in (("-3dB", "relative_minus_3db_hz"),
                                               ("PM", "phase_margin_deg"),
                                               ("GM", "gain_margin_db"))))
            print(f"Quality: {quality}; {quality_reason}")
            for key in ("bandwidth_reason", "phase_margin_reason", "gain_margin_reason"):
                if metrics[key]:
                    print(f"  {key}: {metrics[key]}")
        print("Results:", data_path)
    finally:
        try:
            client.action(ACTION_SIGNAL_ABORT)
        except (OSError, RuntimeError, TimeoutError):
            pass
        if plotting:
            try:
                client.plot_stop()
            except (OSError, RuntimeError, TimeoutError):
                pass
        client.close()


if __name__ == "__main__":
    main()
