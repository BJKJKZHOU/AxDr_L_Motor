"""Frequency-response calculations and landscape-A4 Bode plotting.

Input L/S/T are complex, sampled at the same positive frequency points.
Margins are estimates from measured frequency points, not extrapolations.
"""

from pathlib import Path
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


def bode_arrays(freq_hz, *signals):
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
        if not np.all(np.isfinite(h)):
            raise ValueError("response contains invalid complex numbers")
        gain = 20 * np.log10(np.maximum(np.abs(h), 1e-20))
        phase = np.rad2deg(np.unwrap(np.angle(h)))
        outputs.append((gain, phase))
    return f, outputs


def _crossings(freq, values, target, direction=None):
    """Log-frequency interpolate every measured threshold crossing."""
    out = []
    for i in range(1, len(freq)):
        a, b = values[i - 1] - target, values[i] - target
        if b == a:
            continue
        if direction == "down" and not (a >= 0 > b):
            continue
        if direction == "up" and not (a <= 0 < b):
            continue
        if direction is None and not (a * b <= 0):
            continue
        frac = -a / (b - a)
        if 0 <= frac <= 1:
            log_f = np.log(freq[i - 1]) + frac * (np.log(freq[i]) - np.log(freq[i - 1]))
            out.append((float(np.exp(log_f)), i - 1, float(frac)))
    return out


def bode_metrics(freq_hz, open_loop, closed_loop, low_frequency_points=3):
    """Report only margins/bandwidth with an observed crossing in the sweep."""
    f, ((l_db, l_phase), (t_db, _)) = bode_arrays(freq_hz, open_loop, closed_loop)
    low_db = float(np.mean(t_db[:min(low_frequency_points, len(t_db))]))
    threshold = low_db - 3.0
    cutoffs = _crossings(f, t_db, threshold, direction="down")
    unity = _crossings(f, l_db, 0.0, direction="down")
    phase_crossings = _crossings(f, l_phase, -180.0, direction="down")
    phase_margin = [(180.0 + float(l_phase[i] + frac * (l_phase[i + 1] - l_phase[i])), freq)
                    for freq, i, frac in unity]
    gain_margin = [(-float(l_db[i] + frac * (l_db[i + 1] - l_db[i])), freq)
                   for freq, i, frac in phase_crossings]
    pm = min(phase_margin, key=lambda item: item[0]) if phase_margin else None
    gm = min(gain_margin, key=lambda item: item[0]) if gain_margin else None
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
    }


def plot_bode_a4(freq_hz, open_loop, sensitivity, closed_loop, title, destination):
    """Render L/S/T magnitude + phase on an A4 landscape canvas (PNG + SVG)."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import LogLocator, NullFormatter, MultipleLocator

    f, spectra = bode_arrays(freq_hz, open_loop, sensitivity, closed_loop)
    metrics = bode_metrics(freq_hz, open_loop, closed_loop)
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax_mag, ax_phase) = plt.subplots(
        2, 1, sharex=True, figsize=(11.69, 8.27), dpi=180
    )
    labels = ("Open Loop L", "Sensitivity S", "Closed Loop T")
    colors = ("#367EBC", "#D86B43", "#DCA832")
    for (gain, phase), label, color in zip(spectra, labels, colors):
        ax_mag.semilogx(f, gain, label=label, lw=1.25, color=color)
        ax_phase.semilogx(f, phase, label=label, lw=1.25, color=color)

    for axis in (ax_mag, ax_phase):
        axis.set_xscale("log")
        axis.set_xlim(f[0], f[-1])
        axis.xaxis.set_major_locator(LogLocator(base=10, numticks=15))
        axis.xaxis.set_minor_locator(LogLocator(base=10, subs=np.arange(2, 10), numticks=100))
        axis.xaxis.set_minor_formatter(NullFormatter())
        axis.grid(which="major", color="0.6", lw=0.60, alpha=0.40)
        axis.grid(which="minor", color="0.6", ls=":", lw=0.45, alpha=0.45)
        axis.tick_params(which="major", direction="in", top=True, right=True, length=6)
        axis.tick_params(which="minor", direction="in", top=True, right=True, length=3)

    ax_mag.yaxis.set_major_locator(MultipleLocator(10))
    ax_mag.yaxis.set_minor_locator(MultipleLocator(5))
    ax_phase.yaxis.set_major_locator(MultipleLocator(45))
    ax_phase.yaxis.set_minor_locator(MultipleLocator(15))
    for axis in (ax_mag, ax_phase):
        axis.grid(which="minor", axis="y", color="0.65", ls=":", lw=0.45, alpha=0.40)

    ax_mag.set_ylabel("Magnitude (dB)")
    ax_phase.set_ylabel("Phase (deg)")
    ax_phase.set_xlabel("Frequency (Hz)")
    ax_mag.legend(loc="lower left", fontsize=9, framealpha=0.92)

    bw = metrics["relative_minus_3db_hz"]
    if bw is not None:
        for axis in (ax_mag, ax_phase):
            axis.axvline(bw, color="#D86B43", ls="--", lw=1.05, alpha=0.9)
        ax_mag.axhline(metrics["relative_minus_3db_threshold_db"],
                       color="0.50", ls="--", lw=0.8, alpha=0.7)
        bw_text = f"relative -3 dB bandwidth {bw:.3g} Hz"
    else:
        bw_text = f"relative -3 dB bandwidth not reached (<= {f[-1]:g} Hz)"

    pm = metrics["phase_margin_deg"]
    gm = metrics["gain_margin_db"]
    fc = metrics["gain_crossover_hz"]
    margin = (f"PM = {pm:.1f} deg" if pm is not None else "PM = N/A")
    margin += "   |   " + (f"GM = {gm:.1f} dB" if gm is not None else "GM = N/A (not measured)")
    margin += "   |   " + (f"fc = {fc:.3g} Hz" if fc is not None else "fc = N/A")

    fig.suptitle(f"{title}, {bw_text}", y=0.980, fontsize=12)
    fig.text(0.5, 0.948, margin, fontsize=10, ha="center", va="top", color="0.35")
    fig.subplots_adjust(left=0.075, right=0.985, bottom=0.087, top=0.895, hspace=0.12)
    fig.savefig(output, dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)
    return metrics
