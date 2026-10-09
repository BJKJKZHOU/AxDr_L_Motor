#!/usr/bin/env python3
"""Replot archived AxDr current-FRF point results as landscape-A4 L/S/T Bode.

This reads the original capture summary without rewriting archived measurements.
Open-loop L and sensitivity S are reconstructed from measured complex T, not
independently measured open-loop injection signals.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from bode_plot import plot_bode_a4


def closed_phasor(point):
    h = point["closed_loop_Id_over_IdRef"]
    return 10 ** (h["gain_db"] / 20.0) * np.exp(1j * np.deg2rad(h["phase_deg"]))


def replot(source, output):
    data = json.loads(Path(source).read_text(encoding="utf-8"))
    groups = {}
    for point in data["bode"]:
        groups.setdefault(point["bandwidth_hz"], []).append(point)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    metrics = {}
    for bw, points in sorted(groups.items()):
        points.sort(key=lambda p: p["frequency_hz"])
        freq = np.array([p["frequency_hz"] for p in points], dtype=float)
        T = np.array([closed_phasor(p) for p in points])
        S = 1 - T
        if np.any(abs(S) < 1e-10):
            raise ValueError("loop reconstruction unreliable near unity closed-loop gain")
        L = T / S
        metrics[str(bw)] = plot_bode_a4(
            freq, L, S, T, f"CURRENT Id sweep (PI {bw:g} Hz)",
            output / f"bode_a4_bw_{bw:g}.png"
        )
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    return metrics


def main():
    p = argparse.ArgumentParser(description="Landscape A4 L/S/T plots from archived current FRF")
    p.add_argument("--result", type=Path, required=True, help="archived result.json")
    p.add_argument("--out", type=Path, required=True, help="output directory (does not overwrite source)")
    args = p.parse_args()
    for bw, values in replot(args.result, args.out).items():
        print(f"PI {bw} Hz: -3dB={values['relative_minus_3db_hz']}, PM={values['phase_margin_deg']}, GM={values['gain_margin_db']}")


if __name__ == "__main__":
    main()
