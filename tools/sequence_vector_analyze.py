#!/usr/bin/env python3
"""Analyze negative-sequence voltage-model vectors from sequence_diag JSON.

This is intentionally offline: sequence_diag.py already stores synchronized raw
samples and the extracted complex positive/negative sequence components.  This
tool reuses those results to compare the measured observer flux negative sequence
with the steady-state voltage-model prediction.

For a negative-sequence vector z_-(t) = Z_- exp(-j*theta),

    d/dt z_- = -j*We*z_-

Ignoring the nonlinear observer correction term for the prediction,

    dLambda/dt = U - Rs*I
    Psi = Lambda - Ls*I

therefore

    Lambda_- = (U_- - Rs*I_-) / (-j*We)
    Psi_-    = Lambda_- - Ls*I_-

The difference between predicted and measured Psi_- is useful evidence for the
observer correction term, parameter error, or other unmodelled effects.
"""

import argparse
import cmath
import json
import math
from pathlib import Path


def complex_from_metrics(metrics, prefix="negative"):
    return complex(metrics[f"{prefix}_real"], metrics[f"{prefix}_imag"])


def wrap_deg(angle_rad):
    return math.degrees(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


def vector_row(name, value, unit):
    return (
        f"  {name:18s} mag={abs(value):11.7f} {unit:<3s} "
        f"phase={math.degrees(cmath.phase(value)):8.2f} deg "
        f"re={value.real:+.7f} im={value.imag:+.7f}"
    )


def find_latest(default_dir):
    paths = sorted(default_dir.glob("sequence_diag_*.json"), key=lambda p: p.stat().st_mtime)
    if not paths:
        raise RuntimeError(f"no sequence_diag_*.json found in {default_dir}")
    return paths[-1]


def parse_args():
    parser = argparse.ArgumentParser(description="Offline negative-sequence voltage-model analysis")
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        help="sequence_diag JSON; defaults to newest build/Release/sequence_diag_*.json",
    )
    parser.add_argument(
        "--ls",
        type=float,
        default=None,
        help="override Ls in H; default uses (Ld+Lq)/2 from log",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    path = args.log if args.log is not None else find_latest(repo / "build" / "Release")

    with path.open("r", encoding="utf-8") as handle:
        log = json.load(handle)

    if not log.get("success"):
        raise RuntimeError(f"diagnostic log is not successful: {path}")

    config = log["config"]
    motor = config["motor"]
    sequence = log["sequence"]

    we = float(config["we_if_rad_s"])
    rs = float(motor["rs_ohm"])
    ls = args.ls
    if ls is None:
        ls = 0.5 * (float(motor["ld_h"]) + float(motor["lq_h"]))

    if we <= 0.0:
        raise RuntimeError("We must be positive for this diagnostic")

    i_neg = complex_from_metrics(sequence["current_alpha_beta"])
    u_neg = complex_from_metrics(sequence["voltage_command_alpha_beta"])
    psi_meas = complex_from_metrics(sequence["observer_flux_alpha_beta"])

    ri_neg = rs * i_neg
    emf_neg = u_neg - ri_neg
    lambda_pred = emf_neg / complex(0.0, -we)
    psi_pred = lambda_pred - ls * i_neg
    residual = psi_meas - psi_pred

    mag_error = abs(psi_pred) - abs(psi_meas)
    mag_error_rel = mag_error / abs(psi_meas) if abs(psi_meas) > 1e-15 else math.nan
    phase_error_deg = wrap_deg(cmath.phase(psi_pred) - cmath.phase(psi_meas))
    residual_rel = abs(residual) / abs(psi_meas) if abs(psi_meas) > 1e-15 else math.nan

    print(f"Log: {path}")
    print(f"We={we:.6f} rad/s  Rs={rs:.6f} ohm  Ls={ls * 1e6:.3f} uH")
    print("\nNegative-sequence complex vectors")
    print(vector_row("I_-", i_neg, "A"))
    print(vector_row("Ucmd_-", u_neg, "V"))
    print(vector_row("Rs*I_-", ri_neg, "V"))
    print(vector_row("U_- - Rs*I_-", emf_neg, "V"))
    print(vector_row("Lambda_- pred", lambda_pred, "Wb"))
    print(vector_row("Ls*I_-", ls * i_neg, "Wb"))
    print(vector_row("Psi_- predicted", psi_pred, "Wb"))
    print(vector_row("Psi_- measured", psi_meas, "Wb"))
    print(vector_row("Psi residual", residual, "Wb"))

    print("\nPrediction vs measured Psi_-")
    print(
        f"  magnitude: predicted={abs(psi_pred) * 1e3:.6f} mWb  "
        f"measured={abs(psi_meas) * 1e3:.6f} mWb  "
        f"error={mag_error * 1e3:+.6f} mWb ({100.0 * mag_error_rel:+.2f}%)"
    )
    print(f"  phase error (pred-meas)={phase_error_deg:+.2f} deg")
    print(
        f"  complex residual={abs(residual) * 1e3:.6f} mWb  "
        f"({100.0 * residual_rel:.2f}% of measured Psi_-)"
    )

    result = {
        "source_log": str(path),
        "we_rad_s": we,
        "rs_ohm": rs,
        "ls_h": ls,
        "negative_vectors": {
            "current_a": [i_neg.real, i_neg.imag],
            "voltage_command_v": [u_neg.real, u_neg.imag],
            "rs_current_v": [ri_neg.real, ri_neg.imag],
            "voltage_minus_rs_current_v": [emf_neg.real, emf_neg.imag],
            "lambda_pred_wb": [lambda_pred.real, lambda_pred.imag],
            "ls_current_wb": [(ls * i_neg).real, (ls * i_neg).imag],
            "psi_pred_wb": [psi_pred.real, psi_pred.imag],
            "psi_measured_wb": [psi_meas.real, psi_meas.imag],
            "psi_residual_wb": [residual.real, residual.imag],
        },
        "comparison": {
            "psi_pred_mag_wb": abs(psi_pred),
            "psi_measured_mag_wb": abs(psi_meas),
            "magnitude_error_wb": mag_error,
            "magnitude_error_relative": mag_error_rel,
            "phase_error_deg": phase_error_deg,
            "residual_mag_wb": abs(residual),
            "residual_relative": residual_rel,
        },
    }

    out_path = path.with_name(path.stem + "_vector.json")
    with out_path.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False, allow_nan=False)
    print(f"Vector log: {out_path}")


if __name__ == "__main__":
    main()
