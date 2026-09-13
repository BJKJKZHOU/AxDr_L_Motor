#!/usr/bin/env python3
"""Evaluate the sampled Flux OBS_WAIT handover gates without touching the MCU.

Threshold defaults match Identification/Flux.c. A SWD row is not an atomic
20 kHz control-loop snapshot, so this diagnoses sampled gate values but cannot
reconstruct every qualification cycle or prove a continuous stable interval.
"""

import argparse
import csv
import math
from pathlib import Path
import struct
import sys


FLUX_OBS_WAIT = 3
RAW_FIELDS = (
    "flux_state",
    "pll_active",
    "if_we",
    "we_obs",
    "we_base",
    "pll_err",
    "handover_we_err_f",
    "handover_we_err2_f",
    "handover_pll_err2_f",
    "handover_theta_err2_f",
    "handover_speed_valid",
    "handover_theta_valid",
)
ATOMIC_GATES = (
    "gate_we_obs_finite",
    "gate_pll_err_finite",
    "gate_direction",
    "gate_speed_valid",
    "gate_we_err_f_finite",
    "gate_we_err2_f_finite",
    "gate_pll_err2_f_finite",
    "gate_we_mean",
    "gate_we_rms",
    "gate_pll_rms",
    "gate_theta_valid",
    "gate_theta_err2_f_finite",
    "gate_theta_rms",
)
GATES = (
    *ATOMIC_GATES,
    "gate_obs_state_stable",
    "gate_handover_speed_stable",
    "gate_handover_source_stable",
    "gate_obs_open_stable",
    "gate_pll_active",
    "gate_ready_input",
)
DETAILS = (
    "we_scale",
    "we_mean_limit",
    "we_rms2_limit",
    "pll_rms2_limit",
    "theta_rms2_limit",
    "failed_gates",
)


def f32(value):
    """Round threshold arithmetic as Cortex-M single-precision operations do."""
    try:
        return struct.unpack("<f", struct.pack("<f", value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


def product(*values):
    result = f32(values[0])
    for value in values[1:]:
        result = f32(result * f32(value))
    return result


def evaluate(row, limits):
    def number(name):
        return float(row[name])

    def flag(name):
        return int(row[name]) != 0

    if_we = number("if_we")
    we_obs = number("we_obs")
    we_base = number("we_base")
    pll_err = number("pll_err")
    we_err_f = number("handover_we_err_f")
    we_err2_f = number("handover_we_err2_f")
    pll_err2_f = number("handover_pll_err2_f")
    theta_err2_f = number("handover_theta_err2_f")

    we_scale = abs(if_we)
    if we_scale < we_base:
        we_scale = we_base
    mean_limit = product(limits.we_mean_ratio, we_scale)
    we_rms2_limit = product(limits.we_rms_ratio, limits.we_rms_ratio,
                            we_scale, we_scale)
    pll_rms2_limit = product(limits.pll_rms_max, limits.pll_rms_max)
    theta_rms2_limit = product(limits.theta_rms_max, limits.theta_rms_max)

    gate = {
        "gate_we_obs_finite": math.isfinite(we_obs),
        "gate_pll_err_finite": math.isfinite(pll_err),
        "gate_direction": product(if_we, we_obs) > 0.0,
        "gate_speed_valid": flag("handover_speed_valid"),
        "gate_we_err_f_finite": math.isfinite(we_err_f),
        "gate_we_err2_f_finite": math.isfinite(we_err2_f),
        "gate_pll_err2_f_finite": math.isfinite(pll_err2_f),
        "gate_we_mean": abs(we_err_f) <= mean_limit,
        "gate_we_rms": we_err2_f <= we_rms2_limit,
        "gate_pll_rms": pll_err2_f <= pll_rms2_limit,
        "gate_theta_valid": flag("handover_theta_valid"),
        "gate_theta_err2_f_finite": math.isfinite(theta_err2_f),
        "gate_theta_rms": theta_err2_f <= theta_rms2_limit,
        "gate_pll_active": flag("pll_active"),
    }
    gate["gate_obs_state_stable"] = (
        gate["gate_we_obs_finite"] and gate["gate_pll_err_finite"])
    gate["gate_handover_speed_stable"] = all(gate[name] for name in (
        "gate_speed_valid", "gate_we_err_f_finite", "gate_we_err2_f_finite",
        "gate_pll_err2_f_finite", "gate_we_mean", "gate_we_rms", "gate_pll_rms"))
    gate["gate_handover_source_stable"] = all(gate[name] for name in (
        "gate_handover_speed_stable", "gate_theta_valid",
        "gate_theta_err2_f_finite", "gate_theta_rms"))
    gate["gate_obs_open_stable"] = all(gate[name] for name in (
        "gate_obs_state_stable", "gate_direction",
        "gate_handover_source_stable"))
    gate["gate_ready_input"] = gate["gate_pll_active"] and gate["gate_obs_open_stable"]
    failed = [name.removeprefix("gate_") for name in ATOMIC_GATES
              if not gate[name]]
    if not gate["gate_pll_active"]:
        failed.append("pll_active")

    return {
        **{name: int(gate[name]) for name in GATES},
        "we_scale": we_scale,
        "we_mean_limit": mean_limit,
        "we_rms2_limit": we_rms2_limit,
        "pll_rms2_limit": pll_rms2_limit,
        "theta_rms2_limit": theta_rms2_limit,
        "failed_gates": ",".join(failed),
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="CSV from ident_swd_trace.py")
    parser.add_argument("-o", "--output", type=Path,
                        help="default: TRACE_gates.csv")
    parser.add_argument("--we-mean-ratio", type=float, default=0.010)
    parser.add_argument("--we-rms-ratio", type=float, default=0.015)
    parser.add_argument("--pll-rms-max", type=float, default=0.08)
    parser.add_argument("--theta-rms-max", type=float, default=0.08)
    args = parser.parse_args()
    for name in ("we_mean_ratio", "we_rms_ratio", "pll_rms_max", "theta_rms_max"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    args.output = args.output or args.trace.with_name(args.trace.stem + "_gates.csv")
    if args.output.resolve() == args.trace.resolve():
        parser.error("output must differ from input trace")
    return args


def main():
    args = parse_args()
    try:
        with args.trace.open(newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)
            missing = set(RAW_FIELDS) - set(reader.fieldnames or ())
            if missing:
                raise ValueError(f"trace lacks raw fields {sorted(missing)}; capture a new trace")

            counts = {name: 0 for name in GATES}
            obs_rows = 0
            total_rows = 0
            with args.output.open("w", newline="", encoding="utf-8") as target:
                writer = csv.DictWriter(target, fieldnames=[*reader.fieldnames, *GATES, *DETAILS])
                writer.writeheader()
                for line, row in enumerate(reader, start=2):
                    try:
                        result = evaluate(row, args)
                    except (ValueError, OverflowError) as exc:
                        raise ValueError(f"line {line}: {exc}") from exc
                    writer.writerow({**row, **result})
                    total_rows += 1
                    if int(row["flux_state"]) == FLUX_OBS_WAIT:
                        obs_rows += 1
                        for name in GATES:
                            counts[name] += not result[name]

        print(f"Wrote {total_rows} rows: {args.output}")
        print("Thresholds used (defaults from Identification/Flux.c): "
              f"mean={args.we_mean_ratio:g}, speed RMS={args.we_rms_ratio:g}, "
              f"PLL RMS={args.pll_rms_max:g}, theta RMS={args.theta_rms_max:g}")
        print(f"FLUX_OBS_WAIT sampled rows: {obs_rows}")
        if obs_rows:
            for name in GATES:
                print(f"  {name}: {counts[name]} failed / {obs_rows}")
        print("SWD fields are sampled sequentially; these gates are diagnostic snapshots, "
              "not a 20 kHz qualification history.")
        return 0
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
