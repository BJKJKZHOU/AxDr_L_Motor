#!/usr/bin/env python3
"""Diagnose Flux VF->Observer handover from identification state CSV.

This tool does not change firmware behavior. It checks whether the speed-side
handover conditions were ever satisfied for a continuous window and highlights
large observer-speed discontinuities that may indicate observer/PLL resets.
"""

import argparse
import csv
import math
import statistics


def load_rows(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["time_s"])
                we_ctrl = float(row["We_ctrl_rad_s"])
                we_obs = float(row["We_obs_rad_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(t) and math.isfinite(we_ctrl) and math.isfinite(we_obs)):
                continue
            rows.append((t, we_ctrl, we_obs))
    return rows


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv", help="flux_*_state.csv")
    p.add_argument("--blank-s", type=float, default=6.0)
    p.add_argument("--window-s", type=float, default=0.05)
    p.add_argument("--mean-ratio", type=float, default=0.06)
    p.add_argument("--rms-ratio", type=float, default=0.10)
    p.add_argument("--jump-rad-s", type=float, default=20.0,
                   help="observer-speed jump reported as reset suspect")
    args = p.parse_args()

    rows = [r for r in load_rows(args.csv) if r[0] >= args.blank_s]
    if len(rows) < 2:
        raise SystemExit("not enough samples after blanking interval")

    dts = [rows[i][0] - rows[i - 1][0] for i in range(1, len(rows))
           if rows[i][0] > rows[i - 1][0]]
    dt = statistics.median(dts)
    nwin = max(1, int(round(args.window_s / dt)))

    abs_err = [abs(obs - ctrl) for _, ctrl, obs in rows]
    signed_ok = [ctrl * obs > 0.0 for _, ctrl, obs in rows]
    scale = [max(abs(ctrl), 1e-6) for _, ctrl, _ in rows]

    best = None
    qualifying = []
    for end in range(nwin - 1, len(rows)):
        start = end - nwin + 1
        errs = [rows[i][2] - rows[i][1] for i in range(start, end + 1)]
        ctrl_scale = statistics.mean(abs(rows[i][1]) for i in range(start, end + 1))
        if ctrl_scale <= 1e-6:
            continue
        mean_err = abs(statistics.mean(errs))
        rms_err = rms(errs)
        direction_ok = all(rows[i][1] * rows[i][2] > 0.0 for i in range(start, end + 1))
        mean_ratio = mean_err / ctrl_scale
        rms_ratio = rms_err / ctrl_scale
        score = max(mean_ratio / args.mean_ratio, rms_ratio / args.rms_ratio)
        item = (score, rows[start][0], rows[end][0], mean_err, rms_err,
                mean_ratio, rms_ratio, direction_ok)
        if best is None or score < best[0]:
            best = item
        if direction_ok and mean_ratio <= args.mean_ratio and rms_ratio <= args.rms_ratio:
            qualifying.append(item)

    jumps = []
    for i in range(1, len(rows)):
        jump = rows[i][2] - rows[i - 1][2]
        if abs(jump) >= args.jump_rad_s:
            jumps.append((rows[i][0], jump, rows[i - 1][2], rows[i][2]))

    print(f"samples after {args.blank_s:.3f}s: {len(rows)}")
    print(f"median state sample period: {dt * 1000.0:.3f} ms")
    print(f"handover window: {nwin} samples ~= {nwin * dt * 1000.0:.1f} ms")
    print(f"overall |We_obs-We_ctrl|: mean={statistics.mean(abs_err):.3f} rad/s, "
          f"max={max(abs_err):.3f} rad/s")
    print(f"direction agreement: {100.0 * sum(signed_ok) / len(signed_ok):.2f}%")

    if best is not None:
        _, t0, t1, mean_err, rms_err, mean_ratio, rms_ratio, direction_ok = best
        print("best speed-side window:")
        print(f"  t={t0:.3f}..{t1:.3f}s direction={'PASS' if direction_ok else 'FAIL'}")
        print(f"  mean error={mean_err:.3f} rad/s ({100.0 * mean_ratio:.2f}%) "
              f"limit={100.0 * args.mean_ratio:.2f}%")
        print(f"  rms error ={rms_err:.3f} rad/s ({100.0 * rms_ratio:.2f}%) "
              f"limit={100.0 * args.rms_ratio:.2f}%")

    if qualifying:
        first = qualifying[0]
        print(f"speed-side handover qualification: PASS, first window "
              f"{first[1]:.3f}..{first[2]:.3f}s")
        print("If firmware still never hands over, inspect PLL/theta qualification or observer resets.")
    else:
        print("speed-side handover qualification: FAIL")

    print(f"observer-speed reset suspects (|delta We_obs| >= {args.jump_rad_s:.1f} rad/s): {len(jumps)}")
    for t, jump, before, after in jumps[:20]:
        print(f"  t={t:.3f}s jump={jump:+.3f} rad/s ({before:.3f} -> {after:.3f})")
    if len(jumps) > 20:
        print(f"  ... {len(jumps) - 20} more")


if __name__ == "__main__":
    main()
