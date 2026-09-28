#!/usr/bin/env python3
"""Verify Host-only Parameter metadata never changes firmware generation."""

from __future__ import annotations

from copy import deepcopy
import tomllib

from export_host_schema import render_host_schema
from gen_parameters import generate, host_readback_symbols, load_objects


def legacy_related_parameter(written: str, candidate: str) -> bool:
    current = {
        "PARAM_CTRL_CURRENT_BW_HZ", "PARAM_CTRL_CURRENT_SOURCE",
        "PARAM_CTRL_ID_KP", "PARAM_CTRL_ID_KI", "PARAM_CTRL_IQ_KP", "PARAM_CTRL_IQ_KI",
    }
    speed = {
        "PARAM_CTRL_SPEED_BW_HZ", "PARAM_CTRL_SPEED_SOURCE",
        "PARAM_CTRL_SPEED_KP", "PARAM_CTRL_SPEED_KI",
    }
    model = {
        "PARAM_MOTOR_PP", "PARAM_MOTOR_RS", "PARAM_MOTOR_LD", "PARAM_MOTOR_LQ",
        "PARAM_MOTOR_FLUX", "PARAM_MOTOR_J", "PARAM_MOTOR_B",
    }
    return (
        (written in current and candidate in current)
        or (written in speed and candidate in speed)
        or (written in model and (candidate.startswith("PARAM_CTRL_") or candidate.startswith("PARAM_LIMIT_")))
        or (written.startswith("PARAM_LIMIT_") and candidate.startswith("PARAM_LIMIT_"))
        or (written == "PARAM_MOTION_WM_MAX" and candidate == "PARAM_LIMIT_WM_EFFECTIVE")
        or (written == "PARAM_MOTOR_PP" and candidate == "PARAM_CAL_VALID")
        or ((written.startswith("PARAM_ENCODER_") or written == "PARAM_MOTOR_DIR")
            and (candidate.startswith("PARAM_ENCODER_") or candidate == "PARAM_CAL_VALID"
                or candidate == "PARAM_RUN_POSITION" or candidate == "PARAM_RUN_WM"))
        or (written == "PARAM_MOTOR_MODE"
            and (candidate == "PARAM_MOTOR_STATE" or candidate.startswith("PARAM_TARGET_")))
    )


def main() -> int:
    objects = load_objects()

    stripped = deepcopy(objects)
    for obj in stripped.values():
        obj.pop("host", None)
    with_host = generate(objects)
    without_host = generate(stripped)
    if with_host != without_host:
        changed = [str(path) for path in with_host if with_host[path] != without_host[path]]
        raise AssertionError(f"Host metadata changed firmware generation: {changed}")

    readable = {
        name for name, obj in objects.items()
        if obj["type"] != "action" and "r" in obj["access"]
    }
    for written, obj in objects.items():
        if obj["type"] == "action" or obj["access"] != "rw":
            continue
        expected = {
            candidate for candidate in readable
            if candidate != written and legacy_related_parameter(written, candidate)
        }
        actual = set(host_readback_symbols(objects, written))
        if actual != expected:
            raise AssertionError(
                f"{written}: Host readback mismatch\n"
                f"  missing: {sorted(expected - actual)}\n"
                f"  extra:   {sorted(actual - expected)}"
            )

    schema = tomllib.loads(render_host_schema(objects))
    exported = {item["symbol"]: item.get("readback", []) for item in schema["parameters"]}
    for written, obj in objects.items():
        if obj["type"] == "action" or obj["access"] != "rw":
            continue
        expected = host_readback_symbols(objects, written)
        if exported.get(written, []) != expected:
            raise AssertionError(f"{written}: exported HostSchema readback differs from YAML expansion")

    print("PASS: Host readback metadata preserves firmware generation and legacy readback behavior")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
