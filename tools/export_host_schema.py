#!/usr/bin/env python3
"""Export the host-visible Parameter contract as deterministic TOML.

Parameter/parameter.yaml remains the hand-maintained Host contract. Compile-time
motor defaults are resolved from User/motor_params.h so the Host can present the
same immutable defaults without duplicating their numeric values.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys

from gen_parameters import ROOT, SOURCE, actions, host_readback_symbols, load_objects, values

DEFAULT_OUTPUT = ROOT / "build" / "host" / "axdr-host-schema.toml"
SCHEMA_VERSION = 1
MOTOR_DEFAULT_HEADER = ROOT / "User" / "motor_params.h"
MOTOR_DEFAULT_MACROS = {
    "PARAM_MOTOR_PP": "MOTOR_PP_DEFAULT",
    "PARAM_MOTOR_RS": "MOTOR_RS_DEFAULT",
    "PARAM_MOTOR_LD": "MOTOR_LD_DEFAULT",
    "PARAM_MOTOR_LQ": "MOTOR_LQ_DEFAULT",
    "PARAM_MOTOR_FLUX": "MOTOR_FLUX_DEFAULT",
    "PARAM_MOTOR_J": "MOTOR_J_DEFAULT",
    "PARAM_MOTOR_B": "MOTOR_B_DEFAULT",
}

HOST_VALUE_FIELDS = (
    "label",
    "unit",
    "description",
    "persistent",
    "write_state",
    "range",
    "allowed",
    "allowed_symbols",
)
HOST_ACTION_FIELDS = ("label", "description")


def toml_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def toml_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return toml_string(value)
    if isinstance(value, (int, float)):
        return repr(value)
    raise TypeError(f"unsupported TOML scalar: {value!r}")


def toml_array(values_):
    return "[" + ", ".join(toml_scalar(value) for value in values_) + "]"


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def motor_default_values() -> dict[str, int | float]:
    text = MOTOR_DEFAULT_HEADER.read_text(encoding="utf-8")
    result: dict[str, int | float] = {}

    for parameter, macro in MOTOR_DEFAULT_MACROS.items():
        match = re.search(rf"^\s*#define\s+{re.escape(macro)}\s+([^\s/]+)", text, re.MULTILINE)
        if match is None:
            raise ValueError(f"missing motor default macro {macro}")

        token = match.group(1).rstrip("fFuUlL")
        try:
            value: int | float
            if any(marker in token.lower() for marker in (".", "e")):
                value = float(token)
            else:
                value = int(token, 0)
        except ValueError as exc:
            raise ValueError(f"unsupported motor default {macro}={match.group(1)}") from exc
        result[parameter] = value

    return result


def render_range(lines: list[str], prefix: str, rng: dict):
    lines.append(f"[{prefix}.range]")
    for key in ("min", "max", "exclusive_min", "exclusive_max"):
        if key in rng:
            lines.append(f"{key} = {toml_scalar(rng[key])}")
    # Firmware-derived limits are useful metadata, but are symbolic rather than
    # numeric Host validation constraints.
    if "max_symbol" in rng:
        lines.append(f"max_symbol = {toml_string(rng['max_symbol'])}")
    if "max_binding" in rng:
        lines.append(f"max_binding = {toml_string(rng['max_binding'])}")
    if "max_bindings" in rng:
        lines.append(f"max_bindings = {toml_array(rng['max_bindings'])}")
    lines.append("")


def render_host_schema(objects) -> str:
    defaults = motor_default_values()
    lines = [
        "# AUTO-GENERATED FILE. DO NOT EDIT.",
        "# Source: Parameter/parameter.yaml + User/motor_params.h defaults",
        "# Regenerate with: python3 tools/export_host_schema.py",
        "",
        f"schema_version = {SCHEMA_VERSION}",
        'protocol = "axdr-canfd-v1"',
        "",
        "[source]",
        'repository = "AxDr_L_Motor"',
        f"git_sha = {toml_string(git_sha())}",
        f"parameter_schema = {SCHEMA_VERSION}",
        "",
    ]

    for symbol, obj in values(objects):
        lines += [
            "[[parameters]]",
            f"symbol = {toml_string(symbol)}",
            f"id = {obj['id']}",
            f"type = {toml_string(obj['type'])}",
            f"access = {toml_string(obj['access'])}",
        ]
        if symbol in defaults:
            lines.append(f"default = {toml_scalar(defaults[symbol])}")
        for field in HOST_VALUE_FIELDS:
            if field not in obj or field == "range":
                continue
            value = obj[field]
            if isinstance(value, list):
                lines.append(f"{field} = {toml_array(value)}")
            else:
                lines.append(f"{field} = {toml_scalar(value)}")

        readback = host_readback_symbols(objects, symbol)
        if readback:
            lines.append(f"readback = {toml_array(readback)}")

        # Runtime PLOT_CAPS is authoritative. Keep exporting FAST scale for
        # current HostSchema consumers that still expose this metadata.
        plot = obj.get("plot")
        if isinstance(plot, dict) and "fast" in plot.get("modes", []):
            lines.append(f"plot_scale = {toml_scalar(plot['fast_scale'])}")

        lines.append("")
        if "range" in obj:
            render_range(lines, "parameters", obj["range"])

    for symbol, obj in actions(objects):
        lines += [
            "[[actions]]",
            f"symbol = {toml_string(symbol)}",
            f"id = {obj['id']}",
        ]
        for field in HOST_ACTION_FIELDS:
            if field in obj:
                lines.append(f"{field} = {toml_scalar(obj[field])}")
        lines.append("")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    try:
        content = render_host_schema(load_objects())
    except Exception as exc:
        print(f"host schema export failed: {exc}", file=sys.stderr)
        return 2

    output = args.output if args.output.is_absolute() else ROOT / args.output
    if args.check:
        if not output.exists() or output.read_text(encoding="utf-8") != content:
            print(f"host schema is out of date: {output.relative_to(ROOT)}", file=sys.stderr)
            return 1
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding="utf-8")
    print(output.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
