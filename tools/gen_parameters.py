#!/usr/bin/env python3
"""Generate AxDr_L parameter IDs and code from Parameter/parameter.yaml."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML is required: python -m pip install PyYAML") from exc

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Parameter" / "parameter.yaml"
OUTPUTS = {
    ROOT / "Parameter" / "Parameter.generated.h": "c_header",
    ROOT / "Parameter" / "Parameter.generated.inc": "c_inc",
    ROOT / "tools" / "parameter_ids_generated.py": "py_ids",
}

TYPE_C = {
    "u8": "PARAM_U8",
    "i8": "PARAM_I8",
    "f32": "PARAM_FLOAT",
    "i32": "PARAM_I32",
    "u32": "PARAM_U32",
    "position": "PARAM_POSITION",
}
VALUE_MEMBER_C = {"u8": "U8", "i8": "I8", "f32": "F32", "i32": "I32", "u32": "U32"}
CAST_C = {"u8": "uint8_t", "i8": "int8_t", "f32": "float", "i32": "int32_t", "u32": "uint32_t"}
ON_CHANGE_C = {
    "MOTOR_PARA": "Motor_Para_Update();",
    "MOTOR_PP": "Motor_Pp_Changed();",
    "CONTROL_TUNING": "Motor_Para_Update();",
    "ENCODER_CONFIG": "Encoder_Config_Changed();",
}


def load_objects():
    data = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError("unsupported parameter schema")
    objects = data.get("objects")
    if not isinstance(objects, dict) or not objects:
        raise ValueError("objects must be a non-empty mapping")

    seen_ids = {}
    for name, obj in objects.items():
        if not isinstance(obj, dict):
            raise ValueError(f"{name}: object must be a mapping")
        for field in ("id", "type", "access", "label", "description"):
            if field not in obj:
                raise ValueError(f"{name}: missing {field}")
        if not isinstance(obj["label"], str) or not obj["label"].strip():
            raise ValueError(f"{name}: label must be a non-empty string")
        object_id = obj["id"]
        if not isinstance(object_id, int) or not 0 <= object_id <= 0xFFFF:
            raise ValueError(f"{name}: id must fit uint16")
        if object_id in seen_ids:
            raise ValueError(f"duplicate id 0x{object_id:04X}: {seen_ids[object_id]} and {name}")
        seen_ids[object_id] = name

        if obj["type"] == "action":
            if not name.startswith("ACTION_"):
                raise ValueError(f"{name}: action name must start with ACTION_")
            if obj["access"] != "wo" or "command" not in obj:
                raise ValueError(f"{name}: action requires access=wo and command")
            continue

        if not name.startswith("PARAM_"):
            raise ValueError(f"{name}: value name must start with PARAM_")
        if obj["type"] not in TYPE_C:
            raise ValueError(f"{name}: unsupported type {obj['type']}")
        if obj["access"] not in ("ro", "rw"):
            raise ValueError(f"{name}: value access must be ro or rw")
        if ("binding" in obj) == ("getter" in obj):
            raise ValueError(f"{name}: exactly one of binding/getter is required")
        if "getter" in obj and obj["access"] != "ro":
            raise ValueError(f"{name}: getter-backed object must be read-only")
        write_state = obj.get("write_state")
        if write_state is not None and write_state not in ("disabled", "not_running"):
            raise ValueError(f"{name}: unsupported write_state {write_state}")
        on_change = obj.get("on_change")
        if on_change is not None and on_change not in ON_CHANGE_C:
            raise ValueError(f"{name}: unsupported on_change {on_change}")
    return objects


def values(objects):
    return ((n, o) for n, o in objects.items() if o["type"] != "action")


def actions(objects):
    return ((n, o) for n, o in objects.items() if o["type"] == "action")


def c_number(value):
    text = f"{float(value):.9g}"
    if "e" not in text.lower() and "." not in text:
        text += ".0"
    return f"{text}f"


def min_expr(obj):
    if "allowed_symbols" in obj:
        return f"(float){obj['allowed_symbols'][0]}"
    if "allowed" in obj:
        return c_number(min(obj["allowed"]))
    rng = obj.get("range", {})
    return c_number(rng["min"]) if "min" in rng else "-INFINITY"


def max_expr(obj):
    if "allowed_symbols" in obj:
        return f"(float){obj['allowed_symbols'][-1]}"
    if "allowed" in obj:
        return c_number(max(obj["allowed"]))
    rng = obj.get("range", {})
    if "max_symbol" in rng:
        return f"(float){rng['max_symbol']}"
    return c_number(rng["max"]) if "max" in rng else "INFINITY"


def flags_expr(obj):
    flags = []
    if obj["access"] == "rw":
        flags.append("PARAM_FLAG_HOST_WRITE")
    if obj.get("write_state") == "disabled":
        flags.append("PARAM_FLAG_DISABLED_ONLY")
    elif obj.get("write_state") == "not_running":
        flags.append("PARAM_FLAG_NOT_RUNNING")
    return " | ".join(flags) if flags else "0U"


def validate_conditions(obj):
    if obj["type"] == "position":
        return []

    member = f"Value.{VALUE_MEMBER_C[obj['type']]}"
    cast = CAST_C[obj["type"]]
    conditions = []

    if "allowed" in obj:
        allowed = [f"({member} != ({cast}){c_number(value)})" for value in obj["allowed"]]
        conditions.append(" && ".join(allowed))

    if "allowed_symbols" in obj:
        allowed = [f"({member} != ({cast}){symbol})" for symbol in obj["allowed_symbols"]]
        conditions.append(" && ".join(allowed))

    rng = obj.get("range", {})
    if rng.get("exclusive_min") and "min" in rng:
        conditions.append(f"(Number <= {c_number(rng['min'])})")
    if rng.get("exclusive_max") and "max" in rng:
        conditions.append(f"(Number >= {c_number(rng['max'])})")
    if "max_binding" in rng:
        conditions.append(f"(Number > (float){rng['max_binding']})")
    for binding in rng.get("max_bindings", []):
        conditions.append(f"(Number > (float){binding})")

    return conditions


def render_header(objects):
    lines = [
        "/* Generated from Parameter/parameter.yaml. DO NOT EDIT. */",
        "#ifndef PARAMETER_GENERATED_H",
        "#define PARAMETER_GENERATED_H",
        "",
        "typedef enum",
        "{",
    ]
    for name, obj in objects.items():
        source = obj["command"] if obj["type"] == "action" else (obj["binding"] if "binding" in obj else f"{obj['getter']}()")
        lines += [f"    /* {source}: {obj['description']} */", f"    {name} = 0x{obj['id']:04X}U,", ""]
    lines += ["} Parameter_Id_e;", "", "#endif /* PARAMETER_GENERATED_H */", ""]
    return "\n".join(lines)


def render_inc(objects):
    lines = ["/* Generated from Parameter/parameter.yaml. DO NOT EDIT. */", ""]

    lines.append("#if defined(PARAM_GENERATE_TABLE)")
    for name, obj in values(objects):
        data = f"&{obj['binding']}" if "binding" in obj else "NULL"
        lines.append("{ " + f"{name}, {TYPE_C[obj['type']]}, {data}, {min_expr(obj)}, {max_expr(obj)}, {flags_expr(obj)} " + "},")

    lines += ["", "#elif defined(PARAM_GENERATE_VALIDATE)"]
    for name, obj in values(objects):
        conditions = validate_conditions(obj)
        if not conditions:
            continue
        lines += [f"case {name}:", "    if (" + " ||\n        ".join(conditions) + ")", "    {", "        return PARAM_ERR_VALUE;", "    }", "    break;", ""]

    lines += ["#elif defined(PARAM_GENERATE_ON_CHANGE)"]
    groups = defaultdict(list)
    for name, obj in values(objects):
        if "on_change" in obj:
            groups[obj["on_change"]].append(name)
    for on_change, names in groups.items():
        for name in names:
            lines.append(f"case {name}:")
        lines += [f"    {ON_CHANGE_C[on_change]}", "    break;", ""]

    lines += ["#elif defined(PARAM_GENERATE_READ)"]
    for name, obj in values(objects):
        if "getter" not in obj:
            continue
        if obj["type"] == "position":
            lines += [f"case {name}:", f"    Value->Position = {obj['getter']}();", "    return PARAM_OK;", ""]
            continue
        member = VALUE_MEMBER_C[obj["type"]]
        cast = CAST_C[obj["type"]]
        lines += [f"case {name}:", f"    Value->{member} = ({cast}){obj['getter']}();", "    return PARAM_OK;", ""]

    lines += ["#elif defined(PARAM_GENERATE_ACTION)"]
    for name, obj in actions(objects):
        lines += [f"case {name}:", f"    Msg.Cmd = (ULONG){obj['command']};", f"    Msg.Arg = (ULONG){obj.get('arg', '0U')};", "    break;", ""]

    lines += ["#elif defined(PARAM_GENERATE_PLOT)"]
    for name, obj in values(objects):
        if "plot_scale" not in obj:
            continue
        if "binding" not in obj or obj["type"] != "f32":
            raise ValueError(f"{name}: Plot requires direct f32 binding")
        lines += [f"case {name}:", f"    *Scale = {c_number(obj['plot_scale'])};", f"    return &{obj['binding']};", ""]

    lines += ["#else", '#error "Parameter.generated.inc section not selected"', "#endif", ""]
    return "\n".join(lines)


def render_py_ids(objects):
    lines = ['"""Generated from Parameter/parameter.yaml. DO NOT EDIT."""', ""]
    lines += [f"{name} = 0x{obj['id']:04X}" for name, obj in objects.items()]
    lines.append("")
    return "\n".join(lines)


def generate(objects):
    renderers = {"c_header": render_header, "c_inc": render_inc, "py_ids": render_py_ids}
    return {path: renderers[kind](objects) for path, kind in OUTPUTS.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        outputs = generate(load_objects())
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"parameter generation failed: {exc}", file=sys.stderr)
        return 2

    stale = []
    for path, content in outputs.items():
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                stale.append(path.relative_to(ROOT))
        else:
            path.write_text(content, encoding="utf-8")
            print(path.relative_to(ROOT))
    if stale:
        print("generated parameter files are out of date:", file=sys.stderr)
        for path in stale:
            print(f"  {path}", file=sys.stderr)
        print("run: python3 tools/gen_parameters.py", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
