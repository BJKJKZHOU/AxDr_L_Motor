#!/usr/bin/env python3
"""Generate AxDr_L host parameter/action IDs and tables from YAML."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML is required: python -m pip install PyYAML") from exc

ROOT = Path(__file__).resolve().parents[1]
VALUE_SOURCE = ROOT / "Parameter" / "parameter_objects.yaml"
ACTION_SOURCE = ROOT / "Parameter" / "action_objects.yaml"
OUTPUTS = {
    ROOT / "Parameter" / "Parameter_Id.generated.h": "c_ids",
    ROOT / "Parameter" / "Parameter_Table.generated.inc": "c_table",
    ROOT / "Parameter" / "Parameter_Read.generated.inc": "c_read",
    ROOT / "Parameter" / "Parameter_Action.generated.inc": "c_action",
    ROOT / "Parameter" / "Plot_Data.generated.inc": "c_plot",
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

VALUE_MEMBER_C = {
    "u8": "U8",
    "i8": "I8",
    "f32": "F32",
    "i32": "I32",
    "u32": "U32",
}

CAST_C = {
    "u8": "uint8_t",
    "i8": "int8_t",
    "f32": "float",
    "i32": "int32_t",
    "u32": "uint32_t",
}

CHANGE_C = {
    "MOTOR_PARA_RL": "PARAM_CHANGE_MOTOR_RL",
    "MOTOR_PARA_FLUX": "PARAM_CHANGE_MOTOR_FLUX",
    "MOTOR_PARA_JB": "PARAM_CHANGE_MOTOR_JB",
    "MOTOR_PARA_PP": "PARAM_CHANGE_MOTOR_PP",
    "ENCODER_CONFIG": "PARAM_CHANGE_ENCODER",
}


def load_yaml(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError(f"{path.name}: unsupported schema")
    objects = data.get("objects")
    if not isinstance(objects, dict) or not objects:
        raise ValueError(f"{path.name}: objects must be a non-empty mapping")
    return objects


def load_objects():
    values = load_yaml(VALUE_SOURCE)
    actions = load_yaml(ACTION_SOURCE)
    objects = {}
    seen_ids = {}

    for name, obj in values.items():
        if not name.startswith("PARAM_"):
            raise ValueError(f"{name}: value object name must start with PARAM_")
        if not isinstance(obj, dict):
            raise ValueError(f"{name}: object must be a mapping")
        for field in ("id", "type", "access", "description"):
            if field not in obj:
                raise ValueError(f"{name}: missing {field}")
        if obj["type"] not in TYPE_C:
            raise ValueError(f"{name}: unsupported type {obj['type']}")
        if obj["access"] not in ("ro", "rw"):
            raise ValueError(f"{name}: access must be ro or rw")
        if ("binding" in obj) == ("getter" in obj):
            raise ValueError(f"{name}: exactly one of binding/getter is required")
        if "getter" in obj and obj["access"] != "ro":
            raise ValueError(f"{name}: getter-backed object must be read-only")
        if "getter" in obj and obj["type"] == "position":
            raise ValueError(f"{name}: position objects require direct binding")
        obj = dict(obj)
        obj["kind"] = "value"
        objects[name] = obj

    for name, obj in actions.items():
        if not name.startswith("ACTION_"):
            raise ValueError(f"{name}: action object name must start with ACTION_")
        if not isinstance(obj, dict):
            raise ValueError(f"{name}: object must be a mapping")
        for field in ("id", "command", "description"):
            if field not in obj:
                raise ValueError(f"{name}: missing {field}")
        if obj.get("access", "wo") != "wo":
            raise ValueError(f"{name}: action access must be wo")
        obj = dict(obj)
        obj["kind"] = "action"
        obj["type"] = "action"
        obj["access"] = "wo"
        objects[name] = obj

    for name, obj in objects.items():
        object_id = obj["id"]
        if not isinstance(object_id, int) or not 0 <= object_id <= 0xFFFF:
            raise ValueError(f"{name}: id must fit uint16")
        if object_id in seen_ids:
            raise ValueError(
                f"duplicate id 0x{object_id:04X}: {seen_ids[object_id]} and {name}"
            )
        seen_ids[object_id] = name

    return objects


def value_objects(objects):
    return ((name, obj) for name, obj in objects.items() if obj["kind"] == "value")


def action_objects(objects):
    return ((name, obj) for name, obj in objects.items() if obj["kind"] == "action")


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
    range_obj = obj.get("range", {})
    if "min" in range_obj:
        return c_number(range_obj["min"])
    return "-INFINITY"


def max_expr(obj):
    if "allowed_symbols" in obj:
        return f"(float){obj['allowed_symbols'][-1]}"
    if "allowed" in obj:
        return c_number(max(obj["allowed"]))
    range_obj = obj.get("range", {})
    if "max_symbol" in range_obj:
        return f"(float){range_obj['max_symbol']}"
    if "max" in range_obj:
        return c_number(range_obj["max"])
    return "INFINITY"


def flags_expr(obj):
    flags = []
    if obj["access"] == "rw":
        flags.append("PARAM_FLAG_HOST_WRITE")
    if obj.get("write_state") == "disabled":
        flags.append("PARAM_FLAG_DISABLED_ONLY")
    return " | ".join(flags) if flags else "0U"


def change_expr(obj):
    value = obj.get("on_change")
    if value is None:
        return "PARAM_CHANGE_NONE"
    try:
        return CHANGE_C[value]
    except KeyError as exc:
        raise ValueError(f"unsupported on_change {value}") from exc


def render_c_ids(objects):
    lines = [
        "/* Generated from Parameter/*.yaml. DO NOT EDIT. */",
        "#ifndef PARAMETER_ID_GENERATED_H",
        "#define PARAMETER_ID_GENERATED_H",
        "",
        "typedef enum",
        "{",
    ]
    for name, obj in objects.items():
        if obj["kind"] == "action":
            source = obj["command"]
        else:
            source = obj["binding"] if "binding" in obj else f"{obj['getter']}()"
        lines.append(f"    /* {source}: {obj['description']} */")
        lines.append(f"    {name} = 0x{obj['id']:04X}U,")
        lines.append("")
    lines += ["} Parameter_Id_e;", "", "#endif /* PARAMETER_ID_GENERATED_H */", ""]
    return "\n".join(lines)


def render_c_table(objects):
    lines = ["/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */"]
    for name, obj in value_objects(objects):
        data = f"&{obj['binding']}" if "binding" in obj else "NULL"
        lines.append(
            "{ "
            f"{name}, {TYPE_C[obj['type']]}, {data}, "
            f"{min_expr(obj)}, {max_expr(obj)}, {flags_expr(obj)}, {change_expr(obj)} "
            "},"
        )
    lines.append("")
    return "\n".join(lines)


def render_c_read(objects):
    lines = ["/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */"]
    for name, obj in value_objects(objects):
        if "getter" not in obj:
            continue
        member = VALUE_MEMBER_C[obj["type"]]
        cast = CAST_C[obj["type"]]
        lines += [
            f"case {name}:",
            f"    Value->{member} = ({cast}){obj['getter']}();",
            "    return PARAM_OK;",
            "",
        ]
    return "\n".join(lines)


def render_c_action(objects):
    lines = ["/* Generated from Parameter/action_objects.yaml. DO NOT EDIT. */"]
    for name, obj in action_objects(objects):
        lines += [
            f"case {name}:",
            f"    Msg.Cmd = (ULONG){obj['command']};",
            f"    Msg.Arg = (ULONG){obj.get('arg', '0U')};",
            "    break;",
            "",
        ]
    return "\n".join(lines)


def render_c_plot(objects):
    lines = ["/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */"]
    for name, obj in value_objects(objects):
        if "plot_scale" not in obj:
            continue
        if "binding" not in obj or obj["type"] != "f32":
            raise ValueError(f"{name}: Plot requires direct f32 binding")
        lines += [
            f"case {name}:",
            f"    *Scale = {c_number(obj['plot_scale'])};",
            f"    return &{obj['binding']};",
            "",
        ]
    return "\n".join(lines)


def render_py_ids(objects):
    lines = [
        '"""Generated from Parameter/*.yaml. DO NOT EDIT."""',
        "",
    ]
    for name, obj in objects.items():
        lines.append(f"{name} = 0x{obj['id']:04X}")
    lines.append("")
    return "\n".join(lines)


def generate(objects):
    renderers = {
        "c_ids": render_c_ids,
        "c_table": render_c_table,
        "c_read": render_c_read,
        "c_action": render_c_action,
        "c_plot": render_c_plot,
        "py_ids": render_py_ids,
    }
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
