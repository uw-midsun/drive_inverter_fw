#!/usr/bin/env python3
"""CAN schema codegen: generates comms_schema.rs and can_debug.h from can_schema.yaml.

Usage:
    python3 autogen/generate.py
"""

import sys
import yaml
from pathlib import Path
from jinja2 import Environment, FileSystemLoader

SCRIPT_DIR   = Path(__file__).parent
SCHEMA_PATH  = SCRIPT_DIR / "can_schema.yaml"
TEMPLATE_DIR = SCRIPT_DIR / "templates"
OUT_RS       = SCRIPT_DIR / "../app/src/comms_schema.rs"
OUT_H        = SCRIPT_DIR / "../projects/motor_controller/Core/Src/app/can/can_schema.h"


def to_pascal(s: str) -> str:
    return "".join(w.capitalize() for w in s.split("_"))


def build_context(schema: dict) -> dict:
    meta = schema["meta"]
    all_signals = []  # flat list used for Signal enum indices

    def process_frames(section_key: str, const_prefix: str) -> list:
        frames = []
        for frame in schema.get(section_key, {}).get("frames", []):
            raw_sigs = frame.get("signals", [])
            sigs = []
            for i, sig in enumerate(raw_sigs):
                entry = {
                    "name":     sig["name"],
                    "pascal":   to_pascal(sig["name"]),
                    "index":    len(all_signals),
                    "unit":     sig.get("unit", ""),
                    "position": "lo" if i == 0 else "hi",
                }
                all_signals.append(entry)
                sigs.append(entry)
            frames.append({
                "name":       frame["name"],
                "offset":     frame["offset"],
                "const_name": f"{const_prefix}_{frame['name'].upper()}",
                "signals":    sigs,
                "encode":     frame.get("encode", []),
            })
        return frames

    ws22_frames  = process_frames("ws22",  "MC")
    dc_frames    = process_frames("dc",    "DC")
    debug_frames = process_frames("debug", "MC_DEBUG")

    all_frames = ws22_frames + debug_frames
    total_range = max(f["offset"] for f in all_frames) + 1 if all_frames else 0x10

    return {
        "meta":         meta,
        "ws22_frames":  ws22_frames,
        "dc_frames":    dc_frames,
        "debug_frames": debug_frames,
        "all_signals":  all_signals,
        "signal_count": len(all_signals),
        "total_range":  total_range,
    }


def render(template_name: str, ctx: dict) -> str:
    env = Environment(
        loader=FileSystemLoader(str(TEMPLATE_DIR)),
        trim_blocks=True,
        lstrip_blocks=True,
        keep_trailing_newline=True,
    )
    env.filters["hex"] = lambda v, w=2: f"0x{v:0{w}X}"
    return env.get_template(template_name).render(**ctx)


def main() -> None:
    with open(SCHEMA_PATH) as f:
        schema = yaml.safe_load(f)

    ctx = build_context(schema)

    for template, out_path in [
        ("comms_schema.rs.jinja", OUT_RS),
        ("can_schema.h.jinja",    OUT_H),
    ]:
        text = render(template, ctx)
        out_path.resolve().write_text(text)
        print(f"wrote {out_path.resolve()}")


if __name__ == "__main__":
    main()
