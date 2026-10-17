#!/usr/bin/env python3
"""Check that firmware Host metadata survives TOML export."""

import unittest

from export_host_schema import render_host_schema
from gen_parameters import load_objects


def parameter_block(manifest: str, symbol: str) -> str:
    marker = f'[[parameters]]\nsymbol = "{symbol}"\n'
    if marker not in manifest:
        raise AssertionError(f"missing Host Parameter {symbol}")
    return manifest.split(marker, 1)[1].split("[[parameters]]", 1)[0].split("[[actions]]", 1)[0]


class HostSchemaTests(unittest.TestCase):
    def test_persistent_and_readback_metadata(self):
        manifest = render_host_schema(load_objects())
        current = parameter_block(manifest, "PARAM_CTRL_CURRENT_BW_HZ")
        self.assertIn("persistent = true", current)
        self.assertIn('readback = ["PARAM_CTRL_CURRENT_SOURCE"]', current)

        speed = parameter_block(manifest, "PARAM_CTRL_SPEED_BW_HZ")
        self.assertIn("persistent = true", speed)
        self.assertIn('"PARAM_CTRL_SPEED_KP_EFFECTIVE"', speed)
        self.assertIn('"PARAM_CTRL_SPEED_KI_EFFECTIVE"', speed)

        runtime = parameter_block(manifest, "PARAM_RUN_IQ")
        self.assertNotIn("persistent = true", runtime)

    def test_host_metadata_does_not_change_firmware_parameter_definitions(self):
        objects = load_objects()
        self.assertEqual(objects["PARAM_CTRL_SPEED_BW_HZ"]["on_change"], "SPEED_TUNING")
        self.assertEqual(objects["PARAM_CTRL_SPEED_BW_HZ"]["binding"], "Control_Speed_Bw_Hz")


if __name__ == "__main__":
    unittest.main()
