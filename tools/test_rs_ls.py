#!/usr/bin/env python3
"""Offline RL algorithm and test-result persistence tests; no board access."""
import json
import contextlib
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from types import SimpleNamespace

import identification_test as ident

ROOT = Path(__file__).resolve().parents[1]


class RsLsTest(unittest.TestCase):
    def test_firmware_branches(self):
        with tempfile.TemporaryDirectory(prefix="rs-ls-") as directory:
            out = Path(directory)
            (out / "main.h").write_text("#pragma once\n#include <stdint.h>\n")
            includes = [out, *[ROOT / name for name in (
                "Motor", "Motion", "Algo", "User", "Observer", "Identification",
                "Sensorless", "AZURE_RTOS/App", "Parameter", "Comm", "Storage")]]
            subprocess.run([
                "cc", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                *[f"-I{path}" for path in includes],
                str(ROOT / "tools/tests/rs_ls.c"),
                str(ROOT / "Sensorless/Align.c"), str(ROOT / "Algo/Sin_LUT.c"),
                "-lm", "-o", str(out / "test"),
            ], check=True)
            subprocess.run([str(out / "test")], check=True)

    def test_runs_are_saved_before_continuing(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result.json"
            args = SimpleNamespace(output=str(output), stop_on_error=False, interval=0)
            rows = []

            def run(mode, number, direction=None):
                result = {"run": number, "valid": number == 2, "time_s": 1,
                          "phase_peak_a": 1, "rs_ohm": .3, "ls_h": .00008}
                if number == 1:
                    raise ident.IdentificationFailed("rejected", result)
                self.assertEqual(len(output.with_suffix(".runs.jsonl").read_text().splitlines()), 1)
                return result

            with contextlib.redirect_stdout(io.StringIO()):
                ident.run_group(SimpleNamespace(run_ident=run), ident.IDENT_RS_LS, 2, args, rows)
            saved = [json.loads(line) for line in output.with_suffix(".runs.jsonl").read_text().splitlines()]
            self.assertEqual([row["valid"] for row in saved], [False, True])


if __name__ == "__main__":
    unittest.main()
