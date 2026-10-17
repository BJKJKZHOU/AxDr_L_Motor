#!/usr/bin/env python3
"""Offline waveform NMIXX and USB parsing regressions; no motor or SWD."""

import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import fft
import current_waveform as wave


class WaveformTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.fs = 20000
        self.names = ("ia", "ib", "theta_e")
        theta = 2 * np.pi * 700 * np.arange(640) / self.fs
        ia = 0.7 * np.sin(theta)
        ib = 0.7 * np.sin(theta - 2 * np.pi / 3)
        self.raw = np.rint(np.column_stack((
            ia / .001, ib / .001, (theta % (2 * np.pi)) / .0002
        ))).astype("<i2")
        self.info = dict(
            quality="valid", quality_reason="",
            firmware_drop_before=8, firmware_drop_after=8,
            usb_seq_lost=0, malformed_frames=0,
            frame_seq=np.arange(100, 132, dtype=np.uint16),
            frame_sample_count=np.full(32, 20, dtype=np.uint8)
        )

    def save(self, name="input", info=None):
        path = self.out / f"{name}.waveform.nmixx"
        fft.save_waveform(path, self.raw.tobytes(), "fast", self.names,
                          self.fs, self.info if info is None else info,
                          dict(controller="PI", pole_pairs=7, target_rpm=6000))
        return path

    def test_raw_round_trip_and_fundamental(self):
        path = self.save()
        with np.load(path, allow_pickle=False) as z:
            np.testing.assert_array_equal(z["raw_samples"], self.raw)
            np.testing.assert_array_equal(z["frame_seq"], self.info["frame_seq"])
            self.assertEqual(str(z["quality"][0]), "valid")
            self.assertEqual(tuple(z["channel_name"]), self.names)
        raw, group, names, fs = fft.read_waveform(path)
        self.assertEqual(raw, self.raw.tobytes())
        self.assertEqual(group, "fast")
        self.assertEqual(names, self.names)
        self.assertEqual(fs, self.fs)
        cap = wave.validate(wave.load_capture(path, self.fs))
        self.assertEqual(cap["names"], ["ia", "ib", "theta_e", "ic"])
        np.testing.assert_allclose(cap["values"][:, [0, 1, 3]].sum(axis=1), 0, atol=1e-9)
        phase = wave.fundamental(cap)
        self.assertIsNotNone(phase)
        _, fitted = phase
        self.assertAlmostEqual(fitted["ia"][1], 0.7 / np.sqrt(2), delta=.002)
        self.assertLess(fitted["ib"][2], .003)
        self.assertLess(fitted["ic"][2], .004)

    def test_unknown_drop_is_saved_but_not_spectral(self):
        info = {**self.info, "quality": "unknown",
                "quality_reason": "firmware drop counter unavailable",
                "firmware_drop_before": None, "firmware_drop_after": None}
        path = self.save("unknown", info)
        cap = wave.validate(wave.load_capture(path, self.fs))
        self.assertIsNone(wave.fundamental(cap))
        with self.assertRaisesRegex(ValueError, "verified continuous"):
            fft.read_waveform(path)

    def test_firmware_drop_is_invalid_not_spectral(self):
        info = {**self.info, "quality": "invalid",
                "quality_reason": "firmware dropped=2 blocks/samples",
                "firmware_drop_after": 10}
        path = self.save("dropped", info)
        cap = wave.validate(wave.load_capture(path, self.fs))
        self.assertIsNone(wave.fundamental(cap))
        with self.assertRaises(ValueError):
            fft.read_waveform(path)

    def test_forged_sequence_cannot_be_marked_valid(self):
        original = self.save()
        with np.load(original, allow_pickle=False) as z:
            data = {key: z[key] for key in z.files}
        data["frame_seq"] = data["frame_seq"].copy()
        data["frame_seq"][3] = data["frame_seq"][2]
        bad = self.out / "bad.waveform.nmixx"
        with bad.open("wb") as file:
            np.savez_compressed(file, **data)
        with self.assertRaisesRegex(ValueError, "archive falsely claims"):
            wave.load_capture(bad, self.fs)
        with self.assertRaisesRegex(ValueError, "integrity"):
            fft.read_waveform(bad)

    def test_no_overwrite_and_offline_a4_plot(self):
        path = self.save()
        with self.assertRaises(FileExistsError):
            self.save()
        cap = wave.validate(wave.load_capture(path, self.fs))
        cap["stem"] = "01-PI"
        import matplotlib
        matplotlib.use("Agg")
        self.assertTrue(wave.plot_waveform(cap, self.out, 5, None).is_file())
        self.assertTrue(wave.plot_phase(cap, self.out).is_file())

    def test_parameter_u32_read_uses_existing_usb_message(self):
        client = fft.PlotClient.__new__(fft.PlotClient)
        pid = fft.PARAM_PLOT_FAST_DROP
        data = struct.pack("<HBI", pid, fft.PARAM_U32, 123)
        with patch.object(client, "request", return_value=data) as request:
            self.assertEqual(client.read_u32(pid), 123)
            request.assert_called_once_with(fft.PARAM_READ, struct.pack("<H", pid),
                                            msg_type=fft.MSG_PARAMETER)

    def test_hann_fft_on_saved_raw_waveform(self):
        path = self.save()
        raw, group, names, fs = fft.read_waveform(path)
        data = fft.decode_samples(raw, group, names)
        freq, _, amp = fft.fft_spectrum(data[:2], fs)
        self.assertAlmostEqual(freq[np.argmax(amp[0])], 687.5, delta=25)
        self.assertGreater(amp[0].max(), .60)

    def test_legacy_npz_read_only(self):
        path = self.out / "legacy.npz"
        values = (self.raw[:, :2] * .001).astype(np.float64)
        with path.open("wb") as file:
            np.savez_compressed(
                file, format_version=1,
                time_s=np.arange(len(values)) / self.fs,
                values=values, channel_names=["ia", "ib"],
                phase_pair=np.full(len(values), -1, dtype=np.int8),
                sample_rate_hz=self.fs, label="old", metadata_json="{}")
        cap = wave.validate(wave.load_capture(path, self.fs))
        self.assertEqual(cap["quality"], "unknown")
        self.assertIsNone(wave.fundamental(cap))


if __name__ == "__main__":
    unittest.main()
