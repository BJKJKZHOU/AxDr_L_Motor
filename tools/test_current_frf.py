#!/usr/bin/env python3
"""Host-only synthetic checks for tools/current_frf.py; requires numpy and pyserial."""

import math
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import struct

import numpy as np

from current_frf import Parser, analyze, wire, bode_metrics, plot_bode_a4, responses, replot


def test_protocol():
    msg = wire(0x07, bytes([1, 2, 0x01, 0x13, 6]))
    parser = Parser()
    assert parser.feed(msg[:3]) == []
    assert parser.feed(msg[3:7]) == []
    frames = parser.feed(msg[7:])
    assert len(frames) == 1
    assert frames[0][0] == 0x07
    assert frames[0][1][:5] == bytes([1, 2, 0x01, 0x13, 6])


def test_frequency_response():
    fs = 20000.0
    freq = 100.0
    t = np.arange(30000, dtype=np.float64) / fs
    sine = np.sin(2.0 * math.pi * freq * t)
    signals = np.column_stack([
        0.10 * sine,
        0.10 * sine,
        0.07 * np.sin(2.0 * math.pi * freq * t - math.pi / 4.0),
        0.03 * np.sin(2.0 * math.pi * freq * t + math.pi / 6.0),
    ])
    raw = bytearray(np.rint(signals / 0.001).astype("<i2").tobytes())
    raw_matrix, result = analyze(raw, freq, 0.10, 2.0)
    # Starting the next frequency must not be blocked by the previous analysis.
    expected = raw_matrix.copy()
    raw.clear()
    raw.extend(b"\x00" * 16)
    np.testing.assert_array_equal(raw_matrix, expected)
    closed = result["closed_loop_Id_over_IdRef"]
    assert raw_matrix.shape == (30000, 4)
    assert abs(closed["gain_db"] - 20.0 * math.log10(0.7)) < 0.15, closed
    assert abs(closed["phase_deg"] + 45.0) < 0.5, closed
    assert result["used_samples"] > 20000


def integrator():
    f = np.geomspace(1, 1e4, 130)
    wc = 2 * np.pi * 180
    L = wc / (1j * 2 * np.pi * f)
    T = L / (1 + L)
    S = 1 / (1 + L)
    return f, L, S, T


def test_transfer_identities():
    tr = responses(1 + 0j, 0.7 - 0.1j, 0.5 + 0.2j)
    assert abs(tr["S"] + tr["T"] - 1) < 1e-10
    assert abs(tr["C"] * tr["G"] - tr["L"]) < 1e-10
    assert abs(tr["L"] / (1 + tr["L"]) - tr["T"]) < 1e-10


def test_known_integrator():
    f, L, S, T = integrator()
    m = bode_metrics(f, L, T)
    assert 165 < m["relative_minus_3db_hz"] < 195, m
    assert 170 < m["gain_crossover_hz"] < 190, m
    assert 89 < m["phase_margin_deg"] < 91, m
    assert m["gain_margin_db"] is None, m


def test_plot(tmp_path):
    f, L, S, T = integrator()
    image = tmp_path / "bode.png"
    metrics = plot_bode_a4(f, L, S, T, "POS sweep", image)
    assert image.exists() and image.stat().st_size > 30000
    assert image.with_suffix(".svg").exists()
    import matplotlib.image as mpimg
    a = mpimg.imread(image)
    assert abs(a.shape[1] / a.shape[0] - (11.69 / 8.27)) < 0.01
    assert metrics["relative_minus_3db_hz"] > 0


def test_archive_replot(tmp_path):
    f, L, S, T = integrator()
    payload = {"bode": [
        {"bandwidth_hz": 180, "frequency_hz": float(hz),
         "closed_loop_Id_over_IdRef": {
             "gain_db": float(20 * np.log10(abs(h))),
             "phase_deg": float(np.rad2deg(np.angle(h)))}}
        for hz, h in zip(f, T)
    ]}
    src = tmp_path / "result.json"
    src.write_text(json.dumps(payload))
    data = replot(src, tmp_path / "result_plots")
    assert (tmp_path / "result_plots" / "bode_a4_bw_180.png").exists()
    assert data["180"]["relative_minus_3db_hz"] > 0


def test_no_guess_beyond_sweep():
    f = np.geomspace(1, 5, 20)
    L = 300 / (1j * f)
    T = L / (1 + L)
    x = bode_metrics(f, L, T)
    assert x["relative_minus_3db_hz"] is None
    assert x["gain_crossover_hz"] is None
    assert x["gain_margin_db"] is None


if __name__ == "__main__":
    test_protocol()
    test_frequency_response()
    test_transfer_identities()
    test_known_integrator()
    test_no_guess_beyond_sweep()
    with TemporaryDirectory() as folder:
        test_plot(Path(folder))
        test_archive_replot(Path(folder))
    print("current FRF and Bode offline tests PASS")
