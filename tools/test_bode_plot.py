"""Synthetic regression tests for bode_plot.py and the archive replot utility."""

import json

import numpy as np

from bode_plot import bode_metrics, plot_bode_a4, responses
from bode_replot import replot


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
