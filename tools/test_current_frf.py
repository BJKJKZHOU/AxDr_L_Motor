#!/usr/bin/env python3
"""Host-only synthetic checks for tools/current_frf.py; requires numpy and pyserial."""

import math
import struct

import numpy as np

from current_frf import Parser, analyze, wire


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
    raw = np.rint(signals / 0.001).astype("<i2").tobytes()
    raw_matrix, result = analyze(raw, freq, 0.10, 2.0)
    closed = result["closed_loop_Id_over_IdRef"]
    assert raw_matrix.shape == (30000, 4)
    assert abs(closed["gain_db"] - 20.0 * math.log10(0.7)) < 0.15, closed
    assert abs(closed["phase_deg"] + 45.0) < 0.5, closed
    assert result["used_samples"] > 20000


if __name__ == "__main__":
    test_protocol()
    test_frequency_response()
    print("current FRF offline tests PASS")
