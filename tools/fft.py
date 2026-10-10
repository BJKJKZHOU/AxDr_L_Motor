#!/usr/bin/env python3
"""Capture existing AxDr Plot waveforms and calculate FFT / Welch PSD on the PC.

No motor-control actions, firmware changes or signal injection.
Requires numpy, matplotlib; live USB capture additionally requires pyserial.
"""

import argparse
from datetime import date
from pathlib import Path
import re
import struct
import time

import numpy as np

from parameter_ids_generated import (
    PARAM_ADC_IA, PARAM_ADC_IB, PARAM_ADC_IC, PARAM_ADC_VBUS,
    PARAM_RUN_ID, PARAM_RUN_IQ, PARAM_RUN_UD, PARAM_RUN_UQ,
    PARAM_RUN_THETA_E, PARAM_RUN_UALPHA, PARAM_RUN_UBETA,
    PARAM_OBS_THETA, PARAM_OBS_WE,
    PARAM_MECH_ESO_THETA, PARAM_MECH_ESO_WM,
    PARAM_MECH_ESO_TD, PARAM_MECH_ESO_ERROR,
    PARAM_SIGNAL_ID_REF, PARAM_SIGNAL_IQ_REF,
    PARAM_SIGNAL_ID_PI_OUT, PARAM_SIGNAL_IQ_PI_OUT,
    PARAM_RUN_WM, PARAM_REF_WM, PARAM_REF_IQ,
)

# name: (Parameter ID, physical unit, FAST int16 scale, NORMAL available)
CHANNELS = {
    "ia": (PARAM_ADC_IA, "A", .001, True),
    "ib": (PARAM_ADC_IB, "A", .001, True),
    "ic": (PARAM_ADC_IC, "A", .001, True),
    "vbus": (PARAM_ADC_VBUS, "V", None, True),
    "id": (PARAM_RUN_ID, "A", .001, True),
    "iq": (PARAM_RUN_IQ, "A", .001, True),
    "ud": (PARAM_RUN_UD, "V", .001, True),
    "uq": (PARAM_RUN_UQ, "V", .001, True),
    "theta_e": (PARAM_RUN_THETA_E, "rad", .0002, True),
    "ualpha": (PARAM_RUN_UALPHA, "V", .001, True),
    "ubeta": (PARAM_RUN_UBETA, "V", .001, True),
    "obs_theta": (PARAM_OBS_THETA, "rad", .0002, True),
    "obs_we": (PARAM_OBS_WE, "rad/s", .1, True),
    "eso_theta": (PARAM_MECH_ESO_THETA, "rad", .0002, True),
    "wm_eso": (PARAM_MECH_ESO_WM, "rad/s", .1, True),
    "td_eso": (PARAM_MECH_ESO_TD, "N*m", .001, True),
    "eso_error": (PARAM_MECH_ESO_ERROR, "rad", .0002, True),
    "id_ref": (PARAM_SIGNAL_ID_REF, "A", .001, True),
    "iq_ref": (PARAM_SIGNAL_IQ_REF, "A", .001, True),
    "ud_pi": (PARAM_SIGNAL_ID_PI_OUT, "V", .001, True),
    "uq_pi": (PARAM_SIGNAL_IQ_PI_OUT, "V", .001, True),
    "wm": (PARAM_RUN_WM, "rad/s", None, True),
    "wm_ref": (PARAM_REF_WM, "rad/s", None, True),
    "iq_cmd": (PARAM_REF_IQ, "A", None, True),
}

RATES = {"fast": 20000, "normal": 1000}
MAGIC = b"AXDR"
CANFD_LENGTHS = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)
MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_NORMAL_DATA = 0x10
MSG_FAST_DATA = 0x18
PLOT_CONFIG, PLOT_START, PLOT_STOP = 1, 2, 3


def channel_selection(group, names):
    names = tuple(name.lower() for name in names)
    if not names or len(set(names)) != len(names):
        raise ValueError("select distinct, nonempty channel names")
    limit = 8 if group == "fast" else 15
    if len(names) > limit:
        raise ValueError(f"{group.upper()} supports at most {limit} channels")
    for name in names:
        if name not in CHANNELS:
            raise ValueError(f"unknown channel {name!r}; available: {', '.join(CHANNELS)}")
        if group == "fast" and CHANNELS[name][2] is None:
            raise ValueError(f"{name} is NORMAL only")
        if not CHANNELS[name][3]:
            raise ValueError(f"{name} does not support NORMAL Plot")
    return names


def decode_samples(raw, group, names):
    dtype = "<i2" if group == "fast" else "<f4"
    data = np.frombuffer(raw, dtype=dtype)
    if not len(data) or len(data) % len(names):
        raise ValueError("empty or incomplete interleaved Plot samples")
    values = data.reshape(-1, len(names)).astype(np.float64).T
    if group == "fast":
        values *= np.asarray([CHANNELS[name][2] for name in names])[:, None]
        if np.any((data == 32767) | (data == -32768)):
            print("Warning: FAST Plot contains saturated int16 values")
    if not np.all(np.isfinite(values)):
        raise ValueError("Plot includes non-finite samples")
    return values


class PlotClient:
    def __init__(self, port, baud):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("live capture requires pyserial") from exc
        self.ser = serial.Serial(port, baud, timeout=.02, write_timeout=1)
        self.buffer = bytearray()
        self.txn = 0
        self.collect = False
        self.samples = bytearray()
        self.last_seq = None
        self.lost = 0
        self.bad = 0

    def close(self):
        self.ser.close()

    def poll(self):
        self.buffer.extend(self.ser.read(4096))
        responses = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                del self.buffer[:-3]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 7:
                break
            msg_id, size = struct.unpack_from("<HB", self.buffer, 4)
            if msg_id > 0x07FF or size not in CANFD_LENGTHS:
                del self.buffer[0]
                continue
            if len(self.buffer) < 7 + size:
                break
            payload = bytes(self.buffer[7:7 + size])
            del self.buffer[:7 + size]
            msg_type = msg_id >> 6
            if msg_type == MSG_RESPONSE:
                responses.append(payload)
            elif self.collect and msg_type == self.data_type:
                if len(payload) < 4 or payload[2] != self.config_id:
                    self.bad += 1
                    continue
                seq = struct.unpack_from("<H", payload)[0]
                count = payload[3]
                n = self.count if self.group == "fast" else count
                required = 4 + count * n * (2 if self.group == "fast" else 4)
                if (count == 0 or len(payload) < required or
                    (self.group == "fast" and count > 20) or
                    (self.group == "normal" and count != self.count)):
                    self.bad += 1
                    continue
                if self.last_seq is not None:
                    self.lost += (seq - self.last_seq - 1) & 0xFFFF
                self.last_seq = seq
                self.samples.extend(payload[4:required])
        return responses

    def request(self, op, payload=b"", timeout=2.0):
        self.txn = (self.txn % 255) + 1
        txn = self.txn
        body = bytes((txn, op)) + payload
        length = next(n for n in CANFD_LENGTHS if n >= len(body))
        frame = MAGIC + struct.pack("<HB", (MSG_PLOT << 6) | 1, length)
        self.ser.write(frame + body.ljust(length, b"\x00"))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for response in self.poll():
                if (len(response) >= 4 and
                    response[0] == txn and response[1] == MSG_PLOT and
                    response[2] == op):
                    if response[3] != 0:
                        raise RuntimeError(f"Plot command {op} failed: status {response[3]}")
                    return
        raise TimeoutError(f"Plot command {op} timed out")

    def capture(self, group, names, seconds):
        self.group = group
        self.count = len(names)
        self.config_id = 31 if group == "fast" else 32
        self.data_type = MSG_FAST_DATA if group == "fast" else MSG_NORMAL_DATA
        group_id = 0 if group == "fast" else 1
        payload = bytes((group_id, self.config_id, len(names)))
        payload += b"".join(struct.pack("<H", CHANNELS[name][0]) for name in names)
        self.request(PLOT_CONFIG, payload)
        self.samples.clear()
        self.last_seq = None
        self.lost = self.bad = 0
        self.collect = True
        try:
            self.request(PLOT_START, bytes((1 << group_id,)))
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                self.poll()
        finally:
            try:
                self.request(PLOT_STOP, bytes((1 << group_id,)))
            finally:
                self.collect = False
        if self.lost or self.bad:
            raise RuntimeError(f"incomplete Plot data: seq_lost={self.lost}, malformed={self.bad}")
        raw = bytes(self.samples)
        count = len(raw) // (len(names) * (2 if group == "fast" else 4))
        if count < RATES[group] * seconds * .90:
            raise RuntimeError(f"incomplete Plot rate: {count} samples in {seconds:g}s")
        return raw


def fft_spectrum(data, fs):
    """Single-sided, Hann coherent-gain corrected peak amplitude and complex bins."""
    n = data.shape[1]
    w = np.hanning(n + 1)[:-1]  # periodic Hann
    x = data - np.mean(data, axis=1, keepdims=True)
    bins = np.fft.rfft(x * w, axis=1) / np.sum(w)
    amp = 2.0 * np.abs(bins)
    amp[:, 0] *= .5
    if n % 2 == 0:
        amp[:, -1] *= .5
    return np.fft.rfftfreq(n, 1 / fs), bins, amp


def welch_psd(data, fs, window_size=8192):
    """One-sided Welch PSD with periodic Hann and 50% overlap (unit^2/Hz)."""
    n = min(window_size, data.shape[1])
    if n < 8:
        raise ValueError("not enough samples for Welch PSD")
    hop = max(n // 2, 1)
    w = np.hanning(n + 1)[:-1]
    power = np.zeros((data.shape[0], n // 2 + 1), dtype=np.float64)
    windows = 0
    for start in range(0, data.shape[1] - n + 1, hop):
        segment = data[:, start:start + n]
        bins = np.fft.rfft((segment - segment.mean(axis=1, keepdims=True)) * w, axis=1)
        power += np.abs(bins) ** 2
        windows += 1
    power /= windows * fs * np.sum(w * w)
    power[:, 1:-1] *= 2
    if n % 2:
        power[:, -1] *= 2
    return np.fft.rfftfreq(n, 1 / fs), power, n, windows


def save_spectrum(path, kind, group, names, fs, sample_count, frequency, values,
                  fft_n, windows):
    with path.open("wb") as handle:
        np.savez_compressed(
            handle,
            format_version=np.array([1], dtype=np.int32),
            kind=np.array([kind]),
            group=np.array([group]),
            sample_rate_hz=np.array([fs], dtype=np.float64),
            sample_count=np.array([sample_count], dtype=np.int64),
            channel_id=np.array([CHANNELS[name][0] for name in names], dtype=np.uint16),
            channel_name=np.asarray(names, dtype="<U32"),
            channel_unit=np.asarray([CHANNELS[name][1] for name in names], dtype="<U32"),
            window=np.array(["hann_periodic"]),
            segment_length=np.array([fft_n], dtype=np.int64),
            segment_count=np.array([windows], dtype=np.int64),
            freq_hz=frequency.astype(np.float64),
            spectrum=values,
        )


def plot_spectrum(freq, values, names, units, kind, fs, n_samples, output):
    import matplotlib as mpl
    from matplotlib import pyplot as plt
    from matplotlib.ticker import AutoMinorLocator

    with mpl.rc_context({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.linewidth": .8,
        "savefig.dpi": 180,
    }):
        fig, ax = plt.subplots(figsize=(11.69, 8.27), dpi=180)
        fig.subplots_adjust(left=.105, right=.975, bottom=.12, top=.79)
        for i, name in enumerate(names):
            ax.plot(freq, values[i], linewidth=1.0, label=f"{name} ({units[i]})")
        ax.set_xlim(0, float(fs) / 2)
        ax.set_xlabel("Frequency (Hz)")
        ylabel = "Peak amplitude (physical unit)" if kind == "fft" else "PSD (physical unit²/Hz)"
        ax.set_ylabel(ylabel)
        fig.text(.5, .911,
                 "FFT amplitude spectrum" if kind == "fft" else "Welch power spectral density",
                 ha="center", fontsize=10)
        ax.grid(which="major", linewidth=.55, alpha=.45)
        ax.grid(which="minor", linewidth=.30, alpha=.22)
        ax.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))
        fig.suptitle(f"AxDr  {kind.upper()}  |  Fs={fs:g} Hz  |  N={n_samples}",
                     y=.963, fontsize=13, fontweight="bold")
        ax.legend(loc="lower center", bbox_to_anchor=(.5, 1.015), ncol=min(4, len(names)),
                  fontsize=9, frameon=False)
        fig.savefig(output, dpi=180)
        plt.close(fig)


def draw_archive(archive_path, out):
    with np.load(archive_path, allow_pickle=False) as saved:
        if int(saved["format_version"][0]) != 1:
            raise ValueError("unsupported spectrum NMIXX version")
        kind = str(saved["kind"][0])
        if kind not in ("fft", "psd"):
            raise ValueError("not an FFT/PSD NMIXX archive")
        freq = saved["freq_hz"]
        values = saved["spectrum"]
        names = list(saved["channel_name"])
        units = list(saved["channel_unit"])
        fs = float(saved["sample_rate_hz"][0])
        n = int(saved["sample_count"][0])
        if values.shape != (len(names), len(freq)) or not np.all(np.isfinite(values)):
            raise ValueError("invalid archived spectrum")
        amp = np.abs(values) * 2
        if kind == "fft":
            amp[:, 0] *= .5
            if int(saved["segment_length"][0]) % 2 == 0:
                amp[:, -1] *= .5
        else:
            amp = values
    out.mkdir(parents=True, exist_ok=True)
    image = out / archive_path.name.replace(".nmixx", ".png")
    plot_spectrum(freq, amp, names, units, kind, fs, n, image)
    print("Replotted:", image)


def main():
    ap = argparse.ArgumentParser(description="AxDr USB Plot FFT / Welch PSD")
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="live USB CDC device, e.g. /dev/ttyACM0")
    source.add_argument("--input-raw", type=Path, help="offline interleaved Plot raw samples")
    source.add_argument("--result", type=Path, help="replot a saved *.fft.nmixx or *.psd.nmixx")
    ap.add_argument("--group", choices=("fast", "normal"), default="fast")
    ap.add_argument("--channels", nargs="+", default=["id", "iq"])
    ap.add_argument("--duration", type=float, default=10.0, help="live capture duration in seconds")
    ap.add_argument("--analysis", choices=("fft", "psd", "both"), default="both")
    ap.add_argument("--welch-n", type=int, default=8192)
    ap.add_argument("--name", default="Spectrum", help="test name prefix before -YYYY-MM-DD")
    ap.add_argument("--out", type=Path, default=Path("test-data/fft"))
    ap.add_argument("--save-raw", action="store_true", help="also keep interleaved time samples locally")
    ap.add_argument("--baud", type=int, default=115200, help="USB CDC line coding")
    args = ap.parse_args()
    if args.result is not None:
        draw_archive(args.result, args.out)
        return

    if (not np.isfinite(args.duration) or args.duration <= 0 or
        args.welch_n < 8 or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.name)):
        ap.error("invalid duration, Welch length or test name")
    try:
        names = channel_selection(args.group, args.channels)
    except ValueError as exc:
        ap.error(str(exc))

    fs = RATES[args.group]
    args.out.mkdir(parents=True, exist_ok=True)
    prefix = f"{args.name}-{date.today().isoformat()}"
    selected = ("fft", "psd") if args.analysis == "both" else (args.analysis,)
    for kind in selected:
        path = args.out / f"{prefix}.{kind}.nmixx"
        image = args.out / f"{prefix}.{kind}.png"
        if path.exists() or image.exists():
            ap.error(f"output already exists; change --name or --out: {path} / {image}")

    if args.input_raw is not None:
        raw = args.input_raw.read_bytes()
    else:
        client = PlotClient(args.port, args.baud)
        try:
            raw = client.capture(args.group, names, args.duration)
        finally:
            client.close()

    data = decode_samples(raw, args.group, names)
    if data.shape[1] < 8:
        raise RuntimeError("fewer than 8 valid samples")
    if args.save_raw and args.port is not None:
        raw_path = args.out / f"{prefix}.raw"
        if raw_path.exists():
            ap.error(f"raw file already exists: {raw_path}")
        raw_path.write_bytes(raw)
        print("Raw:", raw_path)
    units = [CHANNELS[name][1] for name in names]
    print(f"Captured {data.shape[1]} samples, {data.shape[1] / fs:.3f} s, "
          f"{fs:g} Hz, channels={','.join(names)}")

    for kind in selected:
        if kind == "fft":
            freq, bins, amplitude = fft_spectrum(data, fs)
            spectrum = bins
            nfft = data.shape[1]
            windows = 1
            plotted = amplitude
        else:
            freq, spectrum, nfft, windows = welch_psd(data, fs, args.welch_n)
            plotted = spectrum
        archive = args.out / f"{prefix}.{kind}.nmixx"
        save_spectrum(archive, kind, args.group, names, fs, data.shape[1],
                      freq, spectrum, nfft, windows)
        image = args.out / f"{prefix}.{kind}.png"
        plot_spectrum(freq, plotted, names, units, kind, fs, data.shape[1], image)
        print("Saved:", archive, "and", image)


if __name__ == "__main__":
    main()
