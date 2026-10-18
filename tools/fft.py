#!/usr/bin/env python3
"""Capture existing AxDr Plot waveforms and calculate FFT / Welch PSD on the PC.

No motor-control actions, firmware changes or signal injection.
Requires numpy, matplotlib; live USB capture additionally requires pyserial.
"""

import argparse
import json
from datetime import date, datetime
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
    PARAM_PLOT_FAST_DROP, PARAM_PLOT_NORMAL_DROP,
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
MSG_PARAMETER = 0x07
PARAM_READ = 0x01
PARAM_U32 = 0x04
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
        self.frame_seq = []
        self.frame_count = []
        self.last_capture_info = None

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
                self.frame_seq.append(seq)
                self.frame_count.append(count)
                self.samples.extend(payload[4:required])
        return responses

    def request(self, op, payload=b"", timeout=2.0, msg_type=MSG_PLOT):
        self.txn = (self.txn % 255) + 1
        txn = self.txn
        body = bytes((txn, op)) + payload
        length = next(n for n in CANFD_LENGTHS if n >= len(body))
        frame = MAGIC + struct.pack("<HB", (msg_type << 6) | 1, length)
        self.ser.write(frame + body.ljust(length, b"\x00"))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for response in self.poll():
                if (len(response) >= 4 and
                    response[0] == txn and response[1] == msg_type and
                    response[2] == op):
                    if response[3] != 0:
                        raise RuntimeError(f"AxDr {msg_type}/{op} failed: status {response[3]}")
                    return response[4:]
        raise TimeoutError(f"AxDr {msg_type}/{op} timed out")

    def read_u32(self, param_id):
        # A read-only Parameter transaction over the existing USB CDC link.
        data = self.request(PARAM_READ, struct.pack("<H", param_id),
                            msg_type=MSG_PARAMETER)
        if (len(data) < 7 or struct.unpack_from("<H", data)[0] != param_id
                or data[2] != PARAM_U32):
            raise ValueError(f"unexpected Parameter response for 0x{param_id:04X}")
        return struct.unpack_from("<I", data, 3)[0]

    def try_read_drop(self, group):
        try:
            return self.read_u32(PARAM_PLOT_FAST_DROP if group == "fast"
                                 else PARAM_PLOT_NORMAL_DROP)
        except (RuntimeError, TimeoutError, ValueError):
            # Older firmware has no USB-readable drop counter.
            return None

    def capture(self, group, names, seconds, *, strict=True):
        self.group = group
        self.count = len(names)
        self.config_id = 31 if group == "fast" else 32
        self.data_type = MSG_FAST_DATA if group == "fast" else MSG_NORMAL_DATA
        group_id = 0 if group == "fast" else 1
        payload = bytes((group_id, self.config_id, len(names)))
        payload += b"".join(struct.pack("<H", CHANNELS[name][0]) for name in names)
        self.request(PLOT_CONFIG, payload)
        before = self.try_read_drop(group)
        self.samples.clear()
        self.frame_seq.clear()
        self.frame_count.clear()
        self.last_seq = None
        self.lost = self.bad = 0
        self.last_capture_info = None
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
        after = self.try_read_drop(group)
        raw = bytes(self.samples)
        sample_count = len(raw) // (len(names) * (2 if group == "fast" else 4))
        delta = ((after - before) & 0xFFFFFFFF) if before is not None and after is not None else None
        problems = []
        if self.lost:
            problems.append(f"USB sequence gaps={self.lost}")
        if self.bad:
            problems.append(f"malformed frames={self.bad}")
        if delta:
            problems.append(f"firmware dropped={delta} blocks/samples")
        if sample_count < RATES[group] * seconds * .90:
            problems.append(f"received only {sample_count} samples in {seconds:g}s")
        if sum(self.frame_count) != sample_count:
            problems.append("packet sample counts do not match captured bytes")
        quality = "invalid" if problems else ("valid" if delta is not None else "unknown")
        reason = "; ".join(problems) if problems else (
            "firmware drop counter unavailable" if quality == "unknown" else "")
        self.last_capture_info = dict(
            quality=quality, quality_reason=reason,
            firmware_drop_before=before, firmware_drop_after=after,
            usb_seq_lost=self.lost, malformed_frames=self.bad,
            frame_seq=np.asarray(self.frame_seq, dtype=np.uint16),
            frame_sample_count=np.asarray(self.frame_count, dtype=np.uint8),
        )
        if strict and quality == "invalid":
            raise RuntimeError(f"incomplete Plot data: {reason}")
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
                  fft_n, windows, quality="unknown", quality_reason=""):
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
            quality=np.asarray([quality], dtype="<U16"),
            quality_reason=np.asarray([quality_reason], dtype="<U256"),
        )


def save_waveform(path, raw, group, names, fs, info, metadata=None):
    """Archive the unmodified USB Plot numeric samples and integrity evidence."""
    if path.exists():
        raise FileExistsError(path)
    dtype = "<i2" if group == "fast" else "<f4"
    samples = np.frombuffer(raw, dtype=dtype).reshape(-1, len(names))
    with path.open("wb") as handle:
        np.savez_compressed(
            handle,
            format_version=np.array([1], dtype=np.int32),
            kind=np.array(["waveform"]),
            group=np.array([group]),
            sample_rate_hz=np.array([fs], dtype=np.float64),
            raw_samples=samples,
            channel_id=np.asarray([CHANNELS[n][0] for n in names], dtype=np.uint16),
            channel_name=np.asarray(names, dtype="<U32"),
            channel_unit=np.asarray([CHANNELS[n][1] for n in names], dtype="<U32"),
            channel_scale=np.asarray([
                CHANNELS[n][2] if group == "fast" else 1.0 for n in names
            ], dtype=np.float64),
            frame_seq=info["frame_seq"],
            frame_sample_count=info["frame_sample_count"],
            usb_seq_lost=np.array([info["usb_seq_lost"]], dtype=np.int64),
            malformed_frames=np.array([info["malformed_frames"]], dtype=np.int64),
            firmware_drop_before=np.array([
                -1 if info["firmware_drop_before"] is None else info["firmware_drop_before"]
            ], dtype=np.int64),
            firmware_drop_after=np.array([
                -1 if info["firmware_drop_after"] is None else info["firmware_drop_after"]
            ], dtype=np.int64),
            quality=np.asarray([info["quality"]], dtype="<U16"),
            quality_reason=np.asarray([info["quality_reason"]], dtype="<U256"),
            captured_at=np.asarray([datetime.now().astimezone().isoformat()], dtype="<U40"),
            metadata_json=np.asarray([json.dumps(metadata or {}, ensure_ascii=False)]),
        )


def read_waveform(path):
    """Read a waveform archive for offline FFT/PSD, rejecting unverified timing."""
    with np.load(path, allow_pickle=False) as z:
        if int(z["format_version"][0]) != 1 or str(z["kind"][0]) != "waveform":
            raise ValueError("unsupported waveform NMIXX")
        if str(z["quality"][0]) != "valid":
            raise ValueError("FFT/PSD requires verified continuous waveform: "
                             + str(z["quality_reason"][0]))
        group = str(z["group"][0])
        names = tuple(map(str, z["channel_name"]))
        fs = float(z["sample_rate_hz"][0])
        raw = np.asarray(z["raw_samples"])
        expected = np.dtype("<i2" if group == "fast" else "<f4")
        if (group not in RATES or fs <= 0 or
            raw.ndim != 2 or raw.shape[1] != len(names) or
            raw.dtype.kind != expected.kind or raw.dtype.itemsize != expected.itemsize or
            not np.isfinite(raw).all() or len(raw) < 8):
            raise ValueError("invalid archived waveform data")
        # Python's built-in sum() preserves numpy.uint8 scalar arithmetic and
        # wraps above 255 samples. Widen before accumulating packet lengths.
        if int(np.sum(z["frame_sample_count"], dtype=np.int64)) != len(raw):
            raise ValueError("waveform packet/sample count mismatch")
        seq = z["frame_seq"].astype(np.int64)
        if (len(seq) != len(z["frame_sample_count"]) or
            np.any((np.diff(seq) & 0xFFFF) != 1) or
            int(z["usb_seq_lost"][0]) or int(z["malformed_frames"][0]) or
            int(z["firmware_drop_before"][0]) < 0 or
            int(z["firmware_drop_after"][0]) < 0 or
            int(z["firmware_drop_before"][0]) != int(z["firmware_drop_after"][0])):
            raise ValueError("waveform frame/drop integrity check failed")
        if tuple(CHANNELS[n][0] for n in names) != tuple(z["channel_id"]):
            raise ValueError("waveform channel IDs do not match current definitions")
        if not np.array_equal(z["channel_scale"], np.asarray([
            CHANNELS[n][2] if group == "fast" else 1.0 for n in names
        ], dtype=np.float64)):
            raise ValueError("waveform scales do not match channel definitions")
        return raw.astype(expected, copy=False).tobytes(), group, names, fs


def plot_spectrum(freq, values, names, units, kind, fs, n_samples, output,
                  quality="unknown", quality_reason=""):
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
        fig.suptitle(f"AxDr  {kind.upper()}  |  Fs={fs:g} Hz  |  N={n_samples}  |  {quality}",
                     y=.963, fontsize=13, fontweight="bold")
        ax.legend(loc="lower center", bbox_to_anchor=(.5, 1.015), ncol=min(4, len(names)),
                  fontsize=9, frameon=False)
        if quality != "valid":
            fig.text(.5, .018, quality_reason or "Historical/unverified signal continuity",
                     ha="center", fontsize=8)
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
        quality = str(saved["quality"][0]) if "quality" in saved.files else "unknown"
        reason = str(saved["quality_reason"][0]) if "quality_reason" in saved.files else "Historical archive: quality not recorded"
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
    plot_spectrum(freq, amp, names, units, kind, fs, n, image, quality, reason)
    print("Replotted:", image)


def main():
    ap = argparse.ArgumentParser(description="AxDr USB Plot waveform capture and FFT / Welch PSD")
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="live USB CDC device")
    source.add_argument("--input-raw", type=Path, help="legacy interleaved Plot samples")
    source.add_argument("--input-waveform", type=Path, help="verified .waveform.nmixx for offline spectrum")
    source.add_argument("--result", type=Path, help="replot an existing .fft.nmixx or .psd.nmixx")
    ap.add_argument("--group", choices=("fast", "normal"), default="fast")
    ap.add_argument("--channels", nargs="+", default=["id", "iq"])
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--analysis", choices=("fft", "psd", "both", "waveform"), default="both")
    ap.add_argument("--welch-n", type=int, default=8192)
    ap.add_argument("--name", default="Spectrum", help="test name prefix before local YYYY-MM-DD")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--save-raw", action="store_true", help="keep legacy raw bytes in local test-data")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--firmware-sha", help="optional user-supplied flashed firmware SHA (not verified)")
    ap.add_argument("--controller", help="optional PI/LADRC label")
    ap.add_argument("--rpm", type=float, help="optional target speed (mechanical rpm)")
    ap.add_argument("--pole-pairs", type=int, help="optional motor pole pairs")
    args = ap.parse_args()

    if args.out is None:
        args.out = (Path("test-data") / f"{date.today():%Y%m%d}_current_waveform"
                    if args.analysis == "waveform" else Path("test-data/fft"))
    if args.result is not None:
        draw_archive(args.result, args.out)
        return

    if (not np.isfinite(args.duration) or args.duration <= 0 or
        args.welch_n < 8 or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.name) or
        (args.rpm is not None and not np.isfinite(args.rpm)) or
        (args.pole_pairs is not None and args.pole_pairs <= 0)):
        ap.error("invalid duration, Welch length, test name, speed or pole pairs")
    if args.analysis == "waveform" and args.port is None:
        ap.error("--analysis waveform requires --port; offline input uses --input-waveform with FFT/PSD")
    if args.input_waveform is not None and args.analysis == "waveform":
        ap.error("do not recapture an existing waveform")

    if args.input_waveform is not None:
        raw, group, names, fs = read_waveform(args.input_waveform)
    else:
        try:
            names = channel_selection(args.group, args.channels)
        except ValueError as exc:
            ap.error(str(exc))
        group = args.group
        fs = RATES[group]
        raw = None

    prefix = f"{args.name}-{date.today().isoformat()}"
    selected = (("waveform",) if args.analysis == "waveform"
                else (("fft", "psd") if args.analysis == "both" else (args.analysis,)))
    for kind in selected:
        path = args.out / f"{prefix}.{kind}.nmixx"
        image = args.out / f"{prefix}.{kind}.png"
        if path.exists() or image.exists():
            ap.error(f"output already exists; choose a new --name or --out: {path}")
    args.out.mkdir(parents=True, exist_ok=True)

    info = None
    if args.port is not None:
        client = PlotClient(args.port, args.baud)
        try:
            raw = client.capture(group, names, args.duration,
                                 strict=(args.analysis != "waveform"))
            info = client.last_capture_info
        finally:
            client.close()
    elif args.input_raw is not None:
        raw = args.input_raw.read_bytes()
        print("Warning: legacy .raw has no frame/drop metadata; timing is unverified.")

    if args.analysis == "waveform":
        meta = dict(firmware_sha=args.firmware_sha or "",
                    controller=args.controller or "",
                    target_rpm=args.rpm,
                    pole_pairs=args.pole_pairs,
                    source="AxDr USB FAST/NORMAL Plot")
        path = args.out / f"{prefix}.waveform.nmixx"
        save_waveform(path, raw, group, names, fs, info, meta)
        print(f"Waveform: {path} ({info['quality']}: {info['quality_reason']})")
        print(f"Draw: python3 tools/current_waveform.py --input {path} --out {args.out / 'plots'}")
        return

    data = decode_samples(raw, group, names)
    if data.shape[1] < 8:
        raise RuntimeError("fewer than 8 valid samples")
    if info is not None and info["quality"] == "unknown":
        print("Warning: firmware drop counter unavailable; spectral timing is unverified.")
    if args.save_raw and args.port is not None:
        raw_path = args.out / f"{prefix}.raw"
        if raw_path.exists():
            ap.error(f"raw file already exists: {raw_path}")
        raw_path.write_bytes(raw)
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
        status = ("valid" if args.input_waveform is not None else
                  (info["quality"] if info is not None else "unknown"))
        reason = ("" if status == "valid" else
                  (info["quality_reason"] if info is not None else
                   "legacy raw without frame/drop metadata"))
        save_spectrum(archive, kind, group, names, fs, data.shape[1],
                      freq, spectrum, nfft, windows, status, reason)
        image = args.out / f"{prefix}.{kind}.png"
        plot_spectrum(freq, plotted, names, units, kind, fs, data.shape[1], image,
                      status, reason)
        print("Saved:", archive, "and", image)


if __name__ == "__main__":
    main()
