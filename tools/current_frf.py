#!/usr/bin/env python3
"""Capture, analyze, plot and offline-replot current-loop FRF from one tool.

Requirements: pip install pyserial numpy
The motor must be manually set to TORQUE + ENABLED before running. This tool
never sends Enable, Run, or a persistent Save. Each frequency starts a finite
sine experiment, captures raw FAST samples, then measures Id/IdRef and UdPI/IdRef.
"""

import argparse
import json
import math
from pathlib import Path
import struct
import time

import numpy as np


def responses(ref, fbk, output):
    """Return complex T, S, C, G, L, H_U from measured same-clock phasors."""
    ref, fbk, output = complex(ref), complex(fbk), complex(output)
    error = ref - fbk
    if abs(ref) < 1e-12:
        raise ValueError("reference phasor is zero")
    if abs(error) < 1e-12 or abs(output) < 1e-12:
        raise ValueError("error or voltage phasor is too small for L/G measurement")
    return {
        "T": fbk / ref,
        "S": error / ref,
        "C": output / error,
        "G": fbk / output,
        "L": fbk / error,
        "HU": output / ref,
    }


def bode_arrays(freq_hz, *signals):
    f = np.asarray(freq_hz, dtype=float)
    if f.ndim != 1 or len(f) < 2 or not np.all(np.isfinite(f)) or np.any(f <= 0):
        raise ValueError("need at least two positive finite frequencies")
    if any(len(s) != len(f) for s in signals):
        raise ValueError("frequency and response lengths differ")
    order = np.argsort(f)
    f = f[order]
    if np.any(np.diff(f) <= 0):
        raise ValueError("frequency points must be unique")
    outputs = []
    for signal in signals:
        h = np.asarray(signal, dtype=np.complex128)[order]
        if not np.all(np.isfinite(h)):
            raise ValueError("response contains invalid complex numbers")
        gain = 20 * np.log10(np.maximum(np.abs(h), 1e-20))
        phase = np.rad2deg(np.unwrap(np.angle(h)))
        outputs.append((gain, phase))
    return f, outputs


def _crossings(freq, values, target, direction=None):
    """Log-frequency interpolate every measured threshold crossing."""
    out = []
    for i in range(1, len(freq)):
        a, b = values[i - 1] - target, values[i] - target
        if b == a:
            continue
        if direction == "down" and not (a >= 0 > b):
            continue
        if direction == "up" and not (a <= 0 < b):
            continue
        if direction is None and not (a * b <= 0):
            continue
        frac = -a / (b - a)
        if 0 <= frac <= 1:
            log_f = np.log(freq[i - 1]) + frac * (np.log(freq[i]) - np.log(freq[i - 1]))
            out.append((float(np.exp(log_f)), i - 1, float(frac)))
    return out


def bode_metrics(freq_hz, open_loop, closed_loop, low_frequency_points=3):
    """Report only margins/bandwidth with an observed crossing in the sweep."""
    f, ((l_db, l_phase), (t_db, _)) = bode_arrays(freq_hz, open_loop, closed_loop)
    low_db = float(np.mean(t_db[:min(low_frequency_points, len(t_db))]))
    threshold = low_db - 3.0
    cutoffs = _crossings(f, t_db, threshold, direction="down")
    unity = _crossings(f, l_db, 0.0, direction="down")
    phase_crossings = _crossings(f, l_phase, -180.0, direction="down")
    phase_margin = [(180.0 + float(l_phase[i] + frac * (l_phase[i + 1] - l_phase[i])), freq)
                    for freq, i, frac in unity]
    gain_margin = [(-float(l_db[i] + frac * (l_db[i + 1] - l_db[i])), freq)
                   for freq, i, frac in phase_crossings]
    pm = min(phase_margin, key=lambda item: item[0]) if phase_margin else None
    gm = min(gain_margin, key=lambda item: item[0]) if gain_margin else None
    return {
        "low_frequency_reference_db": low_db,
        "relative_minus_3db_threshold_db": threshold,
        "relative_minus_3db_hz": cutoffs[0][0] if cutoffs else None,
        "relative_minus_3db_all_hz": [item[0] for item in cutoffs],
        "gain_crossover_hz": pm[1] if pm else None,
        "phase_margin_deg": pm[0] if pm else None,
        "phase_crossing_hz": gm[1] if gm else None,
        "gain_margin_db": gm[0] if gm else None,
        "min_frequency_hz": float(f[0]),
        "max_frequency_hz": float(f[-1]),
    }


def plot_bode_a4(freq_hz, open_loop, sensitivity, closed_loop, title, destination):
    """Render L/S/T magnitude + phase on an A4 landscape canvas (PNG + SVG)."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import LogLocator, NullFormatter, MultipleLocator

    f, spectra = bode_arrays(freq_hz, open_loop, sensitivity, closed_loop)
    metrics = bode_metrics(freq_hz, open_loop, closed_loop)
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax_mag, ax_phase) = plt.subplots(
        2, 1, sharex=True, figsize=(11.69, 8.27), dpi=180
    )
    labels = ("Open Loop L", "Sensitivity S", "Closed Loop T")
    colors = ("#367EBC", "#D86B43", "#DCA832")
    for (gain, phase), label, color in zip(spectra, labels, colors):
        ax_mag.semilogx(f, gain, label=label, lw=1.25, color=color)
        ax_phase.semilogx(f, phase, label=label, lw=1.25, color=color)

    for axis in (ax_mag, ax_phase):
        axis.set_xscale("log")
        axis.set_xlim(f[0], f[-1])
        axis.xaxis.set_major_locator(LogLocator(base=10, numticks=15))
        axis.xaxis.set_minor_locator(LogLocator(base=10, subs=np.arange(2, 10), numticks=100))
        axis.xaxis.set_minor_formatter(NullFormatter())
        axis.grid(which="major", color="0.6", lw=0.60, alpha=0.40)
        axis.grid(which="minor", color="0.6", ls=":", lw=0.45, alpha=0.45)
        axis.tick_params(which="major", direction="in", top=True, right=True, length=6)
        axis.tick_params(which="minor", direction="in", top=True, right=True, length=3)

    ax_mag.yaxis.set_major_locator(MultipleLocator(5))
    ax_mag.yaxis.set_minor_locator(MultipleLocator(1))
    ax_phase.yaxis.set_major_locator(MultipleLocator(30))
    ax_phase.yaxis.set_minor_locator(MultipleLocator(5))
    for axis in (ax_mag, ax_phase):
        axis.grid(which="minor", axis="y", color="0.65", ls=":", lw=0.45, alpha=0.40)

    ax_mag.set_ylabel("Magnitude (dB)")
    ax_phase.set_ylabel("Phase (deg)")
    ax_phase.set_xlabel("Frequency (Hz)")
    ax_mag.legend(loc="lower left", fontsize=9, framealpha=0.92)

    # The reference remains visible when the sweep has no cutoff crossing.
    ax_mag.axhline(metrics["relative_minus_3db_threshold_db"],
                   color="0.40", ls="--", lw=0.9, alpha=0.75)
    bw = metrics["relative_minus_3db_hz"]
    if bw is not None:
        for axis in (ax_mag, ax_phase):
            axis.axvline(bw, color="#D86B43", ls="--", lw=1.05, alpha=0.9)
        ax_mag.annotate(f"{bw:.0f} Hz",
                        xy=(bw, metrics["relative_minus_3db_threshold_db"]),
                        xytext=(8, 10), textcoords="offset points", fontsize=8,
                        color="#D86B43")
        bw_text = f"relative -3 dB bandwidth {bw:.3g} Hz"
    else:
        bw_text = f"relative -3 dB bandwidth not reached (<= {f[-1]:g} Hz)"

    pm = metrics["phase_margin_deg"]
    gm = metrics["gain_margin_db"]
    fc = metrics["gain_crossover_hz"]
    margin = (f"PM = {pm:.1f} deg" if pm is not None else "PM = N/A")
    margin += "   |   " + (f"GM = {gm:.1f} dB" if gm is not None else "GM = N/A (not measured)")
    margin += "   |   " + (f"fc = {fc:.3g} Hz" if fc is not None else "fc = N/A")

    fig.suptitle(f"{title}, {bw_text}", y=0.980, fontsize=12)
    fig.text(0.5, 0.948, margin, fontsize=10, ha="center", va="top", color="0.35")
    fig.subplots_adjust(left=0.075, right=0.985, bottom=0.087, top=0.895, hspace=0.12)
    fig.savefig(output, dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)
    return metrics


def closed_phasor(point):
    h = point["closed_loop_Id_over_IdRef"]
    return 10 ** (h["gain_db"] / 20.0) * np.exp(1j * np.deg2rad(h["phase_deg"]))


def replot(source, output):
    data = json.loads(Path(source).read_text(encoding="utf-8"))
    groups = {}
    for point in data["bode"]:
        groups.setdefault(point["bandwidth_hz"], []).append(point)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    metrics = {}
    for bw, points in sorted(groups.items()):
        points.sort(key=lambda p: p["frequency_hz"])
        freq = np.array([p["frequency_hz"] for p in points], dtype=float)
        T = np.array([closed_phasor(p) for p in points])
        S = 1 - T
        if np.any(abs(S) < 1e-10):
            raise ValueError("loop reconstruction unreliable near unity closed-loop gain")
        L = T / S
        metrics[str(bw)] = plot_bode_a4(
            freq, L, S, T, f"CURRENT Id sweep (PI {bw:g} Hz)",
            output / f"bode_a4_bw_{bw:g}.png"
        )
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    return metrics


try:
    import serial
except ImportError:
    serial = None  # Offline --result plotting does not need serial access.

from parameter_ids_generated import (
    PARAM_MOTOR_MODE, PARAM_MOTOR_STATE,
    PARAM_RUN_ID, PARAM_SIGNAL_ID_REF, PARAM_SIGNAL_ID_PI_OUT,
    PARAM_SIGNAL_OUT, PARAM_SIGNAL_FREQ_HZ, PARAM_SIGNAL_AMP_A,
    PARAM_SIGNAL_TIME_S, PARAM_CTRL_CURRENT_FF_ENABLE,
    ACTION_SIGNAL_START, ACTION_SIGNAL_ABORT,
)

MAGIC = b"AXDR"
NODE_ID = 1
MSG_RESPONSE = 0x02
MSG_PLOT = 0x04
MSG_PARAMETER = 0x07
MSG_FAST_DATA = 0x18
PARAM_READ = 1
PARAM_WRITE = 2
PARAM_FLOAT = 2
PARAM_ACTION = 6
PLOT_CONFIG = 1
PLOT_START = 2
PLOT_STOP = 3
FAST_RATE_HZ = 20000.0
FAST_CONFIG_ID = 21
CHANNELS = (
    ("Injection", PARAM_SIGNAL_OUT, 0.001),
    ("IdRef", PARAM_SIGNAL_ID_REF, 0.001),
    ("Id", PARAM_RUN_ID, 0.001),
    ("UdPI", PARAM_SIGNAL_ID_PI_OUT, 0.001),
)
CANFD_LEN = tuple(range(9)) + (12, 16, 20, 24, 32, 48, 64)


def wire(msg_type, payload):
    length = next(n for n in CANFD_LEN if n >= len(payload))
    return MAGIC + struct.pack("<HB", (msg_type << 6) | NODE_ID, length) + payload.ljust(length, b"\x00")


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 7:
                break
            msg_id, size = struct.unpack_from("<HB", self.buffer, 4)
            if msg_id > 0x07FF or size not in CANFD_LEN:
                del self.buffer[0]
                continue
            end = 7 + size
            if len(self.buffer) < end:
                break
            frames.append((msg_id >> 6, bytes(self.buffer[7:end])))
            del self.buffer[:end]
        return frames


class Client:
    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError("pyserial is required for hardware capture")
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=1.0)
        self.parser = Parser()
        self.txn = 0
        self.fast = bytearray()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = False

    def close(self):
        self.ser.close()

    def poll(self):
        responses = []
        for msg_type, payload in self.parser.feed(self.ser.read(4096)):
            if msg_type == MSG_FAST_DATA and self.collect:
                if len(payload) < 4:
                    self.malformed += 1
                    continue
                seq, = struct.unpack_from("<H", payload)
                n, config_id = payload[3], payload[2]
                needed = 4 + n * len(CHANNELS) * 2
                if config_id != FAST_CONFIG_ID or n == 0 or len(payload) < needed:
                    self.malformed += 1
                    continue
                if self.last_seq is not None:
                    self.dropped += (seq - self.last_seq - 1) & 0xFFFF
                self.last_seq = seq
                self.fast.extend(payload[4:needed])
            elif msg_type == MSG_RESPONSE:
                responses.append(payload)
        return responses

    def request(self, msg_type, operation, body=b"", timeout=2.0):
        self.txn = (self.txn % 255) + 1
        txn = self.txn
        self.ser.write(wire(msg_type, bytes([txn, operation]) + body))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for payload in self.poll():
                if len(payload) < 4:
                    continue
                if payload[0] != txn or payload[1] != msg_type or payload[2] != operation:
                    continue
                if payload[3] != 0:
                    raise RuntimeError(f"request {msg_type:02X}/{operation:02X} status {payload[3]}")
                return payload[4:]
        raise TimeoutError(f"request {msg_type:02X}/{operation:02X} timed out")

    def read_u8(self, pid):
        payload = self.request(MSG_PARAMETER, PARAM_READ, struct.pack("<H", pid))
        if len(payload) < 4 or struct.unpack_from("<H", payload)[0] != pid or payload[2] != 0:
            raise RuntimeError(f"unexpected value reply for 0x{pid:04X}")
        return payload[3]

    def set_float(self, pid, value):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HBf", pid, PARAM_FLOAT, value))

    def action(self, pid):
        self.request(MSG_PARAMETER, PARAM_WRITE, struct.pack("<HB", pid, PARAM_ACTION))

    def plot_start(self):
        vars_payload = bytes([0, FAST_CONFIG_ID, len(CHANNELS)])
        vars_payload += b"".join(struct.pack("<H", pid) for _, pid, _ in CHANNELS)
        self.request(MSG_PLOT, PLOT_CONFIG, vars_payload)
        self.fast.clear()
        self.last_seq = None
        self.dropped = 0
        self.malformed = 0
        self.collect = True
        self.request(MSG_PLOT, PLOT_START, b"\x01")

    def plot_stop(self):
        self.request(MSG_PLOT, PLOT_STOP, b"\x01")
        self.collect = False


def sinusoid_fit(signal, freq, times):
    # Include DC as a separate fit parameter to avoid leakage from offsets.
    w = 2.0 * math.pi * freq
    basis = np.column_stack((np.sin(w * times), np.cos(w * times), np.ones(len(times))))
    coef = np.linalg.lstsq(basis, signal, rcond=None)[0]
    residue = signal - basis @ coef
    # Phasor in cosine convention; ratios do not depend on convention.
    return complex(coef[1], -coef[0]), float(np.sqrt(np.mean(residue ** 2)))


def analyze(raw, freq, amp, settle_cycles):
    data = np.frombuffer(raw, dtype="<i2")
    if len(data) % len(CHANNELS) != 0:
        raise RuntimeError("FAST sample buffer is not a whole number of records")
    raw_matrix = data.reshape(-1, len(CHANNELS))
    samples = raw_matrix.astype(np.float64) * np.array([c[2] for c in CHANNELS])
    inj = samples[:, 0]
    active = np.flatnonzero(np.abs(inj) > max(0.001, amp * 0.02))
    if len(active) < 50:
        raise RuntimeError("sine excitation missing from FAST Plot")
    start = int(active[0])
    end = int(active[-1]) + 1
    discard = int(round(settle_cycles * FAST_RATE_HZ / freq))
    start += discard
    if end - start < max(100, int(round(2.0 * FAST_RATE_HZ / freq))):
        raise RuntimeError("too few settled cycles; increase --duration")
    interval = samples[start:end]
    t = np.arange(len(interval), dtype=np.float64) / FAST_RATE_HZ
    ref, ref_err = sinusoid_fit(interval[:, 1], freq, t)
    current, cur_err = sinusoid_fit(interval[:, 2], freq, t)
    pi_out, out_err = sinusoid_fit(interval[:, 3], freq, t)
    if abs(ref) < max(0.005, amp * 0.2):
        raise RuntimeError("injected IdRef amplitude too small")
    frf = responses(ref, current, pi_out)
    loop = frf["T"]
    control = frf["HU"]

    def response(value):
        return {
            "gain_db": float(20.0 * np.log10(max(abs(value), 1e-20))),
            "phase_deg": float(np.angle(value, deg=True)),
        }

    return raw_matrix, {
        "frequency_hz": freq,
        "amplitude_requested_a": amp,
        "input_amplitude_measured_a": float(abs(ref)),
        "closed_loop_Id_over_IdRef": response(loop),
        "controller_UdPI_over_IdRef": response(control),
        "open_loop_L": response(frf["L"]),
        "sensitivity_S": response(frf["S"]),
        "plant_G": response(frf["G"]),
        "controller_C": response(frf["C"]),
        "phasors": {
            "IdRef": [float(ref.real), float(ref.imag)],
            "Id": [float(current.real), float(current.imag)],
            "UdPI": [float(pi_out.real), float(pi_out.imag)],
        },
        "fit_residual_rms": {
            "id_ref_a": ref_err, "id_a": cur_err, "ud_pi_v": out_err,
        },
        "used_samples": len(interval),
        "captured_samples": len(samples),
        "sample_rate_hz": FAST_RATE_HZ,
        "settle_cycles": settle_cycles,
    }


def main():
    p = argparse.ArgumentParser(description="AxDr sensored d-axis current-loop sine FRF")
    p.add_argument("--port", help="USB CDC port, e.g. /dev/ttyACM0 or COM5")
    p.add_argument("--result", type=Path, help="replot archived result.json without connecting hardware")
    p.add_argument("--baud", type=int, default=115200,
                   help="USB CDC line coding; not a UART bandwidth limit")
    p.add_argument("--freq", type=float, nargs="+", help="Frequency points in Hz")
    p.add_argument("--amp", type=float, default=0.1, help="Injected peak current, A")
    p.add_argument("--duration", type=float, default=2.0, help="Seconds per point, max 60")
    p.add_argument("--settle-cycles", type=float, default=2.0, help="Initial periods excluded")
    p.add_argument("--out", type=Path, default=Path("current_frf_capture"))
    p.add_argument("--plot", action="store_true", help="write bode.png (requires matplotlib)")
    args = p.parse_args()
    if args.result is not None:
        if args.port or args.freq:
            p.error("--result is an offline mode; do not pass --port or --freq")
        metrics = replot(args.result, args.out)
        for bw, values in metrics.items():
            print(f"PI {bw} Hz: -3dB={values['relative_minus_3db_hz']}, "
                  f"PM={values['phase_margin_deg']}, GM={values['gain_margin_db']}")
        print("Results:", args.out)
        return
    if not args.port:
        p.error("hardware capture requires --port")
    if (not args.freq or any(not math.isfinite(f) or f <= 0 or f > 5000 for f in args.freq) or
        not math.isfinite(args.amp) or args.amp <= 0 or
        not math.isfinite(args.duration) or not 0 < args.duration <= 60 or
        not math.isfinite(args.settle_cycles) or args.settle_cycles < 0):
        p.error("invalid frequency, amplitude, duration or settle cycles")

    args.out.mkdir(parents=True, exist_ok=True)
    client = Client(args.port, args.baud)
    results = []
    plotting = False
    try:
        if client.read_u8(PARAM_MOTOR_MODE) != 0 or client.read_u8(PARAM_MOTOR_STATE) != 1:
            raise RuntimeError("set TORQUE mode and ENABLED state before testing; script will not Enable/Run")
        if client.read_u8(PARAM_CTRL_CURRENT_FF_ENABLE) != 0:
            raise RuntimeError("turn off current dq feedforward before baseline current FRF")
        for i, freq in enumerate(args.freq):
            if client.read_u8(PARAM_MOTOR_STATE) != 1:
                raise RuntimeError("motor left ENABLED state")
            client.set_float(PARAM_SIGNAL_FREQ_HZ, freq)
            client.set_float(PARAM_SIGNAL_AMP_A, args.amp)
            client.set_float(PARAM_SIGNAL_TIME_S, args.duration)
            client.plot_start()
            plotting = True
            try:
                client.action(ACTION_SIGNAL_START)
                end = time.monotonic() + args.duration + 0.25
                while time.monotonic() < end:
                    client.poll()
                client.action(ACTION_SIGNAL_ABORT)
                client.plot_stop()
                plotting = False
            finally:
                # Abort is idempotent and does not alter Motor State.
                client.action(ACTION_SIGNAL_ABORT)

            if client.dropped or client.malformed:
                raise RuntimeError(f"FAST data incomplete: seq_lost={client.dropped}, malformed={client.malformed}")
            raw, result = analyze(client.fast, freq, args.amp, args.settle_cycles)
            result["channels"] = [
                {"name": name, "parameter_id": pid, "scale": scale}
                for name, pid, scale in CHANNELS
            ]
            result["fast_frames_lost"] = client.dropped
            raw_path = args.out / f"point_{i:03d}.raw"
            raw_path.write_bytes(client.fast)
            result["raw_file"] = raw_path.name
            results.append(result)
            print(f"{freq:g} Hz: Id/IdRef={result['closed_loop_Id_over_IdRef']['gain_db']:+.2f} dB, "
                  f"phase={result['closed_loop_Id_over_IdRef']['phase_deg']:+.1f} deg, "
                  f"samples={result['used_samples']}")

        (args.out / "bode.json").write_text(
            json.dumps({"measure": "sensored_d_current_closed_loop",
                        "sample_rate_hz": FAST_RATE_HZ, "points": results},
                       ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        if len(results) >= 2:
            def sample_phasor(point, name):
                return complex(*point["phasors"][name])

            transfer = [
                responses(sample_phasor(point, "IdRef"),
                          sample_phasor(point, "Id"),
                          sample_phasor(point, "UdPI"))
                for point in results
            ]
            freq = np.array([point["frequency_hz"] for point in results])
            L = np.array([item["L"] for item in transfer])
            S = np.array([item["S"] for item in transfer])
            T = np.array([item["T"] for item in transfer])
            if args.plot:
                margins = plot_bode_a4(
                    freq, L, S, T, "CURRENT Id sweep", args.out / "bode.png"
                )
            else:
                margins = bode_metrics(freq, L, T)
            (args.out / "bode_metrics.json").write_text(
                json.dumps(margins, indent=2) + "\n", encoding="utf-8"
            )
        elif args.plot:
            print("Bode plot requires at least two frequency points; raw capture saved.")
        print("Results:", args.out / "bode.json")
    finally:
        try:
            client.action(ACTION_SIGNAL_ABORT)
        except (OSError, RuntimeError, TimeoutError):
            pass
        if plotting:
            try:
                client.plot_stop()
            except (OSError, RuntimeError, TimeoutError):
                pass
        client.close()


if __name__ == "__main__":
    main()
