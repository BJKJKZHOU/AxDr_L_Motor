#!/usr/bin/env python3
"""Diagnose observer correction contribution to negative-sequence flux.

Captures Theta_e, Ia/Ib, Ucmd alpha/beta and Psi alpha/beta at FAST rate.
Flux_Err is reconstructed offline from the same firmware equation to avoid
int16 FAST-plot saturation:

    Flux_Err = Flux^2 - PsiAlpha^2 - PsiBeta^2
    Gamma = 2*pi*FLUX_OBS_BW_HZ / Flux^2
    Corr = 0.5 * Gamma * Flux_Err
    CorrPsi = Corr * Psi

Then the negative-sequence voltage-model prediction is evaluated as:

    Lambda_- = (U_- - Rs*I_- + CorrPsi_-) / (-j*We)
    Psi_-    = Lambda_- - Ls*I_-
"""

import math
import signal
import struct
import time

import sequence_diag as seq
import sensorless_run
import sensorless_test as base


FLUX_OBS_BW_HZ = 200.0
FAST_CONFIG_ID = 17
FAST_VARS = (
    ("Theta_e", 0x0014, 0.0002),
    ("Ia", 0x0001, 0.001),
    ("Ib", 0x0002, 0.001),
    ("Ualpha", 0x0015, 0.001),
    ("Ubeta", 0x0016, 0.001),
    ("PsiAlpha", 0x0024, 1.0e-6),
    ("PsiBeta", 0x0025, 1.0e-6),
)


def angle_deg(z):
    return math.degrees(math.atan2(z.imag, z.real))


def angle_diff_deg(a, b):
    value = angle_deg(a) - angle_deg(b)
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def neg_complex(metrics):
    return complex(metrics["negative_real"], metrics["negative_imag"])


def print_vec(name, z, unit):
    print(
        f"  {name:20s} mag={abs(z):12.7f} {unit:<3s} "
        f"phase={angle_deg(z):8.2f} deg re={z.real:+.7f} im={z.imag:+.7f}"
    )


class CorrectionRun(seq.SequenceRun):
    def configure_plot(self):
        self.fast_last = None
        self.fast_lost = 0
        self.normal_last = None
        self.normal_lost = 0
        self.samples = []
        self.fast_saturation = [0] * len(FAST_VARS)

        fast_data = bytes([base.FAST_GROUP, FAST_CONFIG_ID, len(FAST_VARS)])
        fast_data += b"".join(struct.pack("<H", var_id) for _, var_id, _ in FAST_VARS)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, fast_data)

        normal_data = bytes([base.NORMAL_GROUP, sensorless_run.NORMAL_CONFIG_ID, 1])
        normal_data += struct.pack("<H", seq.VBUS_ID)
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(base.MSG_PLOT, base.PLOT_START, bytes([base.FAST_MASK | base.NORMAL_MASK]))
        self.monitor_started = time.monotonic()

    def process_fast(self, payload):
        if len(payload) < 4 or payload[2] != FAST_CONFIG_ID:
            return
        (frame_seq,) = struct.unpack_from("<H", payload, 0)
        sample_count = payload[3]
        count = len(FAST_VARS)
        if len(payload) != 4 + sample_count * count * 2:
            return

        if self.fast_last is not None:
            expected = (self.fast_last + 1) & 0xFFFF
            self.fast_lost += (frame_seq - expected) & 0xFFFF
        self.fast_last = frame_seq
        self.fast_frames += 1
        self.fast_samples += sample_count
        self.last_fast_rx = time.monotonic()

        raw = struct.unpack_from(f"<{sample_count * count}h", payload, 4)
        for sample in range(sample_count):
            start = sample * count
            row = {}
            for index, (name, _, scale) in enumerate(FAST_VARS):
                code = raw[start + index]
                if code in (-32768, 32767):
                    self.fast_saturation[index] += 1
                row[name] = code * scale
            self.current_peak = max(self.current_peak, abs(row["Ia"]), abs(row["Ib"]))
            if self.current_peak > self.args.current_limit:
                self.current_trip = True
            self.samples.append(row)

    def guard(self, require_run=False):
        del require_run
        now = time.monotonic()
        if self.current_trip:
            raise RuntimeError(f"phase current exceeded {self.args.current_limit:.3f} A")
        if any(self.fast_saturation):
            names = [FAST_VARS[i][0] for i, count in enumerate(self.fast_saturation) if count]
            raise RuntimeError(f"FAST quantizer saturated: {names}")
        if self.fast_lost > self.args.max_lost:
            raise RuntimeError(f"FAST lost {self.fast_lost} frames")
        if self.normal_lost > self.args.max_lost:
            raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")
        if self.monitor_started is not None and self.last_fast_rx is None and now - self.monitor_started > 0.5:
            raise RuntimeError("no FAST diagnostic data")
        if self.last_fast_rx is not None and now - self.last_fast_rx > 0.5:
            raise RuntimeError("FAST diagnostic timeout")
        if self.last_normal_rx is not None and now - self.last_normal_rx > 1.0:
            raise RuntimeError("Vbus monitor timeout")
        if self.vbus:
            vbus = self.vbus[-1]
            if not self.args.vbus_min <= vbus <= self.args.vbus_max:
                raise RuntimeError(
                    f"Vbus {vbus:.3f} V outside {self.args.vbus_min:.3f} .. {self.args.vbus_max:.3f} V"
                )


def correction_metrics(rows, flux):
    gamma = 2.0 * math.pi * FLUX_OBS_BW_HZ / (flux * flux)
    mapped = []
    flux_err_values = []
    for row in rows:
        psi_alpha = row["PsiAlpha"]
        psi_beta = row["PsiBeta"]
        flux_err = flux * flux - psi_alpha * psi_alpha - psi_beta * psi_beta
        corr = 0.5 * gamma * flux_err
        flux_err_values.append(flux_err)
        mapped.append(
            {
                "Theta_e": row["Theta_e"],
                "CorrPsiAlpha": corr * psi_alpha,
                "CorrPsiBeta": corr * psi_beta,
            }
        )
    return gamma, seq.sequence_metrics(mapped, "CorrPsiAlpha", "CorrPsiBeta"), flux_err_values


def main():
    args = seq.parse_args()
    stop = sensorless_run.StopRequest()
    signal.signal(signal.SIGINT, stop.handle)
    signal.signal(signal.SIGTERM, stop.handle)

    test = None
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003, write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
            test = CorrectionRun(ser, args, stop)
            try:
                test.prepare()
                time.sleep(0.05)
                test.motor_para_set()
                test.current_limit_set()
                test.current_gain_set()
                test.pll_bw_set(args.pll_bw_hz)
                test.shadow_set(True, quiet=True)
                test.configure_plot()
                test.check_vbus()

                wm = args.we / args.pole_pairs
                test.request(base.MSG_CONTROL, base.CTRL_MODE_SET, bytes([base.MODE_SENSORLESS_SPEED]))
                test.speed_set(wm)
                test.request(base.MSG_CONTROL, base.CTRL_ENABLE)
                test.request(base.MSG_CONTROL, base.CTRL_RUN)
                print(
                    f"Observer correction diagnostic: We={args.we:.1f} rad/s, "
                    f"PLL BW={args.pll_bw_hz:.1f} Hz"
                )
                print("ALIGN -> I/F -> shadow; collecting I/U/Psi at FAST rate")
                test.wait_shadow()
                rows = test.collect()

                current = seq.current_sequence_metrics(rows)
                voltage = seq.sequence_metrics(rows, "Ualpha", "Ubeta")
                flux_m = seq.sequence_metrics(rows, "PsiAlpha", "PsiBeta")
                gamma, correction, flux_err_values = correction_metrics(rows, args.flux)

                i_neg = neg_complex(current)
                u_neg = neg_complex(voltage)
                psi_meas = neg_complex(flux_m)
                corr_neg = neg_complex(correction)
                rs_i = args.rs * i_neg
                drive_no_corr = u_neg - rs_i
                drive_full = drive_no_corr + corr_neg
                lambda_no_corr = drive_no_corr / complex(0.0, -args.we)
                lambda_full = drive_full / complex(0.0, -args.we)
                ls_i = args.ld * i_neg
                psi_no_corr = lambda_no_corr - ls_i
                psi_full = lambda_full - ls_i
                residual_no_corr = psi_meas - psi_no_corr
                residual_full = psi_meas - psi_full

                print(f"Gamma={gamma:.6e} 1/(Wb^2*s)")
                print(
                    f"Flux_Err offline: mean={seq.mean(flux_err_values):+.6e} Wb^2 "
                    f"rms={seq.rms(flux_err_values):.6e} Wb^2"
                )
                print("\nNegative-sequence complex vectors")
                print_vec("I_-", i_neg, "A")
                print_vec("Ucmd_-", u_neg, "V")
                print_vec("Rs*I_-", rs_i, "V")
                print_vec("CorrPsi_-", corr_neg, "V")
                print_vec("U-RsI", drive_no_corr, "V")
                print_vec("U-RsI+CorrPsi", drive_full, "V")
                print_vec("Psi pred no corr", psi_no_corr, "Wb")
                print_vec("Psi pred full", psi_full, "Wb")
                print_vec("Psi measured", psi_meas, "Wb")

                print("\nPrediction error")
                for name, pred, residual in (
                    ("without correction", psi_no_corr, residual_no_corr),
                    ("with correction", psi_full, residual_full),
                ):
                    mag_err = abs(pred) - abs(psi_meas)
                    mag_pct = 100.0 * mag_err / abs(psi_meas) if abs(psi_meas) > 1e-15 else math.nan
                    phase_err = angle_diff_deg(pred, psi_meas)
                    residual_pct = (
                        100.0 * abs(residual) / abs(psi_meas) if abs(psi_meas) > 1e-15 else math.nan
                    )
                    print(
                        f"  {name:18s}: mag error={mag_err * 1e3:+.6f} mWb "
                        f"({mag_pct:+.2f}%)  phase error={phase_err:+.2f} deg  "
                        f"complex residual={residual_pct:.2f}%"
                    )

            finally:
                try:
                    test.stop_all(False)
                except (TimeoutError, RuntimeError):
                    pass
                time.sleep(0.05)
                try:
                    test.shadow_set(False, quiet=True)
                except (TimeoutError, RuntimeError):
                    pass

    except (base.serial.SerialException, OSError, TimeoutError, RuntimeError, KeyboardInterrupt) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
