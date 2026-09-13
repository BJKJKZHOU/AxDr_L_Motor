#!/usr/bin/env python3
"""Run identification with a minimal Flux handover capture.

This diagnostic reuses identification_test.py but disables the 20 kHz current
stream during Flux identification. Flux capture contains only two 1 kHz float
channels:

- Theta_e: angle actually used by FOC
- Theta_Obs: Identification PLL angle

The private plot ID 0xF001 is intentionally not part of Parameter Core.
"""

import struct

import identification_test as base


PLOT_DEBUG_IDENT_THETA_ID = 0xF001
HANDOVER_NORMAL_VARS = (
    ("Theta_e", base.PARAM_RUN_THETA_E),
    ("Theta_Obs", PLOT_DEBUG_IDENT_THETA_ID),
)
HANDOVER_NORMAL_UNITS = {
    "Theta_e": "rad",
    "Theta_Obs": "rad",
}


class HandoverIdentificationClient(base.IdentificationClient):
    def configure_plot(self, flux=False):
        self.fast_last = None
        self.normal_last = None
        self.fast_lost = 0
        self.normal_lost = 0
        self.fast_vars = ()
        self.normal_vars = HANDOVER_NORMAL_VARS if flux else base.VBUS_NORMAL_VARS

        normal_data = bytes(
            [base.NORMAL_GROUP, base.NORMAL_CONFIG_ID, len(self.normal_vars)]
        )
        normal_data += b"".join(
            struct.pack("<H", var_id) for _, var_id in self.normal_vars
        )
        self.request(base.MSG_PLOT, base.PLOT_CONFIG, normal_data)
        self.request(
            base.MSG_PLOT,
            base.PLOT_START,
            bytes([base.NORMAL_MASK]),
        )
        self.plot_started = True

    def stop_plot(self):
        if not self.plot_started:
            return
        self.request(
            base.MSG_PLOT,
            base.PLOT_STOP,
            bytes([base.NORMAL_MASK]),
        )
        self.plot_started = False

    def process_fast(self, payload):
        del payload

    def process_normal(self, payload):
        count = len(self.normal_vars)
        if (
            len(payload) < 4
            or payload[2] != base.NORMAL_CONFIG_ID
            or payload[3] != count
        ):
            return

        expected_length = 4 + count * 4
        payload = base.canfd_payload(payload, expected_length)
        if payload is None:
            return

        seq, = struct.unpack_from("<H", payload, 0)
        if self.normal_last is not None:
            expected = (self.normal_last + 1) & 0xFFFF
            self.normal_lost += (seq - expected) & 0xFFFF
        self.normal_last = seq
        self.normal_frames += 1

        values = struct.unpack_from(f"<{count}f", payload, 4)
        if self.normal_vars == base.VBUS_NORMAL_VARS:
            self.vbus.append(values[0])

        if not self.ident_active or self.flux_state_data is None:
            return

        self.flux_state_data.extend(payload[4:expected_length])
        self.flux_normal_sample += 1

    def flux_plot_start(self, run_number, direction):
        state_path = (
            self.args.capture_dir
            / f"flux_{direction}_{run_number:02d}_handover.axdr"
        )
        state_path.parent.mkdir(parents=True, exist_ok=True)

        self.flux_current_path = None
        self.flux_current_data = None
        self.flux_state_path = state_path
        self.flux_state_data = bytearray()
        self.flux_fast_sample = 0
        self.flux_normal_sample = 0

        # The base runner expects a two-element tuple. Both entries point to the
        # state path; run_ident() removes current_binary from the result below.
        return state_path, state_path

    def flux_plot_stop(self):
        if self.flux_state_data is not None:
            base.capture_write(
                self.flux_state_path,
                {
                    "capture": "flux_handover",
                    "channels": [
                        {"name": name, "unit": HANDOVER_NORMAL_UNITS[name]}
                        for name, _ in HANDOVER_NORMAL_VARS
                    ],
                    "dtype": "<f4",
                    "sample_count": self.flux_normal_sample,
                    "sample_rate_hz": base.NORMAL_RATE_HZ,
                    "version": 1,
                },
                self.flux_state_data,
            )

        self.flux_current_path = None
        self.flux_current_data = None
        self.flux_state_path = None
        self.flux_state_data = None

    def run_ident(self, *args, **kwargs):
        try:
            result = super().run_ident(*args, **kwargs)
        except base.IdentificationFailed as exc:
            exc.result.pop("current_binary", None)
            exc.result["fast_samples"] = 0
            exc.result["fast_sample_rate_hz"] = 0.0
            raise

        result.pop("current_binary", None)
        result["fast_samples"] = 0
        result["fast_sample_rate_hz"] = 0.0
        return result


def run_group(client, mode, count, args, results, direction=None, run_offset=0):
    name = "Rs/Ls" if mode == base.IDENT_RS_LS else f"Flux {direction}"

    for index in range(1, count + 1):
        run = run_offset + index
        try:
            result = client.run_ident(mode, run, direction=direction)
        except base.IdentificationFailed as exc:
            result = exc.result
            results.append(result)
            base.result_print(name, index, count, mode, result)
            if result.get("state_binary"):
                print(f"  Handover binary: {result['state_binary']}")
            if args.stop_on_error:
                return False
        else:
            results.append(result)
            base.result_print(name, index, count, mode, result)
            if mode == base.IDENT_FLUX and result.get("state_binary"):
                print(f"  Handover binary: {result['state_binary']}")

        if args.interval > 0.0 and index < count:
            base.time.sleep(args.interval)

    return True


def main():
    # Keep the normal identification workflow and CLI intact while replacing
    # only its capture transport for this diagnostic run.
    base.IdentificationClient = HandoverIdentificationClient
    base.run_group = run_group
    base.CURRENT_FAST_VARS = ()
    base.FLUX_NORMAL_VARS = HANDOVER_NORMAL_VARS
    base.FLUX_NORMAL_UNITS = HANDOVER_NORMAL_UNITS
    base.main()


if __name__ == "__main__":
    main()
