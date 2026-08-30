#!/usr/bin/env python3
"""Diagnose a sensorless I/F -> observer handover without changing firmware."""

import struct
import time

import sensorless_run
import sensorless_test as base


# Reuse the normal guarded runner, adding the observer flux-magnitude error that
# is already exposed by the firmware Plot registry as variable 0x0023. Use a
# wider host scale so startup transients do not saturate the 16-bit transport.
sensorless_run.FAST_VARS = sensorless_run.FAST_VARS + (
    ("Flux_Err", 0x0023, 1.0e-8),
)
sensorless_run.FAST_INDEX = {
    name: index
    for index, (name, _, _) in enumerate(sensorless_run.FAST_VARS)
}


class HandoverDiag(sensorless_run.SensorlessRun):
    def __init__(self, ser, args, stop):
        super().__init__(ser, args, stop)
        self.we_if = 0.0
        self.handover_started = False

    def read_stage(self):
        data = self.request(base.MSG_SENSORLESS, base.SENSORLESS_STATUS)
        if len(data) != 8:
            raise RuntimeError(f"invalid Sensorless status length: {len(data)}")

        active, ready, stage, if_stage = data[:4]
        self.we_if, = struct.unpack_from("<f", data, 4)
        self.stage = stage
        self.if_stage = if_stage
        self.ready = bool(ready)
        if not active:
            raise RuntimeError("Sensorless stopped while running")
        return stage

    def handover_diag(self):
        values = self.snapshot()
        if values is None:
            print("  Handover diag: monitor data unavailable")
            return

        we_obs = values["We_obs"]
        print(
            "  Handover diag: "
            f"We_IF={self.we_if:.3f} rad/s, "
            f"We_obs={we_obs:.3f} rad/s, "
            f"We_err={we_obs - self.we_if:+.3f} rad/s, "
            f"PLL_mean={values['PLL_Err']:+.5f}, "
            f"PLL_RMS={values['PLL_RMS']:.5f}, "
            f"Flux_Err={values['Flux_Err']:+.8e} Wb^2"
        )

    def wait_run(self):
        deadline = time.monotonic() + self.args.ready_timeout
        next_status = time.monotonic()
        next_diag = None
        last_stage = None

        while time.monotonic() < deadline:
            if self.stop_request.requested:
                return False

            now = time.monotonic()
            if now >= next_status:
                stage = self.read_stage()
                if stage != last_stage:
                    name = sensorless_run.STAGE_NAME.get(stage, str(stage))
                    print(f"  Sensorless stage={name}")
                    last_stage = stage

                if (stage == 2) and not self.handover_started:
                    # Discard ALIGN/IF monitor history so all following averages
                    # describe the actual I/F -> observer handover window.
                    self.blocks.clear()
                    self.fast_saturation = [0] * len(sensorless_run.FAST_VARS)
                    self.handover_started = True
                    next_diag = now + 0.5

                if stage == sensorless_run.SENSORLESS_RUN:
                    self.handover_diag()
                    return True
                next_status = now + 0.05
            else:
                self.process(self.parser.feed(self.ser.read(4096)))

            if self.handover_started:
                self.guard(require_run=False)
                if next_diag is not None and now >= next_diag:
                    self.handover_diag()
                    next_diag = now + 1.0
            else:
                # Before IF_TO_OBS, only keep basic safety/transport checks from
                # interfering with the diagnostic window.
                if self.current_trip:
                    raise RuntimeError(
                        f"current magnitude exceeded {self.args.current_limit:.3f} A"
                    )
                if self.fast_lost > self.args.max_lost:
                    raise RuntimeError(f"FAST lost {self.fast_lost} frames")
                if self.normal_lost > self.args.max_lost:
                    raise RuntimeError(f"NORMAL lost {self.normal_lost} frames")

        self.handover_diag()
        raise TimeoutError("Sensorless did not enter SENSORLESS_RUN")


def main():
    args, wm_target = sensorless_run.parse_args()
    stop = sensorless_run.StopRequest()

    print(
        f"Handover diagnostic target: {wm_target:.3f} mechanical rad/s "
        f"({wm_target * args.pole_pairs:.1f} electrical rad/s)"
    )

    test = None
    error = None
    graceful = False
    try:
        with base.serial.Serial(args.port, args.baud, timeout=0.003,
                                write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            test = HandoverDiag(ser, args, stop)
            try:
                test.prepare()
                test.configure_plot()
                test.check_vbus()
                test.run_motor(wm_target)
                graceful = True
            except (TimeoutError, RuntimeError) as exc:
                error = exc
            finally:
                test.stop_all(graceful)

    except (base.serial.SerialException, OSError, TimeoutError,
            RuntimeError, KeyboardInterrupt) as exc:
        error = exc

    if error is not None:
        print(f"ERROR: {error}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
