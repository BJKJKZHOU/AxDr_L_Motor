#!/usr/bin/env python3
"""Host regression of the firmware ESO: wrap crossings and angle-origin invariance."""

import ctypes as ct
import math
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TS = 1.0 / 20000.0
TAU = ct.c_float(2.0 * math.pi).value
J, B, KT = 1.7e-5, 8.0e-6, 0.024


class Para(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in
                ("Kt_Over_J", "Inv_J", "B_Over_J", "L1", "L2", "L3")]
    _fields_ += [("Valid", ct.c_uint8)]


class State(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ("Theta", "Wm", "Td", "Error")]


class ESO(ct.Structure):
    _fields_ = [("Para", Para), ("State", State)]


class MechanicalESOTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="mechanical-eso-")
        cls.addClassCleanup(cls.temp.cleanup)
        library = Path(cls.temp.name) / "eso.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-ffp-contract=off",
            "-IObserver", "-IUser", "-IAlgo", "-IMotor",
            "Observer/Mechanical_ESO.c", "Algo/Math.c", "-lm",
            "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Mechanical_ESO_Para_Build.argtypes = [ct.POINTER(Para)] + [ct.c_float] * 4
        cls.lib.Mechanical_ESO_Para_Build.restype = ct.c_bool
        cls.lib.Mechanical_ESO_Run.argtypes = [ct.c_float, ct.c_bool, ct.c_float]
        cls.eso = ESO.in_dll(cls.lib, "Mechanical_ESO")

    def configure(self, *args):
        candidate = Para.from_buffer_copy(self.eso.Para)
        before = bytes(candidate)
        active = bytes(self.eso)
        success = self.lib.Mechanical_ESO_Para_Build(ct.byref(candidate), *args)
        self.assertEqual(bytes(self.eso), active)
        if success:
            self.eso.Para = candidate
        else:
            self.assertEqual(bytes(candidate), before)
        return success

    def setUp(self):
        self.assertTrue(self.configure(J, B, KT, 2 * math.pi * 100))

    def test_constant_speed_across_turns(self):
        # The position chain may count any number of turns; ESO receives only phase.
        for speed in (-150.0, -80.0, -1.0, 1.0, 80.0, 150.0):
            for origin in (0.0, 4096.0, 8192.0, 65536.0):
                with self.subTest(speed=speed, origin=origin):
                    load = math.copysign(0.002, speed)
                    iq = (B * speed + load) / KT
                    self.eso.State = State(origin % TAU, speed, 0, 0)
                    error_sum = 0.0
                    peak = 0.0
                    for n in range(20000):
                        theta = (origin + speed * n * TS) % TAU
                        self.lib.Mechanical_ESO_Run(theta, True, iq)
                        state = self.eso.State
                        self.assertTrue(0.0 <= state.Theta < TAU)
                        if n >= 2000:
                            error_sum += state.Wm - speed
                            peak = max(peak, abs(state.Wm - speed))
                            self.assertLess(abs(state.Error), 0.0001)
                    self.assertLess(abs(error_sum / 18000), 0.005)
                    self.assertLess(peak, 0.02)
                    self.assertAlmostEqual(self.eso.State.Td, load, delta=1e-5)

    def test_wrap_with_temporarily_invalid_position(self):
        for speed, theta in ((80.0, TAU - 0.02), (-80.0, 0.02)):
            with self.subTest(speed=speed):
                self.eso.State = State(theta, speed, 0, 0)
                for n in range(200):
                    self.lib.Mechanical_ESO_Run(
                        (theta + speed * n * TS) % TAU, n >= 100, B * speed / KT
                    )
                    state = self.eso.State
                    self.assertTrue(0.0 <= state.Theta < TAU)
                    self.assertLess(abs(state.Wm - speed), 0.02)
                    self.assertLess(abs(state.Error), 0.0001)

    def test_angle_only_start_and_coast(self):
        # No differential-speed seed: angle feedback acquires an already
        # moving rotor and continues tracking with zero measured torque.
        for speed in (-1000 * math.pi / 30, 1000 * math.pi / 30):
            with self.subTest(speed=speed):
                theta = 1.0
                self.eso.State = State(theta, 0, 0, 0)
                for n in range(20000):
                    self.lib.Mechanical_ESO_Run(theta, True, 0)
                    if n > 2000:
                        self.assertLess(abs(self.eso.State.Wm - speed), .03)
                    theta = (theta + speed * TS) % TAU
                    speed *= math.exp(-B / J * TS)

    def test_retuning_preserves_observer_state(self):
        self.eso.State = State(1, 30, .002, .0001)
        before = bytes(self.eso.State)
        for bandwidth in (1, 50, 100, 150, 200, 2000):
            with self.subTest(bandwidth=bandwidth):
                self.assertTrue(self.configure(
                    J, B, KT, 2 * math.pi * bandwidth))
                self.assertEqual(self.eso.Para.Valid, 1)
                self.assertEqual(bytes(self.eso.State), before)

    def test_invalid_inputs_preserve_active_observer(self):
        self.eso.State = State(1, 30, .002, .0001)
        before = bytes(self.eso)
        for index in range(4):
            invalid = [-1.0, float("nan"), float("inf")]
            if index != 1:  # Zero damping is a valid model.
                invalid.append(0.0)
            for value in invalid:
                with self.subTest(index=index, value=value):
                    args = [J, B, KT, 2 * math.pi * 100]
                    args[index] = value
                    self.assertFalse(self.configure(*args))
                    self.assertEqual(bytes(self.eso), before)

    def test_finite_negative_l1_is_allowed(self):
        # A finite but aggressive B/J to bandwidth ratio must not be
        # rejected solely because the designed observer gain L1 is negative.
        self.eso.State = State(1, 30, .002, .0001)
        before = bytes(self.eso.State)
        self.assertTrue(self.configure(J, B, KT, 2 * math.pi * .001))
        self.assertLess(self.eso.Para.L1, 0)
        self.assertEqual(bytes(self.eso.State), before)

    def test_unrepresentable_coefficients_preserve_active_observer(self):
        self.eso.State = State(1, 30, .002, .0001)
        before = bytes(self.eso)
        for args in ((J, B, KT, 1e20),
                     (1e-10, 0, 1e30, 628),
                     (1e-40, 0, KT, 628)):
            with self.subTest(args=args):
                self.assertFalse(self.configure(*args))
                self.assertEqual(bytes(self.eso), before)
        self.lib.Mechanical_ESO_Run(1.0, True, B * 30 / KT)
        self.assertNotEqual(self.eso.State.Theta, 1.0)
        self.assertTrue(math.isfinite(self.eso.State.Wm))

    def test_failed_initial_config_stays_invalid(self):
        self.eso.Para = Para()
        before = bytes(self.eso)
        self.assertFalse(self.configure(J, B, KT, 0))
        self.assertEqual(bytes(self.eso), before)
        self.assertEqual(self.eso.Para.Valid, 0)


if __name__ == "__main__":
    unittest.main()
