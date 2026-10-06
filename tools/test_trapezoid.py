#!/usr/bin/env python3
"""Run the firmware trajectory on the host; check motion constraints and endpoints."""

import ctypes as ct
import math
from pathlib import Path
import random
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TAU = ct.c_float(2 * math.pi).value


class Ref(ct.Structure):
    _fields_ = [("Turn", ct.c_int32)] + [
        (name, ct.c_float) for name in ("Theta", "Wm", "Am")]


def position(ref):
    return ref.Turn * TAU + ref.Theta


def target(x):
    turn = math.floor(x / TAU)
    return turn, ct.c_float(x - turn * TAU).value


class TrapezoidTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="trapezoid-")
        cls.addClassCleanup(cls.temp.cleanup)
        library = Path(cls.temp.name) / "trapezoid.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-ffp-contract=off", "-IMotion", "-IAlgo", "-IUser",
            "Motion/Trapezoid.c", "Motion/Ramp.c", "-lm", "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Trapezoid_Run.argtypes = [ct.POINTER(Ref), ct.c_int32] + [ct.c_float] * 5
        cls.lib.Trapezoid_Run.restype = None
        cls.lib.Trapezoid_Stop.argtypes = [ct.POINTER(Ref), ct.c_float, ct.c_float]
        cls.lib.Trapezoid_Stop.restype = ct.c_bool

    def run_move(self, ref, goal, vmax, acc, dec, ts, monotonic=False):
        goal = target(goal)
        end = goal[0] * TAU + goal[1]
        direction = 1 if end >= position(ref) else -1
        initial_speed = abs(ref.Wm)
        peak = 0
        # Bound includes stopping from a nonzero initial velocity and returning.
        timeout = 5 + 4 * (abs(end - position(ref)) + initial_speed**2 / dec) / vmax
        timeout += 4 * (vmax + initial_speed) / min(acc, dec)
        for i in range(math.ceil(timeout / ts)):
            old_x, old_v = position(ref), ref.Wm
            self.lib.Trapezoid_Run(ct.byref(ref), *goal, vmax, acc, dec, ts)
            self.assertTrue(all(math.isfinite(v) for v in (ref.Theta, ref.Wm, ref.Am)))
            self.assertTrue(0 <= ref.Theta < TAU)
            self.assertLessEqual(abs(ref.Wm), max(vmax, initial_speed) + 3e-5)
            self.assertLessEqual(abs(ref.Wm - old_v), max(acc, dec) * ts + 3e-5)
            if ref.Wm * old_v >= 0:
                limit = acc if abs(ref.Wm) > abs(old_v) else dec
                self.assertLessEqual(abs(ref.Wm - old_v), limit * ts + 3e-5)
            self.assertLessEqual(abs(position(ref) - old_x),
                                 max(abs(old_v), abs(ref.Wm)) * ts + max(acc, dec) * ts**2 + 3e-5)
            if monotonic:
                self.assertGreaterEqual(direction * (position(ref) - old_x), -2e-6)
                self.assertGreaterEqual(direction * ref.Wm, -2e-4)
                self.assertLessEqual(direction * (position(ref) - end), 2e-6)
            peak = max(peak, abs(ref.Wm))
            if (ref.Turn, ref.Theta, ref.Wm) == (*goal, 0):
                return (i + 1) * ts, peak
        self.fail(f"did not settle: {position(ref)=}, {end=}, {ref.Wm=}")

    def test_five_turns(self):
        for direction in (-1, 1):
            for turn in (-1000000, 0, 1000000):
                with self.subTest(direction=direction, turn=turn):
                    ref = Ref(turn, 1, 0, 0)
                    duration, peak = self.run_move(
                        ref, position(ref) + direction * 5 * TAU,
                        1000 * math.pi / 30, 500, 500, .001, True)
                    # Analytic duration: distance/vmax + vmax/acc = 0.50944 s.
                    self.assertAlmostEqual(duration, .510, delta=.002)
                    self.assertAlmostEqual(peak, 1000 * math.pi / 30, delta=2e-5)

    def test_short_and_asymmetric_moves(self):
        rng = random.Random(6835)
        for _ in range(100):
            distance = rng.choice((-1, 1)) * 10**rng.uniform(-4, 2)
            vmax = rng.uniform(5, 150)
            acc, dec = rng.uniform(20, 800), rng.uniform(20, 800)
            ts = rng.choice((.0005, .001, .004))
            with self.subTest(distance=distance, acc=acc, dec=dec, ts=ts):
                ref = Ref(0, 3, 0, 0)
                self.run_move(ref, 3 + distance, vmax, acc, dec, ts, True)

    def test_nonzero_speed_and_changed_target(self):
        # Reverse motion, unreachable stopping point, and a lowered speed limit.
        for x, speed, goal, vmax in ((1, -30, 10, 50), (1, 30, 1.1, 50),
                                      (1, 80, 40, 20), (1, -80, -40, 20)):
            with self.subTest(speed=speed, goal=goal):
                ref = Ref(*target(x), speed, 0)
                self.run_move(ref, goal, vmax, 100, 200, .001)
        ref = Ref(0, 1, 0, 0)
        for _ in range(100):
            self.lib.Trapezoid_Run(ct.byref(ref), *target(30), 100, 500, 300, .001)
        self.run_move(ref, -5, 100, 500, 300, .001)

    def test_phase_crossing_within_sample(self):
        # Accelerate for .1 s then decelerate for .1 s; cross the switch in one step.
        ref = Ref(0, 0, 0, 0)
        self.lib.Trapezoid_Run(ct.byref(ref), 0, .01, 10, 1, 1, .15)
        self.assertAlmostEqual(position(ref), .00875, delta=1e-7)
        self.assertAlmostEqual(ref.Wm, .05, delta=1e-7)
        self.lib.Trapezoid_Run(ct.byref(ref), 0, .01, 10, 1, 1, .15)
        self.assertEqual(ref.Theta, ct.c_float(.01).value)
        self.assertEqual(ref.Wm, 0)

        # Cross both acceleration and cruise boundaries while braking still
        # has more than one turn left; check the local displacement integral.
        peak = ct.c_float(100.1).value
        ta, tv, ts = (peak - 100) / 500, .0003, .001
        step = ts - ta - tv
        distance = .5 * (100 + peak) * ta + peak * tv + peak**2 / 200
        delta = .5 * (100 + peak) * ta + peak * tv + peak * step - 50 * step**2
        for direction in (-1, 1):
            ref = Ref(0, 1, direction * 100, 0)
            self.lib.Trapezoid_Run(ct.byref(ref), *target(1 + direction * distance),
                                   peak, 500, 100, ts)
            self.assertAlmostEqual(position(ref), 1 + direction * delta, delta=2e-6)
            self.assertAlmostEqual(ref.Wm, direction * (peak - 100 * step), delta=3e-5)

    def test_high_speed_braking_increment(self):
        # A long remaining stop must not become a large angle to normalize.
        ts = ct.c_float(.001).value
        for rpm in (3000, 12000):
            speed = ct.c_float(rpm * math.pi / 30).value
            for dec in (10, 100):
                for direction in (-1, 1):
                    for turn in (-1000000, 0, 1000000):
                        with self.subTest(rpm=rpm, dec=dec, direction=direction, turn=turn):
                            ref = Ref(turn, 1, direction * speed, 0)
                            stop = direction * speed**2 / (2 * dec)
                            goal_turn, goal_theta = target(1 + stop)
                            self.lib.Trapezoid_Run(ct.byref(ref), turn + goal_turn,
                                                   goal_theta, speed, dec, dec, ts)
                            delta = (ref.Turn - turn) * TAU + ref.Theta - 1
                            expected = direction * (speed * ts - .5 * dec * ts**2)
                            self.assertAlmostEqual(delta, expected, delta=2e-6)
                            self.assertAlmostEqual(ref.Wm, direction * (speed - dec * ts),
                                                   delta=2e-4)

    def test_long_braking_to_endpoint(self):
        speed = ct.c_float(12000 * math.pi / 30).value
        ts = ct.c_float(.001).value
        for dec in (10, 100):
            for direction in (-1, 1):
                with self.subTest(dec=dec, direction=direction):
                    ref = Ref(0, 1, direction * speed, 0)
                    goal = target(1 + direction * speed**2 / (2 * dec))
                    end = goal[0] * TAU + goal[1]
                    for _ in range(math.ceil(speed / dec / ts) + 100):
                        old_turn, old_theta, old_v = ref.Turn, ref.Theta, ref.Wm
                        self.lib.Trapezoid_Run(ct.byref(ref), *goal, speed, dec, dec, ts)
                        delta = (ref.Turn - old_turn) * TAU + ref.Theta - old_theta
                        # Allow a few float ULPs at high speed, not a fixed
                        # percentage of acceleration or a position deadband.
                        v_tol = max(3e-5, 4 * 2**-23 * abs(old_v))
                        self.assertGreaterEqual(direction * ref.Wm, 0)
                        self.assertLessEqual(abs(ref.Wm), abs(old_v) + v_tol)
                        self.assertLessEqual(abs(ref.Wm - old_v), dec * ts + v_tol)
                        self.assertGreaterEqual(direction * delta, -2e-6)
                        self.assertLessEqual(abs(delta), abs(old_v) * ts + 3e-6)
                        self.assertLessEqual(direction * (position(ref) - end), 2e-6)
                        self.assertTrue(0 <= ref.Theta < TAU)
                        if (ref.Turn, ref.Theta, ref.Wm) == (*goal, 0):
                            break
                    else:
                        self.fail("long braking trajectory did not reach its endpoint")

    def test_stop_partial_sample_and_zero_speed_limit(self):
        for speed in (-.2, .2, -17.3, 17.3):
            for zero_limit in (False, True):
                ref = Ref(0, 2, speed, 0)
                expected = 2 + math.copysign(speed**2 / 1000, speed)
                for _ in range(100):
                    if zero_limit:
                        self.lib.Trapezoid_Run(ct.byref(ref), 5, 2, 0, 100, 500, .001)
                    else:
                        self.lib.Trapezoid_Stop(ct.byref(ref), 500, .001)
                    if ref.Wm == 0:
                        break
                self.assertEqual(ref.Wm, 0)
                self.assertAlmostEqual(position(ref), expected, delta=3e-6)


if __name__ == "__main__":
    unittest.main()
