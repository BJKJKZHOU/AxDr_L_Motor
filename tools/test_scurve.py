#!/usr/bin/env python3
"""Exercise actual S-curve C code: analytical profiles and interrupted motion."""
import ctypes as ct
import math
from pathlib import Path
import random
import subprocess
import tempfile
import unittest

from test_trapezoid import Ref, TAU, position, target

ROOT = Path(__file__).resolve().parents[1]


class Segment(ct.Structure):
    _fields_ = [('Wm', ct.c_float), ('B', ct.c_float * 6), ('T', ct.c_float)]


class Curve(ct.Structure):
    _fields_ = [('Seg', Segment * 4), ('Time', ct.c_float), ('Time_Rem', ct.c_float),
                ('Pos_Rem', ct.c_float), ('Turn', ct.c_int32),
                ('Theta', ct.c_float), ('Wm', ct.c_float), ('Acc', ct.c_float),
                ('Dec', ct.c_float), ('Profile', ct.c_uint8), ('Count', ct.c_uint8),
                ('Position', ct.c_bool), ('To_Goal', ct.c_bool)]


class SCurveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='scurve-')
        cls.addClassCleanup(cls.temp.cleanup)
        library = Path(cls.temp.name) / 'scurve.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-ffp-contract=off', '-IMotion', '-IAlgo', '-IUser',
                        'Motion/SCurve.c', '-lm', '-o', str(library)], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.SCurve_Position.argtypes = [ct.POINTER(Curve), ct.POINTER(Ref), ct.c_int32,
                                           ct.c_float, ct.c_float, ct.c_float, ct.c_float,
                                           ct.c_uint8, ct.c_float]
        cls.lib.SCurve_Speed.argtypes = [ct.POINTER(Curve), ct.POINTER(Ref), ct.c_float,
                                       ct.c_float, ct.c_float, ct.c_uint8, ct.c_float]

    def tick(self, s, r, goal, vmax=10, acc=20, dec=30, profile=2, ts=.001):
        self.lib.SCurve_Position(ct.byref(s), ct.byref(r), *target(goal),
                                 vmax, acc, dec, profile, ts)

    def speed(self, s, r, v, acc=20, dec=30, profile=2, ts=.001):
        self.lib.SCurve_Speed(ct.byref(s), ct.byref(r), v, acc, dec, profile, ts)

    def finish(self, s, r, goal, vmax=10, acc=20, dec=30, profile=2, ts=.001):
        peak_v = peak_a = 0
        timeout = 10 + 5 * (abs(goal-position(r)) + r.Wm**2 / min(acc, dec)) / vmax
        timeout += 6 * (vmax + abs(r.Wm)) / min(acc, dec)
        max_acc = (1.875 if profile == 1 else 1) * max(acc, dec)
        v_limit = max(abs(r.Wm), vmax, abs(s.Wm))
        for n in range(math.ceil(timeout/ts)):
            old_x, old_v = position(r), r.Wm
            self.tick(s, r, goal, vmax, acc, dec, profile, ts)
            self.assertTrue(all(math.isfinite(x) for x in (r.Theta, r.Wm, r.Am)))
            self.assertLessEqual(s.Count, 4)
            self.assertTrue(0 <= r.Theta < TAU)
            self.assertLessEqual(abs(r.Wm), v_limit + .001)
            self.assertLessEqual(abs(r.Am), max_acc * 1.0001 + .001)
            self.assertLessEqual(abs(r.Wm-old_v), max_acc*ts*1.001 + 5e-5)
            self.assertLessEqual(abs(position(r)-old_x),
                                 max(abs(old_v), abs(r.Wm))*ts + max_acc*ts**2 + 2e-5)
            peak_v = max(peak_v, abs(r.Wm)); peak_a = max(peak_a, abs(r.Am))
            if s.Count == 0 and (r.Turn, r.Theta, r.Wm) == (*target(goal), 0):
                self.assertEqual(r.Am, 0)
                return (n+1)*ts, peak_v, peak_a
        self.fail(f'failed to arrive: {goal=} {position(r)=} {r.Wm=} {s.Count=}')

    def test_speed_polynomial_and_acceleration_meaning(self):
        for profile, k in ((1, 1), (2, 1.875)):
            for direction in (-1, 1):
                s, r = Curve(), Ref(0, 2, 0, 0)
                duration = k * 10 / 20
                for n in range(1, 1001):
                    self.speed(s, r, direction*10, profile=profile, ts=duration/1000)
                    u = n/1000
                    f = 10*u**3-15*u**4+6*u**5
                    self.assertAlmostEqual(r.Wm, direction*10*f, delta=.0003)
                    self.assertAlmostEqual(r.Am, direction*10/duration*30*u*u*(1-u)**2,
                                           delta=.002)
                self.speed(s, r, direction*10, profile=profile)
                self.assertEqual(r.Wm, direction*10)
                self.assertEqual(r.Am, 0)
                self.assertEqual(s.Count, 0)
                self.assertAlmostEqual(position(r), 2+direction*10*(duration/2+.001), delta=.0003)

    def test_five_turns_both_scalings_and_coordinates(self):
        vmax = 1000*math.pi/30
        for profile, k in ((1, 1), (2, 1.875)):
            vp = min(vmax, math.sqrt(5*TAU*500/k))
            expected = 5*TAU/vp + k*vp/500
            for direction in (-1, 1):
                for turn in (-1000000, 0, 1000000):
                    with self.subTest(profile=profile, direction=direction, turn=turn):
                        s, r = Curve(), Ref(turn, 1, 0, 0)
                        duration, peak, acc = self.finish(s, r, position(r)+direction*5*TAU,
                                                         vmax, 500, 500, profile)
                        self.assertAlmostEqual(duration, expected, delta=.002)
                        self.assertAlmostEqual(peak, vp, delta=.003)
                        self.assertAlmostEqual(acc, 500*(1.875 if profile == 1 else 1), delta=.1)

    def test_short_asymmetric_moves(self):
        rng = random.Random(6835)
        for n in range(100):
            profile = rng.choice((1, 2)); acc = rng.uniform(5, 800); dec = rng.uniform(5, 800)
            delta = rng.choice((-1, 1))*10**rng.uniform(-5, 2)
            vmax = rng.uniform(1, 150)
            with self.subTest(n=n, delta=delta, profile=profile):
                self.finish(Curve(), Ref(0, 3, 0, 0), 3+delta, vmax, acc, dec,
                            profile, rng.choice((.0005, .001, .004)))

    def test_cross_all_segments_in_one_sample(self):
        for profile in (1, 2):
            s, r = Curve(), Ref(0, 1, 0, 0)
            self.tick(s, r, 1.01, 1, 20, 30, profile, 1)
            self.assertEqual((r.Turn, r.Theta), target(1.01))
            self.assertEqual((r.Wm, r.Am, s.Count), (0, 0, 0))

    def test_mid_motion_retargets(self):
        for profile in (1, 2):
            for ticks in (1, 30, 90, 200, 650, 1000):
                for goal in (-3, .01, 7, 25):
                    with self.subTest(profile=profile, ticks=ticks, goal=goal):
                        s, r = Curve(), Ref(0, 0, 0, 0)
                        for _ in range(ticks):
                            self.tick(s, r, 20, profile=profile)
                        self.finish(s, r, goal, profile=profile)

    def test_release_preserves_acceleration_and_jerk(self):
        for profile in (1, 2):
            s, r = Curve(), Ref(0, 0, 0, 0)
            for _ in range(60):
                self.tick(s, r, 20, profile=profile)
            old_a = r.Am
            seg = s.Seg[0]; u = s.Time/seg.T
            # Independent power-free Bernstein derivative evaluation at the splice.
            a = [5*(seg.B[i+1]-seg.B[i])/seg.T for i in range(5)]
            j = [4*(a[i+1]-a[i])/seg.T for i in range(4)]
            old_j = sum(math.comb(3,i)*(1-u)**(3-i)*u**i*j[i] for i in range(4))
            self.tick(s, r, -2, profile=profile, ts=1e-7)
            seg = s.Seg[0]
            new_a = 5*(seg.B[1]-seg.B[0])/seg.T
            new_j = 20*(seg.B[2]-2*seg.B[1]+seg.B[0])/seg.T**2
            self.assertAlmostEqual(old_a, new_a, delta=1e-4)
            self.assertAlmostEqual(old_j, new_j, delta=.02)
            self.assertAlmostEqual(r.Am, old_a, delta=.001)

    def test_zero_limit_stop_and_resume(self):
        for profile in (1, 2):
            for ticks in (1, 50, 200, 1000):
                s, r = Curve(), Ref(0, 1, 0, 0)
                for _ in range(ticks):
                    self.tick(s, r, 20, profile=profile)
                for n in range(10000):
                    self.tick(s, r, 20, 0, profile=profile)
                    if s.Count == 0 and r.Wm == 0:
                        break
                else:
                    self.fail('zero speed did not stop')
                end = position(r)
                for _ in range(20):
                    self.tick(s, r, -2, 0, profile=profile)
                    self.assertAlmostEqual(position(r), end, delta=1e-6)
                self.finish(s, r, -2, profile=profile)

    def test_explicit_stop_during_each_phase(self):
        for profile in (1, 2):
            for ticks in (1, 50, 400, 1700, 2400):
                s, r = Curve(), Ref(0, 1, 0, 0)
                for _ in range(ticks):
                    self.tick(s, r, 20, profile=profile)
                for n in range(10000):
                    old = r.Wm
                    self.speed(s, r, 0, profile=profile)
                    self.assertLessEqual(abs(r.Wm-old), 1.875*30*.001+.0001)
                    if s.Count == 0 and r.Wm == 0:
                        break
                else:
                    self.fail('Stop did not finish')
                self.assertEqual(r.Am, 0)
                self.finish(s, r, 3, profile=profile)

    def test_speed_reversal_and_parameter_changes(self):
        for profile in (1, 2):
            s, r = Curve(), Ref(0, 1, 0, 0)
            for v, acc, dec in ((10,20,30), (-5,10,15), (3,100,50), (0,20,30)):
                for n in range(10000):
                    self.speed(s, r, v, acc, dec, profile)
                    if s.Count == 0 and r.Wm == v:
                        break
                else:
                    self.fail('speed command did not finish')
                self.assertEqual(r.Am, 0)

    def test_limit_reduction_during_acceleration(self):
        s, r = Curve(), Ref(0, 0, 0, 0)
        for _ in range(200):
            self.tick(s, r, 30, 10, 20, 30)
        old_a = r.Am
        self.tick(s, r, 30, 2, 5, 7, ts=1e-7)
        self.assertAlmostEqual(r.Am, old_a, delta=.001)
        for _ in range(2000):
            self.tick(s, r, 30, 2, 5, 7)
        self.assertLessEqual(abs(r.Wm), 2.0001)
        self.assertLessEqual(abs(r.Am), 7.0001)
        self.finish(s, r, 30, 2, 5, 7)

    def test_repeated_replans_within_release(self):
        rng = random.Random(474)
        for profile in (1, 2):
            s, r = Curve(), Ref(0, 1, 0, 0)
            for _ in range(300):
                goal = rng.uniform(-5, 5)
                vmax = rng.uniform(.1, 20)
                acc, dec = rng.uniform(10, 300), rng.uniform(10, 300)
                for _ in range(rng.randint(1, 20)):
                    old_x, old_v = position(r), r.Wm
                    self.tick(s, r, goal, vmax, acc, dec, profile)
                    self.assertLessEqual(s.Count, 4)
                    self.assertLessEqual(abs(r.Am), 1.875*300 + .01)
                    self.assertLessEqual(abs(r.Wm), 20.001)
                    self.assertLessEqual(abs(r.Wm-old_v), 1.875*300*.001+.0001)
                    self.assertLessEqual(abs(position(r)-old_x),
                                         max(abs(old_v), abs(r.Wm))*.001+.0006)
            self.finish(s, r, 2, 20, 500, 500, profile)

    def test_long_cruise_and_braking(self):
        for profile in (1, 2):
            for direction in (-1, 1):
                s, r = Curve(), Ref(1000000, 1, 0, 0)
                self.finish(s, r, position(r)+direction*100*TAU,
                            100, 100, 100, profile)


if __name__ == '__main__':
    unittest.main()
