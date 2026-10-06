#!/usr/bin/env python3
"""Run the firmware current protection with simulated motor state and PWM."""

import ctypes as ct
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ProtectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="protection-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        # Only the motor/hardware boundary is stubbed; compile Protection.c itself.
        header = path / "boundary.h"
        header.write_text('''
#include <stdbool.h>
#include "Motor_Type.h"
#define MOTOR_CONTROL_H
Motor_State_e Motor_State_Get(void);
bool Motor_Encoder_Required(void);
void Motor_Disable(void);
''')
        source = path / "boundary.c"
        source.write_text('''
#include "boundary.h"
#include "Motor_ADC.h"
#include "Encoder.h"
volatile ADC_T ADC = { .Vbus_V = 24.0f };
volatile Encoder_T Encoder;
int State;
int Pwm_Off;
Motor_State_e Motor_State_Get(void) { return (Motor_State_e)State; }
bool Motor_Encoder_Required(void) { return false; }
void Motor_Disable(void) { State = DISABLED; Pwm_Off = 1; }
void PWM_Disable(void) { Pwm_Off = 1; }
''')
        library = path / "protection.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-IMotor", "-IUser", "-include", str(header),
            "Motor/Protection.c", str(source), "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Protection_Current_Fast.argtypes = [ct.c_float] * 3
        cls.lib.Protection_Current_Fast.restype = ct.c_bool
        cls.lib.Protection_Clear.restype = ct.c_bool
        cls.lib.Protection_Enable_Allowed.restype = ct.c_bool
        cls.state = ct.c_int.in_dll(cls.lib, "State")
        cls.pwm_off = ct.c_int.in_dll(cls.lib, "Pwm_Off")
        cls.events = (ct.c_uint32 * 4).in_dll(cls.lib, "Protection")

    def setUp(self):
        self.state.value = 0
        self.assertTrue(self.lib.Protection_Clear())
        self.pwm_off.value = 0
        self.state.value = 2

    def sample(self, peak):
        return self.lib.Protection_Current_Fast(peak, -peak / 2, -peak / 2)

    def test_fast_threshold_delay_and_latch(self):
        for _ in range(4):
            self.assertTrue(self.sample(26))
            self.assertEqual(self.pwm_off.value, 0)
        self.assertFalse(self.sample(26))
        self.assertEqual(self.pwm_off.value, 1)
        self.assertFalse(self.sample(0))
        self.assertFalse(self.lib.Protection_Enable_Allowed())
        self.assertFalse(self.lib.Protection_Clear())
        self.lib.Protection_Control()
        self.assertEqual(self.events[2], 1 << 3)
        self.assertEqual(self.events[3], 0)
        self.assertEqual(self.state.value, 0)
        self.assertTrue(self.lib.Protection_Clear())
        self.assertTrue(self.lib.Protection_Enable_Allowed())

    def test_overload_delay(self):
        for _ in range(4999):
            self.assertTrue(self.sample(16))
        self.assertFalse(self.sample(16))
        self.assertEqual(self.pwm_off.value, 1)

    def test_thresholds_remain_strict(self):
        for _ in range(6000):
            self.assertTrue(self.sample(15))
        for _ in range(5):
            self.assertTrue(self.sample(25))
        self.assertEqual(self.pwm_off.value, 0)

    def test_interrupted_overcurrent_resets_delay(self):
        for _ in range(2):
            for _ in range(4):
                self.assertTrue(self.sample(26))
            self.assertTrue(self.sample(0))
        for _ in range(4999):
            self.assertTrue(self.sample(16))
        self.assertTrue(self.sample(0))
        self.assertTrue(self.sample(16))

    def test_invalid_phase_feedback_stops_immediately(self):
        for phase in range(3):
            for bad in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(phase=phase, bad=bad):
                    self.setUp()
                    currents = [0.0] * 3
                    currents[phase] = bad
                    self.assertFalse(self.lib.Protection_Current_Fast(*currents))
                    self.assertEqual(self.pwm_off.value, 1)
                    self.lib.Protection_Control()
                    self.assertEqual(self.events[2], 1 << 3)
                    self.assertEqual(self.state.value, 0)

    def test_enabled_is_protected_disabled_resets_delay(self):
        self.state.value = 1
        for _ in range(4):
            self.assertTrue(self.sample(26))
        self.state.value = 0
        self.assertTrue(self.sample(26))
        self.state.value = 2
        for _ in range(4):
            self.assertTrue(self.sample(26))
        self.assertFalse(self.sample(26))


if __name__ == "__main__":
    unittest.main()
