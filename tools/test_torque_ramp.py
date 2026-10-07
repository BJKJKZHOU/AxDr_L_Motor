#!/usr/bin/env python3
"""Exercise torque slew and Stop through the firmware's real motor-control path."""

import ctypes as ct
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
TS = 0.0005
DISABLED, ENABLED, RUN = range(3)
TORQUE, SPEED, POSITION = range(3)


class Position(ct.Structure):
    _fields_ = [("Turn", ct.c_int32), ("Theta", ct.c_float)]


class Command(ct.Structure):
    _fields_ = [("Te_Target", ct.c_float), ("Wm_Target", ct.c_float),
                ("Position_Target", Position)]


class MotionConfig(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in ("Wm_Acc", "Wm_Dec", "Te_Rate")]


class TorqueRampTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="torque-ramp-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        boundary = path / "boundary.c"
        boundary.write_text('''
#include <stdlib.h>
#include "Motor_Control.h"
#include "Motor_Config.h"
#include "Motor_ADC.h"
#include "Motor_PWM.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Mechanical_ESO.h"
#include "Motion_Loop.h"
#include "Motion_Type.h"
#include "Open_Loop.h"
#include "Sensorless.h"
#include "Servo_Phase.h"

Motor_Para_T Motor_Para;
volatile ADC_T ADC;
volatile Encoder_T Encoder;
Mechanical_ESO_T Mechanical_ESO;
int Pwm_On;

void PWM_Enable(void) { Pwm_On = 1; }
void PWM_Disable(void) { Pwm_On = 0; }
void Current_Loop_State_Reset(void) {}
float Mechanical_ESO_Wm_Get(void) { return Mechanical_ESO.State.Wm; }

/* These modes must never execute in a sensored control regression. */
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool Sensorless_Begin(void) { abort(); }
void Sensorless_Stop_Request(void) { abort(); }
void Sensorless_Stop(void) { abort(); }
bool Sensorless_Active(void) { abort(); }
bool Sensorless_Speed_Control_Active(void) { abort(); }
float Sensorless_Wm_Get(void) { abort(); }
Sensorless_State_e Sensorless_State_Get(void) { abort(); }
void Sensorless_Control(float We, float Min, float Max) { abort(); }
void Sensorless_Run(float Ia, float Ib, float We, float *Theta,
                    float *Id, float *Iq) { abort(); }
bool Servo_Phase_Start(void) { abort(); }
void Servo_Phase_Abort(void) { abort(); }
bool Servo_Phase_Active(void) { abort(); }
bool Servo_Phase_Apply(void) { abort(); }
Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta, float *Id, float *Iq,
                                      float *Ua, float *Ub) { abort(); }
void Open_Loop_Reset(void) { abort(); }
void Open_Loop(float We, float *Theta, float *Id, float *Iq) { abort(); }
bool Identification_Start(Ident_Mode_e Mode, float Wm) { abort(); }
void Identification_Abort(void) { abort(); }
void Identification_Control(void) { abort(); }
bool Identification_Apply(void) { abort(); }
bool Identification_Active(void) { abort(); }
bool Identification_Result_Valid(void) { abort(); }
Ident_State_e Identification_State_Get(void) { abort(); }
Motor_Fast_Mode_e Identification_Fast_Run(float Ia, float Ib, float Ic,
                                         float *Theta, float *Id, float *Iq,
                                         float *Ua, float *Ub) { abort(); }

void Test_Setup(int Mode, float Kt, int Dir, float Rate)
{
    Motor_Disable();
    Motor_Mode = (uint8_t)Mode;
    Motor_Para = (Motor_Para_T){ .Pp = 2, .Rs = 0.1f, .Ld = 0.001f,
                               .Lq = 0.001f, .Flux = Kt / 3.0f };
    Motor_Run = (Motor_Run_T){ 0 };
    Motor_Cmd = (Motor_Cmd_T){ 0 };
    Motor_Config.Dir = (int8_t)Dir;
    User_Lim.I_Max = 5.0f;
    User_Lim.Wm_Max = 100.0f;
    ADC.Vbus_V = 24.0f;
    Encoder.Ready = 1;
    Encoder.Fault = 0;
    Motor_Cal.Valid = 1;
    Mechanical_ESO = (Mechanical_ESO_T){ .Para.Valid = 1 };
    Motion_Config = (Motion_Config_T){ .Wm_Acc = 100.0f, .Wm_Dec = 100.0f,
                                     .Te_Rate = Rate };
    Pos_Ctrl = (PID_T){ .Para.Kp = 5.0f };
    Speed_Ctrl = (PID_T){ .Para.Kp = 1.0f };
    Motor_Enable();
}

float Test_Iq(void)
{
    float Theta, Id, Iq, Ua, Ub;
    (void)Motor_Fast_Run(&Theta, &Id, &Iq, &Ua, &Ub);
    return Iq;
}

void Test_Current_Limit(float Limit) { User_Lim.I_Max = Limit; }
void Test_Vbus(float Vbus) { ADC.Vbus_V = Vbus; }
''')
        library = path / "torque.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-IMotor", "-IMotion", "-IAlgo", "-IUser", "-IObserver",
            "-IIdentification", "-ISensorless",
            "Motor/Motor_Control.c", "Motor/Motor_Config.c", "Motor/Motion_Loop.c",
            "Motion/Motion.c", "Motion/Ramp.c", "Motion/Trapezoid.c",
            "Algo/Math.c", "Algo/PID.c", str(boundary), "-lm", "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Test_Setup.argtypes = [ct.c_int, ct.c_float, ct.c_int, ct.c_float]
        cls.lib.Test_Iq.restype = ct.c_float
        cls.lib.Test_Current_Limit.argtypes = [ct.c_float]
        cls.lib.Test_Vbus.argtypes = [ct.c_float]
        cls.lib.Motor_State_Get.restype = ct.c_int
        cls.lib.Motor_Fast_Run.argtypes = [ct.POINTER(ct.c_float)] * 5
        cls.lib.Motor_Fast_Run.restype = ct.c_int
        cls.lib.Motor_Position_Ref_Get.restype = Position
        cls.lib.Motor_Wm_Ref_Get.restype = ct.c_float
        cls.cmd = Command.in_dll(cls.lib, "Motor_Cmd")
        cls.config = MotionConfig.in_dll(cls.lib, "Motion_Config")
        cls.pwm = ct.c_int.in_dll(cls.lib, "Pwm_On")
        if cls.config.Te_Rate != 0:
            raise AssertionError("torque slew must be disabled by default")

    def setUp(self):
        self.lib.Test_Setup(TORQUE, 0.5, 1, 2.0)
        self.assertEqual(self.lib.Motor_State_Get(), ENABLED)

    def step(self):
        self.lib.Motor_Control()
        return self.lib.Test_Iq()

    def test_slew_units_directions_reversal_and_endpoint(self):
        for kt in (0.25, 0.5, 1.0):
            for direction in (-1, 1):
                with self.subTest(kt=kt, direction=direction):
                    self.lib.Test_Setup(TORQUE, kt, direction, 2.0)
                    self.lib.Motor_Start()
                    for target in (0.0713, -0.0407, 0.0):
                        self.cmd.Te_Target = target
                        old = self.lib.Test_Iq()
                        for _ in range(200):
                            new = self.step()
                            self.assertLessEqual(abs(new - old) * kt, 2 * TS + 1e-7)
                            end = direction * target / kt
                            self.assertGreaterEqual(new, min(old, end) - 1e-7)
                            self.assertLessEqual(new, max(old, end) + 1e-7)
                            old = new
                        self.assertAlmostEqual(new, end, delta=1e-7)
                    self.assertEqual(new, 0)
                    self.assertEqual(self.lib.Motor_State_Get(), RUN)

    def test_stop_ramps_while_accepting_targets_and_restart_uses_latest(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.lib.Test_Setup(TORQUE, 0.5, direction, 2.0)
                self.cmd.Te_Target = 0.0407
                self.lib.Motor_Start()
                for _ in range(50):
                    self.step()
                self.lib.Motor_Stop()
                self.assertEqual(self.lib.Motor_State_Get(), RUN)
                old = abs(self.lib.Test_Iq())
                for n in range(50):
                    # A target accepted during Stop must not restart the ramp.
                    self.cmd.Te_Target = -0.1 - n * 0.001
                    self.lib.Motor_Stop()  # Repeated Stop must not reset progress.
                    new = abs(self.step())
                    self.assertLessEqual(new, old)
                    self.assertLessEqual(old - new, 2 * TS / 0.5 + 1e-7)
                    if new == 0:
                        break
                    self.assertEqual(self.lib.Motor_State_Get(), RUN)
                    old = new
                else:
                    self.fail("Stop did not finish")
                self.assertEqual(self.lib.Motor_State_Get(), ENABLED)
                self.assertEqual(self.pwm.value, 1)
                saved = self.cmd.Te_Target
                for _ in range(10):
                    self.assertEqual(self.step(), 0)
                self.assertEqual(self.cmd.Te_Target, saved)
                self.lib.Motor_Start()
                self.assertAlmostEqual(self.step(), -direction * 2 * TS / 0.5)

    def test_disabled_ramp_and_runtime_rate_changes(self):
        self.config.Te_Rate = 0
        self.cmd.Te_Target = 0.5
        self.lib.Motor_Start()
        self.assertEqual(self.step(), 1)
        self.config.Te_Rate = 2
        self.cmd.Te_Target = -0.5
        self.assertAlmostEqual(self.step(), 0.998, delta=1e-7)
        self.config.Te_Rate = 4
        self.assertAlmostEqual(self.step(), 0.994, delta=1e-7)
        self.config.Te_Rate = 0
        self.assertEqual(self.step(), -1)
        self.lib.Motor_Stop()
        self.assertEqual(self.step(), 0)
        self.assertEqual(self.lib.Motor_State_Get(), ENABLED)
        self.assertEqual(self.cmd.Te_Target, -0.5)

    def test_disable_bypasses_ramp_and_enable_starts_at_zero(self):
        self.cmd.Te_Target = 0.1
        for _ in range(10):
            self.assertEqual(self.step(), 0)  # ENABLED accepts but does not execute.
        self.lib.Motor_Start()
        for _ in range(20):
            self.step()
        self.lib.Motor_Stop()
        self.lib.Motor_Disable()
        self.assertEqual(self.lib.Motor_State_Get(), DISABLED)
        self.assertEqual(self.pwm.value, 0)
        self.assertEqual(self.lib.Test_Iq(), 0)
        self.assertAlmostEqual(self.cmd.Te_Target, 0.1)
        self.cmd.Te_Target = -0.2  # DISABLED also accepts targets.
        self.lib.Motor_Enable()
        self.assertEqual(self.step(), 0)
        self.lib.Motor_Start()
        self.assertAlmostEqual(self.step(), -0.002)

    def test_limits_override_slew_without_hidden_ramp_state(self):
        self.config.Te_Rate = 1000
        self.cmd.Te_Target = 100
        self.lib.Motor_Start()
        for _ in range(20):
            self.assertLessEqual(self.step(), 5)
        self.assertEqual(self.lib.Test_Iq(), 5)
        self.config.Te_Rate = 2
        self.lib.Test_Current_Limit(1)
        self.assertEqual(self.step(), 1)  # Limit reduction takes precedence.
        self.lib.Test_Current_Limit(5)
        self.assertAlmostEqual(self.step(), 1.002, delta=1e-7)
        self.lib.Test_Vbus(0)
        self.assertEqual(self.step(), 0)  # Existing voltage envelope also wins.
        self.lib.Test_Vbus(24)
        self.assertAlmostEqual(self.step(), 0.002)

    def test_speed_and_position_ignore_torque_rate(self):
        for mode in (SPEED, POSITION):
            traces = []
            for rate in (0, 0.0001, 1000):
                self.lib.Test_Setup(mode, 0.5, 1, rate)
                self.cmd.Wm_Target = 2
                self.cmd.Position_Target = Position(0, 0.02)
                self.lib.Motor_Start()
                trace = []
                for n in range(150):
                    if n == 80:
                        self.lib.Motor_Stop()
                    iq = self.step()
                    pos = self.lib.Motor_Position_Ref_Get()
                    trace.append((iq, self.lib.Motor_Wm_Ref_Get(), pos.Turn,
                                  pos.Theta, self.lib.Motor_State_Get()))
                traces.append(trace)
            self.assertEqual(traces[0], traces[1])
            self.assertEqual(traces[0], traces[2])
            self.assertTrue(any(row[0] != 0 for row in traces[0]))
            self.assertEqual(traces[0][-1][-1], ENABLED)

    def test_fast_control_holds_servo_but_gates_nonservo_modes(self):
        mode = ct.c_uint8.in_dll(self.lib, "Motor_Mode")
        for state in (ENABLED, DISABLED):
            if state == DISABLED:
                self.lib.Motor_Disable()
            for value in range(7):
                with self.subTest(state=state, mode=value):
                    # Exercise dispatch for each mode without starting its algorithm.
                    # Restore TORQUE before lifecycle calls to keep hardware stubs strict.
                    mode.value = value
                    try:
                        outputs = [ct.c_float(123) for _ in range(5)]
                        fast = self.lib.Motor_Fast_Run(*(ct.byref(x) for x in outputs))
                        expected = 1 if state == ENABLED and value in (TORQUE, SPEED, POSITION) else 0
                        self.assertEqual(fast, expected)
                        self.assertEqual([x.value for x in outputs], [0] * 5)
                    finally:
                        mode.value = TORQUE


if __name__ == "__main__":
    unittest.main()
