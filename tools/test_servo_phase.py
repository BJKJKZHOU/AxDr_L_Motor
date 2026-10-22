#!/usr/bin/env python3
"""验证实际寻相代码的有效标记、结果应用及独立保存。完成通知由 encoder_auto_cal 覆盖。"""

import ctypes as ct
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ServoPhaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="servo-phase-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        source = '''
#include <assert.h>
#include "Encoder.h"
#include "Motor_Control.h"
#include "Motor_Cal.h"
#include "Motor_Para.h"
#include "Servo_Phase.h"
#include "NVS_Storage.h"
#include "Parameter.h"
#include "Protocol.h"
volatile Encoder_T Encoder;
Motor_Cal_T Motor_Cal;
Motor_Para_T Motor_Para;
int Test_State, Test_Save_Status, Test_Saves;
Motor_State_e Motor_State_Get(void) { return (Motor_State_e)Test_State; }
void Encoder_DMA_Config(void) { Encoder.Ready = 0; }
float Encoder_Angle_Correct(float Theta) { return Theta; }
int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count)
{
    const uint16_t Expected[] = {
        PARAM_CAL_ENC_DIR, PARAM_CAL_THETA_OFF, PARAM_CAL_VALID,
    };
    assert(Count == 3 && Motor_Cal.Valid == 1);
    for (unsigned i = 0; i < Count; i++) { assert(Ids[i] == Expected[i]); }
    Test_Saves++;
    return Test_Save_Status;
}
void Test_Setup(void)
{
    Servo_Phase_Abort();
    Encoder = (Encoder_T){ .Ready = 1, .Valid = 1, .Theta_Native = 1.0f };
    Motor_Para.Pp = 7;
    Motor_Cal = (Motor_Cal_T){ .Enc_Dir = 1, .Theta_Off = 2.0f, .Valid = 1 };
    Test_State = DISABLED;
    Test_Save_Status = Test_Saves = 0;
}
int Test_Valid(void) { return Motor_Cal.Valid; }
void Test_Ready(int Ready) { Encoder.Ready = Ready; }
void Test_Fault(void) { Encoder.Fault = 1; }
void Test_Running(void) { Test_State = RUN; }
void Test_Samples(unsigned Count, float Theta)
{
    float Theta_e, Id, Iq, Ua, Ub;
    Encoder.Theta_Native = Theta;
    for (unsigned i = 0; i < Count; i++)
    {
        Servo_Phase_Fast_Run(&Theta_e, &Id, &Iq, &Ua, &Ub);
    }
}
'''
        (path / "boundary.c").write_text(source)
        includes = ["Motor", "User", "Algo", "Observer", "Identification",
                    "Parameter", "Comm", "Storage"]
        subprocess.run([
            "gcc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            *[f"-I{ROOT / name}" for name in includes], str(path / "boundary.c"),
            str(ROOT / "Motor/Servo_Phase.c"), str(ROOT / "Motor/Motor_Cal.c"),
            str(ROOT / "Algo/Math.c"), "-lm", "-o", str(path / "phase.so"),
        ], check=True)
        cls.lib = ct.CDLL(str(path / "phase.so"))
        for name in ("Servo_Phase_Start", "Servo_Phase_Active", "Servo_Phase_Apply", "Servo_Phase_Save"):
            getattr(cls.lib, name).restype = ct.c_bool
        cls.lib.Test_Samples.argtypes = [ct.c_uint, ct.c_float]

    def setUp(self):
        self.lib.Test_Setup()

    def value(self, name, value=None):
        item = ct.c_int.in_dll(self.lib, name)
        if value is not None:
            item.value = value
        return item.value

    def start(self):
        self.assertTrue(self.lib.Servo_Phase_Start())
        self.lib.Test_Running()
        self.assertEqual(self.lib.Test_Valid(), 0)

    def finish_samples(self):
        self.lib.Test_Samples(2100, 1.0)
        self.lib.Test_Samples(13000, 1.02)
        self.assertFalse(self.lib.Servo_Phase_Active())
        self.assertEqual(self.lib.Test_Valid(), 0)

    def test_rejected_start_preserves_valid(self):
        self.lib.Test_Ready(0)
        self.assertFalse(self.lib.Servo_Phase_Start())
        self.assertEqual(self.lib.Test_Valid(), 1)

    def test_apply_and_save_are_separate(self):
        self.start()
        self.finish_samples()
        self.assertEqual(self.value("Test_Saves"), 0)
        self.value("Test_State", 0)
        self.assertTrue(self.lib.Servo_Phase_Apply())
        self.assertEqual(self.lib.Test_Valid(), 1)
        self.assertEqual(self.value("Test_Saves"), 0)
        self.assertTrue(self.lib.Servo_Phase_Save())
        self.assertEqual(self.value("Test_Saves"), 1)
        self.start()
        self.lib.Servo_Phase_Abort()
        self.value("Test_State", 0)
        self.assertFalse(self.lib.Servo_Phase_Apply())
        self.assertFalse(self.lib.Servo_Phase_Save())
        self.assertEqual(self.lib.Test_Valid(), 0)
        self.assertEqual(self.value("Test_Saves"), 1)

    def test_abort_and_encoder_fault(self):
        for fault in (False, True):
            with self.subTest(fault=fault):
                self.setUp()
                self.start()
                if fault:
                    self.lib.Test_Fault()
                    self.lib.Test_Samples(1, 1.0)
                else:
                    self.lib.Servo_Phase_Abort()
                self.value("Test_State", 0)
                self.assertFalse(self.lib.Servo_Phase_Apply())
                self.assertEqual(self.lib.Test_Valid(), 0)
                self.assertEqual(self.value("Test_Saves"), 0)

    def test_apply_and_save_failure(self):
        self.start()
        self.finish_samples()
        self.value("Test_State", 1)
        self.assertFalse(self.lib.Servo_Phase_Apply())
        self.assertEqual(self.lib.Test_Valid(), 0)
        self.assertFalse(self.lib.Servo_Phase_Save())
        self.assertEqual(self.value("Test_Saves"), 0)

        self.setUp()
        self.start()
        self.finish_samples()
        self.value("Test_State", 0)
        self.assertTrue(self.lib.Servo_Phase_Apply())
        self.value("Test_Save_Status", -5)
        self.assertFalse(self.lib.Servo_Phase_Save())
        self.assertEqual(self.lib.Test_Valid(), 1)
        self.value("Test_Save_Status", 0)
        self.assertTrue(self.lib.Servo_Phase_Save())
        self.assertEqual(self.value("Test_Saves"), 2)


if __name__ == "__main__":
    unittest.main()
