#!/usr/bin/env python3
"""Exercise real parameter, tuning and storage code with an in-memory NVS boundary."""

import ctypes as ct
from pathlib import Path
import subprocess
import tempfile
import unittest

import parameter_ids_generated as p


ROOT = Path(__file__).resolve().parents[1]
CURRENT = (p.PARAM_CTRL_CURRENT_SOURCE, p.PARAM_CTRL_CURRENT_BW_HZ,
           (p.PARAM_CTRL_ID_KP, p.PARAM_CTRL_ID_KI,
            p.PARAM_CTRL_IQ_KP, p.PARAM_CTRL_IQ_KI))
SPEED = (p.PARAM_CTRL_SPEED_SOURCE, p.PARAM_CTRL_SPEED_BW_HZ,
         (p.PARAM_CTRL_SPEED_KP, p.PARAM_CTRL_SPEED_KI))
GROUPS = (CURRENT, SPEED)


class TuningStorageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="tuning-storage-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        (path / "main.h").write_text('''
#include <stdint.h>
extern uint32_t Test_Primask;
static inline uint32_t __get_PRIMASK(void) { return Test_Primask; }
static inline void __disable_irq(void) { Test_Primask = 1U; }
static inline void __set_PRIMASK(uint32_t Value) { Test_Primask = Value; }
''')
        nvs = path / "zephyr/kvss/nvs.h"
        nvs.parent.mkdir(parents=True)
        nvs.write_text('''
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
struct nvs_fs {
    uint32_t offset;
    uint16_t sector_size;
    uint16_t sector_count;
    void *flash_device;
};
int nvs_mount(struct nvs_fs *Fs);
int nvs_clear(struct nvs_fs *Fs);
ssize_t nvs_read(struct nvs_fs *Fs, uint16_t Id, void *Data, size_t Size);
ssize_t nvs_write(struct nvs_fs *Fs, uint16_t Id, const void *Data, size_t Size);
''')
        source = path / "boundary.c"
        source.write_text('''
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "Current_Loop.h"
#include "Encoder.h"
#include "Flash_Storage.h"
#include "Mechanical_ESO.h"
#include "Motion_Loop.h"
#include "Motion_Type.h"
#include "Motor_ADC.h"
#include "Motor_Cal.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "NVS_Storage.h"
#include "Parameter.h"
#include "Protection.h"
#include "Sensorless.h"
#include "Servo_Phase.h"
#include "control_params.h"
#include "motor_params.h"
#include <zephyr/kvss/nvs.h>

/* Unrelated control/hardware boundaries; parameter/tuning/storage are real. */
PID_T Id_Ctrl, Iq_Ctrl, Speed_Ctrl, Pos_Ctrl;
volatile ADC_T ADC;
volatile Encoder_T Encoder;
Encoder_Config_T Encoder_Config;
Motor_Config_T Motor_Config;
Motor_Cmd_T Motor_Cmd;
Motor_Cal_T Motor_Cal;
Motor_Run_T Motor_Run;
uint8_t Motor_Mode;
const Motor_Limit_T Motor_Lim = { .I_Max = 20.0f, .Wm_Max = 1000.0f };
Motor_Limit_T User_Lim;
Motion_Config_T Motion_Config;
volatile Protection_T Protection;
Servo_Phase_Config_T Servo_Phase_Config;
PLL_T Flux_PLL;
Mechanical_ESO_T Mechanical_ESO;
float Mechanical_ESO_Bw_Hz = 100.0f;
volatile float Ident_JB_Excite_Ratio, Ident_JB_Excite_Hz;
int Test_State, Test_Reads, Test_Reads_Masked, Test_Writes;
uint32_t Test_Primask;
uint16_t Test_Write_Log[256];
int Test_Write_Fail;

Motor_State_e Motor_State_Get(void) { return (Motor_State_e)Test_State; }
float Motor_I_Limit_Effective_Get(void) { return User_Lim.I_Max; }
float Motor_Wm_Limit_Effective_Get(void) { return User_Lim.Wm_Max; }
float Motor_Wm_Get(void) { return 0.0f; }
float Motor_Wm_Ref_Get(void) { return 0.0f; }
float Motor_Iq_Ref_Get(void) { return 0.0f; }
Motor_Position_T Motor_Position_Ref_Get(void) { return (Motor_Position_T){0}; }
void Motor_Position_Get(int32_t *Turn, float *Theta) { *Turn = 0; *Theta = 0; }
float Protection_Vbus_Min_Get(void) { return 0.0f; }
float Protection_Vbus_Max_Get(void) { return 60.0f; }
void Encoder_Config_Changed(void) { }
void Motor_Cal_Invalidate(void) { Motor_Cal.Valid = 0; }
bool Mechanical_ESO_Config(float J, float B, float Kt, float Wo)
{
    (void)J; (void)B; (void)Kt; (void)Wo;
    return true;
}
const Servo_Phase_Result_T *Servo_Phase_Last_Result_Get(void)
{
    static const Servo_Phase_Result_T Result;
    return &Result;
}
uint8_t Identification_Rs_Ls_Valid_Get(void) { return 0; }
uint8_t Identification_Flux_Valid_Get(void) { return 0; }
uint8_t Identification_JB_Valid_Get(void) { return 0; }
uint8_t Identification_Fail_Reason_Get(void) { return 0; }
float Identification_Rs_Get(void) { return 0; }
float Identification_Ls_Get(void) { return 0; }
float Identification_Flux_Get(void) { return 0; }
float Identification_J_Get(void) { return 0; }
float Identification_B_Get(void) { return 0; }

/* Records model the documented nvs_read/write return values, including errors
 * and oversized records. This does not emulate Flash wear or power loss. */
static struct { unsigned char Data[8]; int Size; } Record[65536];
bool Flash_Storage_Geometry_Valid(void) { return true; }
int nvs_mount(struct nvs_fs *Fs) { (void)Fs; return 0; }
int nvs_clear(struct nvs_fs *Fs)
{
    (void)Fs;
    memset(Record, 0, sizeof(Record));
    return 0;
}
ssize_t nvs_read(struct nvs_fs *Fs, uint16_t Id, void *Data, size_t Size)
{
    (void)Fs;
    Test_Reads++;
    Test_Reads_Masked += Test_Primask != 0;
    int Stored = Record[Id].Size;
    if (Stored == 0) { return -ENOENT; }
    if (Stored < 0) { return Stored; }
    memcpy(Data, Record[Id].Data, Size < (size_t)Stored ? Size : (size_t)Stored);
    return Stored;
}
ssize_t nvs_write(struct nvs_fs *Fs, uint16_t Id, const void *Data, size_t Size)
{
    (void)Fs;
    assert(Size <= sizeof(Record[Id].Data));
    assert(Test_Writes < 256);
    Test_Write_Log[Test_Writes++] = Id;
    if (Test_Write_Fail == Test_Writes) { return -EIO; }
    if (Record[Id].Size == (int)Size && memcmp(Record[Id].Data, Data, Size) == 0)
    {
        return 0;
    }
    memcpy(Record[Id].Data, Data, Size);
    Record[Id].Size = (int)Size;
    return (ssize_t)Size;
}
void Test_Record(uint16_t Id, float Gain, int Size)
{
    memcpy(Record[Id].Data, &Gain, sizeof(Gain));
    Record[Id].Size = Size;
}
int Test_Record_Size(uint16_t Id) { return Record[Id].Size; }
void Test_Ram_Defaults(void)
{
    Test_State = DISABLED;
    Test_Primask = 0;
    Motor_Para = (Motor_Para_T)MOTOR_PARA_DEFAULT;
    Control_Current_Tune_Source = CTRL_TUNE_BANDWIDTH;
    Control_Speed_Tune_Source = CTRL_TUNE_BANDWIDTH;
    Control_Current_Bw_Hz = CUR_BW_HZ_DEFAULT;
    Control_Speed_Bw_Hz = SPD_BW_HZ_DEFAULT;
    Id_Ctrl = (PID_T){0}; Iq_Ctrl = (PID_T){0};
    Speed_Ctrl = (PID_T){0}; Pos_Ctrl = (PID_T){0};
    Motor_Para_Update();
    Test_Reads = Test_Reads_Masked = Test_Writes = Test_Write_Fail = 0;
}
void Test_Setup(void) { nvs_clear(NULL); Test_Ram_Defaults(); }
int Test_Write(uint16_t Id, float Value)
{
    Parameter_Value_T Data;
    Parameter_Type_e Type;
    assert(Parameter_Read(Id, &Type, &Data) == PARAM_OK);
    if (Type == PARAM_U8) { Data.U8 = (uint8_t)Value; }
    else { Data.F32 = Value; }
    return Parameter_Write(Id, Type, Data);
}
float Test_Read(uint16_t Id)
{
    Parameter_Value_T Data;
    Parameter_Type_e Type;
    assert(Parameter_Read(Id, &Type, &Data) == PARAM_OK);
    return Type == PARAM_U8 ? (float)Data.U8 : Data.F32;
}
''')
        # Compile the actual command case and calibration save function rather
        # than copying their filtering/order policy into the test boundary.
        motor = (ROOT / "AZURE_RTOS/App/motor_thread.c").read_text()
        command = motor.split("case MOTOR_CMD_PARAMETER_SAVE:", 1)[1].split("default:", 1)[0]
        phase = (ROOT / "Motor/Servo_Phase.c").read_text()
        calibration = phase.split("static bool Calibration_Save(void)", 1)[1].split("bool Servo_Phase_Start", 1)[0]
        with source.open("a") as file:
            file.write("""
#define AXDR_OK 0
#define AXDR_ERR_STATE 2
#define AXDR_ERR_CONFIG 3
static void Motor_Action_Response(const int *Msg, int Status)
{
    (void)Msg; (void)Status;
}
int Test_Save_Config(void)
{
    int Action_Status = 0;
    int Msg = 0;
    switch (0) { case 0:
""" + command + """
    }
    return Action_Status;
}
bool Test_Calibration_Save(void)
""" + calibration)
        library = path / "tuning.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-Wl,--no-undefined", "-I" + str(path),
            *["-I" + name for name in ("Motor", "User", "Algo", "Observer",
              "Identification", "Motion", "Sensorless", "Parameter", "Storage")],
            "Parameter/Parameter.c", "Storage/NVS_Storage.c", "Motor/Motor_Para.c",
            str(source), "-lm", "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Test_Write.argtypes = [ct.c_uint16, ct.c_float]
        cls.lib.Test_Read.argtypes = [ct.c_uint16]
        cls.lib.Test_Read.restype = ct.c_float
        cls.lib.Test_Record.argtypes = [ct.c_uint16, ct.c_float, ct.c_int]
        cls.lib.Test_Record_Size.argtypes = [ct.c_uint16]
        cls.lib.NVS_Storage_Save.argtypes = [ct.POINTER(ct.c_uint16), ct.c_uint16, ct.c_int]
        cls.lib.NVS_Storage_Load.argtypes = [ct.c_uint16]
        cls.lib.Test_Calibration_Save.restype = ct.c_bool

    def setUp(self):
        self.lib.Test_Setup()

    def write(self, param, value):
        self.assertEqual(self.lib.Test_Write(param, value), 0)

    def values(self, group):
        return tuple(self.lib.Test_Read(param) for param in group[2])

    def manual(self, group):
        for n, param in enumerate(group[2]):
            self.write(param, n + 1.25)
        self.assertEqual(self.lib.Test_Read(group[0]), 1)
        return self.values(group)

    def count(self, name):
        return ct.c_int.in_dll(self.lib, name).value

    def test_save_bandwidth_preserves_manual_for_each_loop(self):
        saved = [self.manual(group) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        for group in GROUPS:
            self.write(group[1], self.lib.Test_Read(group[1]) * 0.75)
            self.assertEqual(self.lib.Test_Read(group[0]), 0)
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        for group, expected in zip(GROUPS, saved):
            self.write(group[0], 1)
            self.assertEqual(self.values(group), expected)
        self.assertEqual(self.count("Test_Reads_Masked"), 0)
        self.assertEqual(self.count("Test_Primask"), 0)

    def test_mixed_sources_save_independently_and_position_still_saves(self):
        for manual, bandwidth in (GROUPS, GROUPS[::-1]):
            with self.subTest(manual=manual[0]):
                self.setUp()
                expected = self.manual(manual)
                self.write(p.PARAM_CTRL_POSITION_KP, 21)
                self.assertEqual(self.lib.Test_Save_Config(), 0)
                for gain in bandwidth[2]:
                    self.assertEqual(self.lib.Test_Record_Size(gain), 0)
                self.lib.Test_Ram_Defaults()
                self.assertEqual(self.lib.NVS_Storage_Load(0xFFFF), 0)
                self.assertEqual(self.lib.Test_Read(manual[0]), 1)
                self.assertEqual(self.values(manual), expected)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_POSITION_KP), 21)

    def test_first_manual_keeps_current_gains_and_does_not_write_flash(self):
        for group in GROUPS:
            expected = self.values(group)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), expected)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_bad_saved_group_leaves_source_and_all_gains_unchanged(self):
        for group in GROUPS:
            for bad in ((0, 0), (float("nan"), 4), (float("inf"), 4),
                        (-1, 4), (1, 3), (1, 8), (0, -5)):
                with self.subTest(group=group[0], bad=bad):
                    self.setUp()
                    self.manual(group)
                    self.assertEqual(self.lib.Test_Save_Config(), 0)
                    self.write(group[0], 0)
                    before = self.values(group)
                    # Last member invalid: earlier reads must not publish gains.
                    self.lib.Test_Record(group[2][-1], *bad)
                    self.assertEqual(self.lib.Test_Write(group[0], 1), 6)
                    self.assertEqual(self.lib.Test_Read(group[0]), 0)
                    self.assertEqual(self.values(group), before)
                    self.assertEqual(self.count("Test_Primask"), 0)

    def test_repeating_manual_keeps_unsaved_edits(self):
        for group in GROUPS:
            self.manual(group)
            self.assertEqual(self.lib.Test_Save_Config(), 0)
            self.write(group[2][0], 99)
            expected = self.values(group)
            reads = self.count("Test_Reads")
            self.write(group[0], 1)
            self.assertEqual(self.values(group), expected)
            self.assertEqual(self.count("Test_Reads"), reads)
            self.write(group[0], 0)
            self.write(group[0], 1)
            self.assertEqual(self.values(group)[0], 1.25)

    def test_direct_gain_edit_does_not_reload_other_saved_gains(self):
        for group in GROUPS:
            self.manual(group)
            self.assertEqual(self.lib.Test_Save_Config(), 0)
            self.write(group[0], 0)
            calculated = self.values(group)
            reads = self.count("Test_Reads")
            self.write(group[2][0], 123)
            self.assertEqual(self.lib.Test_Read(group[0]), 1)
            self.assertEqual(self.values(group), (123,) + calculated[1:])
            self.assertEqual(self.count("Test_Reads"), reads)

    def test_run_or_invalid_source_rejected_before_flash_read(self):
        for group in GROUPS:
            before = self.values(group)
            self.assertEqual(self.lib.Test_Write(group[0], 2), 4)
            ct.c_int.in_dll(self.lib, "Test_State").value = 2
            self.assertEqual(self.lib.Test_Write(group[0], 1), 5)
            self.assertEqual(self.lib.Test_Write(group[2][0], before[0]), 5)
            self.assertEqual(self.lib.Test_Write(group[1], self.lib.Test_Read(group[1])), 5)
            ct.c_int.in_dll(self.lib, "Test_State").value = 0
            self.assertEqual(self.values(group), before)
            self.assertEqual(self.lib.Test_Read(group[0]), 0)
        self.assertEqual(self.count("Test_Reads"), 0)

    def test_boot_bandwidth_recalculates_and_manual_remains_recoverable(self):
        saved = [self.manual(group) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        for group in GROUPS:
            self.write(group[1], self.lib.Test_Read(group[1]) * 0.5)
        calculated = [self.values(group) for group in GROUPS]
        bandwidths = [self.lib.Test_Read(group[1]) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        self.lib.Test_Ram_Defaults()
        self.assertEqual(self.lib.NVS_Storage_Load(0xFFFF), 0)
        for group, gains, bw, manual in zip(GROUPS, calculated, bandwidths, saved):
            self.assertEqual(self.lib.Test_Read(group[0]), 0)
            self.assertEqual(self.lib.Test_Read(group[1]), bw)
            self.assertEqual(self.values(group), gains)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), manual)

    def test_boot_manual_and_motor_model_changes_keep_manual_gains(self):
        expected = [self.manual(group) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        self.lib.Test_Ram_Defaults()
        self.assertEqual(self.lib.NVS_Storage_Load(0xFFFF), 0)
        for param in (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ,
                      p.PARAM_MOTOR_J, p.PARAM_MOTOR_B, p.PARAM_MOTOR_FLUX):
            self.write(param, self.lib.Test_Read(param) * 1.1)
        for group, gains in zip(GROUPS, expected):
            self.assertEqual(self.lib.Test_Read(group[0]), 1)
            self.assertEqual(self.values(group), gains)

    def test_zero_gains_are_valid(self):
        for group in GROUPS:
            for gain in group[2]:
                self.lib.Test_Record(gain, 0, 4)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), (0,) * len(group[2]))

    def save(self, ids=(), select=0):
        data = (ct.c_uint16 * len(ids))(*ids) if ids else None
        return self.lib.NVS_Storage_Save(data, len(ids), select)

    def test_public_save_has_no_tuning_policy(self):
        self.assertEqual(self.lib.Test_Read(SPEED[0]), 0)
        self.assertEqual(self.save([SPEED[2][0]]), 0)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][0]), 4)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][1]), 0)
        self.assertEqual(self.save(select=1), 0)
        for group in GROUPS:
            for gain in group[2]:
                self.assertEqual(self.lib.Test_Record_Size(gain), 4)

    def test_empty_include_exclusion_and_duplicate_ids(self):
        self.assertEqual(self.save(), 0)
        self.assertEqual(self.count("Test_Writes"), 0)
        excluded = CURRENT[2] + SPEED[2]
        self.assertEqual(self.save(excluded, 1), 0)
        for gain in excluded:
            self.assertEqual(self.lib.Test_Record_Size(gain), 0)
        self.assertEqual(self.lib.Test_Record_Size(p.PARAM_CTRL_POSITION_KP), 4)
        self.assertEqual(self.save([SPEED[2][0], SPEED[2][0]]), 0)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][0]), 4)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][1]), 0)

    def test_invalid_selection_rejected_before_any_write(self):
        for invalid in (0xFFFF, 0xEEEE, p.PARAM_ADC_IA, p.PARAM_TARGET_SPEED):
            for select in (0, 1):
                self.assertEqual(self.save([SPEED[2][0], invalid], select), -22)
        self.assertEqual(self.save(select=2), -22)
        self.assertEqual(self.lib.NVS_Storage_Save(None, 1, 0), -22)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_single_load_validates_and_does_not_select_mode(self):
        gain = SPEED[2][0]
        before = self.lib.Test_Read(gain)
        self.assertEqual(self.lib.NVS_Storage_Load(gain), -2)
        self.assertEqual(self.lib.Test_Read(gain), before)
        self.lib.Test_Record(gain, 17, 4)
        self.assertEqual(self.lib.NVS_Storage_Load(gain), 0)
        self.assertEqual(self.lib.Test_Read(gain), 17)
        self.assertEqual(self.lib.Test_Read(SPEED[0]), 0)
        self.lib.Test_Record(gain, -1, 4)
        self.assertEqual(self.lib.NVS_Storage_Load(gain), -22)
        self.assertEqual(self.lib.Test_Read(gain), 17)
        for invalid in (0xEEEE, p.PARAM_ADC_IA, p.PARAM_TARGET_SPEED):
            self.assertEqual(self.lib.NVS_Storage_Load(invalid), -22)

    def test_config_save_requires_disabled(self):
        for state in (1, 2):
            ct.c_int.in_dll(self.lib, "Test_State").value = state
            self.assertEqual(self.lib.Test_Save_Config(), 2)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_manual_restore_is_allowed_while_enabled_without_masking_reads(self):
        expected = [self.manual(group) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        for group in GROUPS:
            self.write(group[0], 0)
        ct.c_int.in_dll(self.lib, "Test_State").value = 1
        for group, gains in zip(GROUPS, expected):
            self.write(group[0], 1)
            self.assertEqual(self.values(group), gains)
        self.assertEqual(self.count("Test_Reads_Masked"), 0)

    def test_calibration_preserves_valid_last_order_and_stops_on_write_error(self):
        expected = [p.PARAM_CAL_VALID, p.PARAM_MOTOR_PP, p.PARAM_ENCODER_PROTOCOL,
                    p.PARAM_ENCODER_SPI_TYPE, p.PARAM_CAL_ENC_DIR,
                    p.PARAM_CAL_THETA_OFF, p.PARAM_CAL_VALID]
        self.assertTrue(self.lib.Test_Calibration_Save())
        log = (ct.c_uint16 * 256).in_dll(self.lib, "Test_Write_Log")
        self.assertEqual(list(log[:self.count("Test_Writes")]), expected)
        for fail in (1, 3, 7):
            self.setUp()
            ct.c_int.in_dll(self.lib, "Test_Write_Fail").value = fail
            self.assertFalse(self.lib.Test_Calibration_Save())
            self.assertEqual(self.count("Test_Writes"), fail)
            self.assertEqual(list(log[:fail]), expected[:fail])
            if fail > 1:
                self.assertEqual(self.lib.NVS_Storage_Load(p.PARAM_CAL_VALID), 0)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 0)


if __name__ == "__main__":
    unittest.main()
