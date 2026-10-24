#!/usr/bin/env python3
"""Exercise real parameter, tuning and storage code with an in-memory NVS boundary."""

import ctypes as ct
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

import parameter_ids_generated as p
from test_mechanical_eso import ESO, State
from test_usb_protocol import function


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
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
struct nvs_fs {
    uint32_t offset;
    uint16_t sector_size;
    uint16_t sector_count;
    bool ready;
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
#include "Signal_Injection.h"
#include "control_params.h"
#include "motor_params.h"
#include <zephyr/kvss/nvs.h>

/* Unrelated control/hardware boundaries; parameter/tuning/storage are real. */
PID_T Id_Ctrl, Iq_Ctrl, Speed_Ctrl, Pos_Ctrl;
LADRC_T Id_LADRC, Iq_LADRC;
uint8_t Current_Controller = CURRENT_CTRL_PI;
float Control_LADRC_Bw_Hz = CUR_LADRC_BW_HZ_DEFAULT;
float Control_LADRC_Eso_Bw_Hz = CUR_LADRC_ESO_BW_HZ_DEFAULT;
volatile uint32_t Plot_Fast_Drop, Plot_Normal_Drop;
volatile ADC_T ADC;
volatile Encoder_T Encoder;
Encoder_Config_T Encoder_Config;
Encoder_Harmonic_T Encoder_Harmonic;
uint8_t Open_Loop_Method;
Motor_Config_T Motor_Config;
Motor_Cmd_T Motor_Cmd;
Motor_Cal_T Motor_Cal;
Motor_Run_T Motor_Run;
volatile Motor_Frf_Speed_T Motor_Frf_Speed;
volatile Signal_Injection_T Signal_Injection;
uint8_t Motor_Mode;
const Motor_Limit_T Motor_Lim = { .I_Max = 20.0f, .Wm_Max = 1000.0f };
Motor_Limit_T User_Lim;
Motion_Config_T Motion_Config;
volatile Protection_T Protection;
Servo_Phase_Config_T Servo_Phase_Config;
PLL_T Flux_PLL;
volatile float Ident_JB_Excite_Ratio, Ident_JB_Excite_Hz;
int Test_State, Test_Reads, Test_Reads_Masked, Test_Writes;
uint32_t Test_Primask;
uint16_t Test_Write_Log[256];
int Test_Write_Fail;
int Test_Mount_Status;
int Test_Encoder_Configs, Test_Encoder_Pp, Test_Encoder_Dir, Test_Encoder_Eso_Valid;

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
void Encoder_DMA_Config(void)
{
    Test_Encoder_Configs++;
    Test_Encoder_Pp = Motor_Para.Pp;
    Test_Encoder_Dir = Motor_Cal.Enc_Dir;
    Test_Encoder_Eso_Valid = Mechanical_ESO.Para.Valid;
    Mechanical_ESO.State = (Mechanical_ESO_State_T){0};
}
float Servo_Phase_Theta_Off_Error_Get(void) { return 0.0f; }
volatile uint8_t Ident_Fail_Reason;

/* Records model the documented nvs_read/write return values, including errors
 * and oversized records. This does not emulate Flash wear or power loss. */
static struct { unsigned char Data[8]; int Size; } Record[65536];
static struct nvs_fs *Test_Fs;
bool Flash_Storage_Geometry_Valid(void) { return true; }
int nvs_mount(struct nvs_fs *Fs)
{
    Test_Fs = Fs;
    assert(!Fs->ready);
    assert(Fs->offset == STORAGE_NVS_OFFSET);
    assert(Fs->sector_size == STORAGE_NVS_SECTOR_SIZE);
    assert(Fs->sector_count == STORAGE_NVS_SECTOR_COUNT);
    assert(Fs->flash_device == NULL);
    if (Test_Mount_Status != 0) { return Test_Mount_Status; }
    Fs->ready = true;
    return 0;
}
int nvs_clear(struct nvs_fs *Fs)
{
    if (Fs != NULL) { Fs->ready = false; }
    memset(Record, 0, sizeof(Record));
    return 0;
}
ssize_t nvs_read(struct nvs_fs *Fs, uint16_t Id, void *Data, size_t Size)
{
    assert(Fs->ready);
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
    assert(Fs->ready);
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
void Test_Record_Byte(uint16_t Id, uint8_t Value)
{
    Record[Id].Data[0] = Value;
    Record[Id].Size = 1;
}
int Test_Record_Size(uint16_t Id) { return Record[Id].Size; }
int Test_Record_U8(uint16_t Id)
{
    return (Record[Id].Size == 1) ? (int)Record[Id].Data[0] : -1;
}
void Test_Ram_Defaults(void)
{
    Test_State = DISABLED;
    Test_Primask = 0;
    Test_Mount_Status = 0;
    Test_Encoder_Configs = 0;
    Motor_Cal = (Motor_Cal_T)MOTOR_CAL_DEFAULT;
    Encoder_Config = (Encoder_Config_T)ENCODER_CONFIG_DEFAULT;
    Motion_Config = (Motion_Config_T){ .Wm_Acc = 100, .Wm_Dec = 100 };
    Motor_Para = (Motor_Para_T)MOTOR_PARA_DEFAULT;
    Control_Current_Tune_Source = CTRL_TUNE_BANDWIDTH;
    Control_Speed_Tune_Source = CTRL_TUNE_BANDWIDTH;
    Control_Current_Bw_Hz = CUR_BW_HZ_DEFAULT;
    Control_Speed_Bw_Hz = SPD_BW_HZ_DEFAULT;
    Id_Ctrl = (PID_T){0}; Iq_Ctrl = (PID_T){0};
    Speed_Ctrl = (PID_T){0}; Pos_Ctrl = (PID_T){0};
    Current_Manual = (Current_Manual_T){ ID_KP_DEFAULT, ID_KI_DEFAULT,
                                       IQ_KP_DEFAULT, IQ_KI_DEFAULT };
    Speed_Manual = (Speed_Manual_T){ SPD_KP_DEFAULT, SPD_KI_DEFAULT };
    Mechanical_ESO = (Mechanical_ESO_T){0};
    assert(Motor_Para_Update(&Motor_Para, MECH_ESO_BW_HZ_DEFAULT));
    Test_Reads = Test_Reads_Masked = Test_Writes = Test_Write_Fail = 0;
}
void Test_Setup(void) { nvs_clear(Test_Fs); Test_Ram_Defaults(); }
int Test_Write(uint16_t Id, float Value)
{
    Parameter_Value_T Data;
    Parameter_Type_e Type;
    assert(Parameter_Read(Id, &Type, &Data) == PARAM_OK);
    if (Type == PARAM_U8) { Data.U8 = (uint8_t)Value; }
    else if (Type == PARAM_I8) { Data.I8 = (int8_t)Value; }
    else { Data.F32 = Value; }
    return Parameter_Write(Id, Type, Data);
}
int Test_Restore(uint16_t Id, float Value)
{
    Parameter_Value_T Data = {0};
    Parameter_Type_e Type;
    assert(Parameter_Read(Id, &Type, &Data) == PARAM_OK);
    if (Type == PARAM_U8) { Data.U8 = (uint8_t)Value; }
    else if (Type == PARAM_I8) { Data.I8 = (int8_t)Value; }
    else { Data.F32 = Value; }
    return Parameter_Restore(Id, Type, Data);
}
float Test_Effective(uint16_t Id)
{
    switch (Id)
    {
        case PARAM_CTRL_ID_KP: return Id_Ctrl.Para.Kp;
        case PARAM_CTRL_ID_KI: return Id_Ctrl.Para.Ki;
        case PARAM_CTRL_IQ_KP: return Iq_Ctrl.Para.Kp;
        case PARAM_CTRL_IQ_KI: return Iq_Ctrl.Para.Ki;
        case PARAM_CTRL_SPEED_KP: return Speed_Ctrl.Para.Kp;
        case PARAM_CTRL_SPEED_KI: return Speed_Ctrl.Para.Ki;
        default: assert(0); return 0;
    }
}
float Test_Read(uint16_t Id)
{
    Parameter_Value_T Data;
    Parameter_Type_e Type;
    assert(Parameter_Read(Id, &Type, &Data) == PARAM_OK);
    if (Type == PARAM_U8) { return (float)Data.U8; }
    if (Type == PARAM_I8) { return (float)Data.I8; }
    return Data.F32;
}
''')
        # 编译实际命令分支和标定保存函数，验证真实的检查及保存顺序。
        motor = (ROOT / "AZURE_RTOS/App/motor_thread.c").read_text()
        # 截取实际启动配置代码，ThreadX 的资源创建不参与主机测试。
        boot = motor.split("UINT Motor_Thread_Init(VOID *memory_ptr)", 1)[1]
        boot = boot.split("    UINT Work_Result", 1)[0]
        boot = boot.replace("{", "{\n    void *memory_ptr = NULL;", 1)
        with source.open("a") as file:
            file.write("typedef int TX_BYTE_POOL; typedef char CHAR;\n"
                       "int Test_Boot(void)\n" + boot +
                       "    (void)byte_pool; (void)sizeof(pointer); return 0; }\n")
        command = motor.split("case MOTOR_CMD_PARAMETER_SAVE:", 1)[1].split("default:", 1)[0]
        phase = (ROOT / "Motor/Servo_Phase.c").read_text()
        calibration = function(phase, "Servo_Phase_Save")
        rl = (ROOT / "Identification/Rs_Ls.c").read_text()
        rough = rl[rl.index("static void Rough_RL_Apply("):rl.index("static void Fail(void)")]
        ident = (ROOT / "Identification/Identification.c").read_text()
        apply = function(ident, "Identification_Apply")
        with source.open("a") as file:
            file.write('''
#include "Identification.h"
#include "Rs_Ls.h"
#include "Flux.h"
#include "JB.h"
static bool RL_Temporary;
void Current_Loop_State_Reset(void) { }
''' + rough + '''
void Test_Rough(float Rs, float Ls) { Rough_RL_Apply(Rs, Ls); }
void Test_Rough_Restore(void) { Rough_RL_Restore(); }
volatile Ident_State_e Ident_State;
static Ident_Mode_e Ident_Mode;
Rs_Ls_Result_T Rs_Ls_Result;
Flux_Result_T Flux_Result;
JB_Result_T JB_Result;
''' + apply + '''
bool Test_Ident_Apply(int Mode, float First, float Second, bool Valid)
{
    Ident_State = Valid ? IDENT_DONE : IDENT_FAILED;
    Ident_Mode = (Ident_Mode_e)Mode;
    Rs_Ls_Result = (Rs_Ls_Result_T){ First, Second, Valid };
    Flux_Result = (Flux_Result_T){ First, Valid };
    JB_Result = (JB_Result_T){ First, Second, Valid };
    return Identification_Apply();
}
''')
            file.write("""
#define AXDR_OK 0
#define AXDR_ERR_STATE 2
#define AXDR_ERR_CONFIG 3
#define MOTOR_WORK_IDLE 0
#define MOTOR_WORK_SAVE_ALL 1
static int Motor_Work_State, Save_Action;
static bool Save_Action_Pending;
static int Test_Response;
static bool Enc_Control_Active(void) { return false; }
static bool Motor_Work_Request(int Job)
{ assert(Job == MOTOR_WORK_SAVE_ALL); Motor_Work_State = 1; return true; }
static void Motor_Action_Response(int Msg, int Status)
{ (void)Msg; Test_Response = Status; }
int Test_Save_Config(void)
{
    struct { int Reserved; } Msg = {0};
    Test_Response = -1;
    switch (0) { case 0:
""" + command + """
    }
    if (Save_Action_Pending)
    {
        Test_Response = NVS_Storage_Save_All() == 0 ? AXDR_OK : AXDR_ERR_CONFIG;
        Save_Action_Pending = false;
        Motor_Work_State = MOTOR_WORK_IDLE;
    }
    return Test_Response;
}
""" + calibration + """
bool Test_Calibration_Save(void) { return Servo_Phase_Save(); }
""")
        library = path / "tuning.so"
        subprocess.run([
            "cc", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
            "-Wl,--no-undefined", "-I" + str(path),
            *["-I" + name for name in ("Motor", "User", "Algo", "Observer",
              "Identification", "Motion", "Sensorless", "Parameter", "Storage", "Comm")],
            "Parameter/Parameter.c", "Storage/NVS_Storage.c", "Motor/Motor_Para.c",
            "Observer/Mechanical_ESO.c", "Algo/Math.c",
            str(source), "-lm", "-o", str(library),
        ], cwd=ROOT, check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Test_Write.argtypes = [ct.c_uint16, ct.c_float]
        cls.lib.Test_Restore.argtypes = [ct.c_uint16, ct.c_float]
        cls.lib.Test_Record_Byte.argtypes = [ct.c_uint16, ct.c_uint8]
        cls.lib.Test_Read.argtypes = [ct.c_uint16]
        cls.lib.Test_Read.restype = ct.c_float
        cls.lib.Test_Effective.argtypes = [ct.c_uint16]
        cls.lib.Test_Effective.restype = ct.c_float
        cls.lib.Test_Record.argtypes = [ct.c_uint16, ct.c_float, ct.c_int]
        cls.lib.Test_Record_Size.argtypes = [ct.c_uint16]
        cls.lib.NVS_Storage_Save_Ids.argtypes = [ct.POINTER(ct.c_uint16), ct.c_uint16]
        cls.lib.Test_Record_U8.argtypes = [ct.c_uint16]
        cls.lib.Test_Calibration_Save.restype = ct.c_bool
        cls.lib.Test_Rough.argtypes = [ct.c_float, ct.c_float]
        cls.lib.Test_Ident_Apply.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_bool]
        cls.lib.Test_Ident_Apply.restype = ct.c_bool

    def setUp(self):
        self.lib.Test_Setup()

    def write(self, param, value):
        self.assertEqual(self.lib.Test_Write(param, value), 0)

    def values(self, group):
        return tuple(self.lib.Test_Read(param) for param in group[2])

    def effective(self, group):
        return tuple(self.lib.Test_Effective(param) for param in group[2])

    def manual(self, group):
        for n, param in enumerate(group[2]):
            self.write(param, n + 1.25)
        self.assertEqual(self.lib.Test_Read(group[0]), 1)
        self.assertEqual(self.effective(group), self.values(group))
        return self.values(group)

    def count(self, name):
        return ct.c_int.in_dll(self.lib, name).value

    def test_load_only_restores_ram_without_updating_control_or_encoder(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        eso.State = State(1, 30, .002, .0001)
        before = bytes(eso)
        gains = [self.effective(group) for group in GROUPS]
        rs = self.lib.Test_Read(p.PARAM_MOTOR_RS) * 2
        self.lib.Test_Record(p.PARAM_MOTOR_RS, rs, 4)
        self.lib.Test_Record(p.PARAM_CTRL_SPEED_KP, 17, 4)
        self.lib.Test_Record_Byte(p.PARAM_CTRL_SPEED_SOURCE, 1)
        self.lib.Test_Record_Byte(p.PARAM_ENCODER_SPI_TYPE, 2)
        self.lib.Test_Record_Byte(p.PARAM_CAL_VALID, 1)
        self.assertEqual(self.lib.NVS_Storage_Load_All(), 0)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_MOTOR_RS), rs)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_SPEED_KP), 17)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_SPEED_SOURCE), 1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_ENCODER_SPI_TYPE), 2)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        self.assertEqual([self.effective(group) for group in GROUPS], gains)
        self.assertEqual(bytes(eso), before)
        self.assertEqual(self.count("Test_Encoder_Configs"), 0)

    def test_empty_flash_boot_builds_coefficients_before_encoder_config(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        expected = [getattr(eso.Para, name) for name, _ in eso.Para._fields_]
        ct.memset(ct.addressof(eso), 0, ct.sizeof(eso))
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual([getattr(eso.Para, name) for name, _ in eso.Para._fields_], expected)
        self.assertEqual(self.count("Test_Encoder_Configs"), 1)
        self.assertEqual(self.count("Test_Encoder_Eso_Valid"), 1)
        self.assertEqual(self.count("Test_Encoder_Pp"), self.lib.Test_Read(p.PARAM_MOTOR_PP))
        self.assertEqual(self.count("Test_Writes"), 0)
        self.assertEqual(self.count("Test_Reads_Masked"), 0)

    def test_storage_error_does_not_skip_default_motor_configuration(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        ct.memset(ct.addressof(eso), 0, ct.sizeof(eso))
        ct.c_int.in_dll(self.lib, "Test_Mount_Status").value = -5
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(eso.Para.Valid, 1)
        self.assertEqual(self.count("Test_Encoder_Configs"), 1)
        self.assertEqual(self.count("Test_Reads"), 0)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_motion_profile_permissions_and_persistence(self):
        profile = p.PARAM_MOTION_PROFILE
        state = ct.c_int.in_dll(self.lib, "Test_State")
        self.assertEqual(self.lib.Test_Read(profile), 0)
        for allowed in (0, 1):
            state.value = allowed
            for value in (1, 2, 0):
                self.write(profile, value)
        state.value = 2
        self.assertNotEqual(self.lib.Test_Write(profile, 1), 0)
        self.assertEqual(self.lib.Test_Read(profile), 0)
        state.value = 0
        self.assertNotEqual(self.lib.Test_Write(profile, 3), 0)
        self.write(profile, 2)
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        self.lib.Test_Ram_Defaults()
        self.assertEqual(self.lib.Test_Read(profile), 0)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(profile), 2)
        self.lib.Test_Setup()
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(profile), 0)

    def test_save_bandwidth_preserves_manual_for_each_loop(self):
        saved = [self.manual(group) for group in GROUPS]
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
                    self.assertEqual(self.lib.Test_Record_Size(gain), 4)
                self.lib.Test_Ram_Defaults()
                self.assertEqual(self.lib.Test_Boot(), 0)
                self.assertEqual(self.lib.Test_Read(manual[0]), 1)
                self.assertEqual(self.values(manual), expected)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_POSITION_KP), 21)

    def test_first_manual_publishes_defaults_without_flash_access(self):
        for group in GROUPS:
            expected = self.values(group)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), expected)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_switch_ignores_missing_or_bad_flash_records(self):
        for group in GROUPS:
            for bad in ((0, 0), (float("nan"), 4), (-1, 4), (1, 3), (1, 8), (0, -5)):
                with self.subTest(group=group[0], bad=bad):
                    self.setUp()
                    expected = self.manual(group)
                    self.write(group[0], 0)
                    self.lib.Test_Record(group[2][-1], *bad)
                    self.write(group[0], 1)
                    self.assertEqual(self.values(group), expected)
                    self.assertEqual(self.effective(group), expected)
                    self.assertEqual(self.count("Test_Reads"), 0)

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
            self.assertEqual(self.values(group), expected)
            self.assertEqual(self.effective(group), expected)
            self.assertEqual(self.count("Test_Reads"), reads)

    def test_direct_gain_edit_does_not_reload_other_saved_gains(self):
        for group in GROUPS:
            self.manual(group)
            self.assertEqual(self.lib.Test_Save_Config(), 0)
            self.write(group[0], 0)
            manual = self.values(group)
            reads = self.count("Test_Reads")
            self.write(group[2][0], 123)
            self.assertEqual(self.lib.Test_Read(group[0]), 1)
            self.assertEqual(self.values(group), (123,) + manual[1:])
            self.assertEqual(self.effective(group), self.values(group))
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
        for group in GROUPS:
            self.write(group[1], self.lib.Test_Read(group[1]) * 0.5)
        calculated = [self.effective(group) for group in GROUPS]
        bandwidths = [self.lib.Test_Read(group[1]) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        self.lib.Test_Ram_Defaults()
        self.assertEqual(self.lib.Test_Boot(), 0)
        for group, gains, bw, manual in zip(GROUPS, calculated, bandwidths, saved):
            self.assertEqual(self.lib.Test_Read(group[0]), 0)
            self.assertEqual(self.lib.Test_Read(group[1]), bw)
            self.assertEqual(self.effective(group), gains)
            self.assertEqual(self.values(group), manual)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), manual)

    def test_boot_manual_and_motor_model_changes_keep_manual_gains(self):
        expected = [self.manual(group) for group in GROUPS]
        self.assertEqual(self.lib.Test_Save_Config(), 0)
        self.lib.Test_Ram_Defaults()
        self.assertEqual(self.lib.Test_Boot(), 0)
        for param in (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ,
                      p.PARAM_MOTOR_J, p.PARAM_MOTOR_B, p.PARAM_MOTOR_FLUX):
            self.write(param, self.lib.Test_Read(param) * 1.1)
        for group, gains in zip(GROUPS, expected):
            self.assertEqual(self.lib.Test_Read(group[0]), 1)
            self.assertEqual(self.values(group), gains)
            self.assertEqual(self.effective(group), gains)

    def test_zero_gains_are_valid(self):
        for group in GROUPS:
            for gain in group[2]:
                self.write(gain, 0)
            self.write(group[0], 1)
            self.assertEqual(self.values(group), (0,) * len(group[2]))

    def test_equal_value_writes_select_source_and_preserve_manual(self):
        for group in GROUPS:
            manual = self.values(group)
            bw = self.lib.Test_Read(group[1])
            computed = self.effective(group)
            self.write(group[2][0], manual[0])
            self.assertEqual(self.lib.Test_Read(group[0]), 1)
            self.assertEqual(self.effective(group), manual)
            self.write(group[1], bw)
            self.assertEqual(self.lib.Test_Read(group[0]), 0)
            self.assertEqual(self.effective(group), computed)
            self.assertEqual(self.values(group), manual)

    def test_bandwidth_model_change_updates_only_effective(self):
        manual = [self.values(group) for group in GROUPS]
        effective = [self.effective(group) for group in GROUPS]
        for param in (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ,
                      p.PARAM_MOTOR_J, p.PARAM_MOTOR_B):
            self.write(param, self.lib.Test_Read(param) * 2)
        for group, old_manual, old_effective in zip(GROUPS, manual, effective):
            self.assertEqual(self.values(group), old_manual)
            for actual, old in zip(self.effective(group), old_effective):
                self.assertAlmostEqual(actual, old * 2, delta=abs(old) * 1e-5)

    def test_speed_effective_read_only_and_not_persistent(self):
        ids = (p.PARAM_CTRL_SPEED_KP_EFFECTIVE, p.PARAM_CTRL_SPEED_KI_EFFECTIVE)
        for source in (0, 1):
            self.write(SPEED[0], source)
            self.assertEqual(tuple(self.lib.Test_Read(i) for i in ids), self.effective(SPEED))
            for param in ids:
                self.assertEqual(self.lib.Test_Write(param, 1), 3)
                self.assertEqual(self.save_ids([param]), -22)

    def test_eso_accepts_finite_gains_and_rejects_numeric_overflow(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        eso.State = State(1, 30, .002, .0001)
        old_state = bytes(eso.State)

        # A negative L1 at low bandwidth is not itself an invalid model.
        self.write(p.PARAM_CTRL_MECH_ESO_BW_HZ, 100)
        self.write(p.PARAM_MOTOR_B, self.lib.Test_Read(p.PARAM_MOTOR_J) * 1000)
        self.write(p.PARAM_CTRL_MECH_ESO_BW_HZ, 1)
        self.assertLess(eso.Para.L1, 0)
        self.assertEqual(bytes(eso.State), old_state)

        # Mathematically unrepresentable float coefficients remain rejected.
        for param, value in ((p.PARAM_MOTOR_J, 1e-40),
                             (p.PARAM_MOTOR_FLUX, 3e38)):
            before = bytes(eso)
            old = self.lib.Test_Read(param)
            self.assertEqual(self.lib.Test_Write(param, value), 4)
            self.assertEqual(self.lib.Test_Read(param), old)
            self.assertEqual(bytes(eso), before)

        self.write(p.PARAM_MOTOR_FLUX, 1e32)
        before = bytes(eso)
        old_pp = self.lib.Test_Read(p.PARAM_MOTOR_PP)
        self.assertEqual(self.lib.Test_Write(p.PARAM_MOTOR_PP, 255), 4)
        self.assertEqual(self.lib.Test_Read(p.PARAM_MOTOR_PP), old_pp)
        self.assertEqual(bytes(eso), before)

    def test_nvs_restores_complete_model_before_eso_update(self):
        # J and B belong to one saved motor model, not two successive updates.
        self.write(p.PARAM_CTRL_MECH_ESO_BW_HZ, 100)
        self.write(p.PARAM_MOTOR_B, self.lib.Test_Read(p.PARAM_MOTOR_J) * 100)
        self.lib.Test_Record(p.PARAM_MOTOR_J, 1e-8, 4)
        self.lib.Test_Record(p.PARAM_MOTOR_B, 1e-8, 4)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_MOTOR_J), 1e-8, delta=1e-14)
        self.assertAlmostEqual(ESO.in_dll(self.lib, "Mechanical_ESO").Para.B_Over_J, 1)

    def test_boot_unrepresentable_model_keeps_model_and_configures_encoder(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        eso.State = State(1, 30, .002, .0001)
        before, old_j = bytes(eso.Para), self.lib.Test_Read(p.PARAM_MOTOR_J)
        self.lib.Test_Record_Byte(p.PARAM_CAL_VALID, 1)
        self.lib.Test_Record(p.PARAM_MOTOR_J, 1e-40, 4)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_MOTOR_J), old_j)
        self.assertEqual(bytes(eso.Para), before)
        self.assertEqual(self.count("Test_Encoder_Configs"), 1)
        self.assertEqual(bytes(eso.State), bytes(State()))
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)

    def test_boot_restores_calibration_with_saved_pole_pairs(self):
        for valid in (0, 1):
            with self.subTest(valid=valid):
                self.lib.Test_Setup()
                pp = int(self.lib.Test_Read(p.PARAM_MOTOR_PP)) + 1
                self.write(p.PARAM_MOTOR_PP, pp)
                for ident, value in ((p.PARAM_CAL_ENC_DIR, -1),
                                     (p.PARAM_CAL_THETA_OFF, 1.25),
                                     (p.PARAM_CAL_VALID, valid)):
                    self.assertEqual(self.lib.Test_Restore(ident, value), 0)
                self.assertEqual(self.lib.NVS_Storage_Save_All(), 0)
                self.lib.Test_Ram_Defaults()
                self.assertEqual(self.lib.Test_Boot(), 0)
                self.assertEqual(self.lib.Test_Read(p.PARAM_MOTOR_PP), pp)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_ENC_DIR), -1)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_THETA_OFF), 1.25)
                self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), valid)
                self.assertEqual(self.count("Test_Encoder_Configs"), 1)
                self.assertEqual(self.count("Test_Encoder_Pp"), pp)
                self.assertEqual(self.count("Test_Encoder_Dir"), -1)
                self.assertEqual(self.count("Test_Encoder_Eso_Valid"), 1)

    def test_calibration_records_restore_independently(self):
        # 有效标记按保存值恢复，不额外要求方向、偏置记录同时存在。
        self.lib.Test_Record_Byte(p.PARAM_CAL_VALID, 1)
        direction = self.lib.Test_Read(p.PARAM_CAL_ENC_DIR)
        offset = self.lib.Test_Read(p.PARAM_CAL_THETA_OFF)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_ENC_DIR), direction)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_THETA_OFF), offset)
        self.lib.Test_Record_Byte(p.PARAM_CAL_ENC_DIR, 0)
        self.lib.Test_Record(p.PARAM_CAL_THETA_OFF, math.nan, 4)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_ENC_DIR), direction)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_THETA_OFF), offset)
        # 没有有效标记时，方向和偏置也可以独立恢复。
        self.lib.Test_Setup()
        self.lib.Test_Record_Byte(p.PARAM_CAL_ENC_DIR, 255)
        self.lib.Test_Record(p.PARAM_CAL_THETA_OFF, 2.5, 4)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_ENC_DIR), -1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_THETA_OFF), 2.5)

    def test_only_changed_accepted_pole_pairs_clear_calibration(self):
        self.assertEqual(self.lib.Test_Restore(p.PARAM_CAL_VALID, 1), 0)
        self.assertEqual(self.lib.Test_Restore(p.PARAM_CAL_ENC_DIR, -1), 0)
        self.assertEqual(self.lib.Test_Restore(p.PARAM_CAL_THETA_OFF, 1.25), 0)
        pp = int(self.lib.Test_Read(p.PARAM_MOTOR_PP))
        self.write(p.PARAM_MOTOR_PP, pp)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        self.assertEqual(self.lib.Test_Write(p.PARAM_MOTOR_PP, 0), 4)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        ct.c_int.in_dll(self.lib, "Test_State").value = 1
        self.assertEqual(self.lib.Test_Write(p.PARAM_MOTOR_PP, pp + 1), 5)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        ct.c_int.in_dll(self.lib, "Test_State").value = 0
        self.write(p.PARAM_MOTOR_PP, pp + 1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_ENC_DIR), -1)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_THETA_OFF), 1.25)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_rejected_model_keeps_calibration_encoder_changes_invalidate(self):
        self.assertEqual(self.lib.Test_Restore(p.PARAM_CAL_VALID, 1), 0)
        self.write(p.PARAM_MOTOR_FLUX, 1e32)
        self.assertEqual(self.lib.Test_Write(p.PARAM_MOTOR_PP, 255), 4)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 1)
        spi = int(self.lib.Test_Read(p.PARAM_ENCODER_SPI_TYPE))
        self.write(p.PARAM_ENCODER_SPI_TYPE, 1 if spi == 2 else 2)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 0)
        self.assertEqual(self.lib.Test_Restore(p.PARAM_CAL_VALID, 1), 0)
        self.write(p.PARAM_ENCODER_PROTOCOL, 0)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CAL_VALID), 0)
        self.assertEqual(self.count("Test_Encoder_Configs"), 2)

    def test_identification_temporary_current_gains_restore_source(self):
        for source in (0, 1):
            self.setUp()
            manual = self.manual(CURRENT)
            self.write(CURRENT[0], source)
            before = self.effective(CURRENT)
            model = tuple(self.lib.Test_Read(i) for i in
                          (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ))
            bw = self.lib.Test_Read(CURRENT[1])
            self.lib.Test_Rough(.1, .0001)

            self.assertEqual(self.lib.Test_Read(CURRENT[0]), source)
            expected = (.0001 * 2 * math.pi * bw, .1 * 2 * math.pi * bw) * 2
            for actual, target in zip(self.effective(CURRENT), expected):
                self.assertAlmostEqual(actual, target, delta=max(1e-6, abs(target) * 1e-5))
            self.assertEqual(self.values(CURRENT), manual)
            self.assertEqual(tuple(self.lib.Test_Read(i) for i in
                                   (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ)), model)

            self.lib.Test_Rough(.2, .0002)
            self.lib.Test_Rough_Restore()
            self.assertEqual(self.lib.Test_Read(CURRENT[0]), source)
            self.assertEqual(self.effective(CURRENT), before)
            self.assertEqual(self.values(CURRENT), manual)
            self.lib.Test_Rough_Restore()
            self.assertEqual(self.effective(CURRENT), before)

    def test_identification_temporary_current_gains_zero_bandwidth(self):
        self.assertEqual(self.lib.Test_Write(p.PARAM_CTRL_CURRENT_BW_HZ, 0), 4)
        # 参数写入接口不接受零带宽，因此直接修改 RAM，测试内部的异常处理。
        ct.c_float.in_dll(self.lib, "Control_Current_Bw_Hz").value = 0
        self.write(p.PARAM_CTRL_CURRENT_SOURCE, 1)
        before = self.effective(CURRENT)
        self.lib.Test_Rough(.2, .0003)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_CURRENT_SOURCE), 1)

        self.assertEqual(self.effective(CURRENT), (0.0, 0.0, 0.0, 0.0))

        self.lib.Test_Rough_Restore()
        self.assertEqual(self.effective(CURRENT), before)
        self.assertEqual(self.lib.Test_Read(p.PARAM_CTRL_CURRENT_BW_HZ), 0)

    def test_identification_apply_validates_complete_model(self):
        eso = ESO.in_dll(self.lib, "Mechanical_ESO")
        before = bytes(eso)
        ids = (p.PARAM_MOTOR_RS, p.PARAM_MOTOR_LD, p.PARAM_MOTOR_LQ,
               p.PARAM_MOTOR_FLUX, p.PARAM_MOTOR_J, p.PARAM_MOTOR_B)
        model = tuple(self.lib.Test_Read(i) for i in ids)
        for mode, valid in ((0, True), (4, True), (1, False), (2, False), (3, False)):
            self.assertFalse(self.lib.Test_Ident_Apply(mode, .1, .0001, valid))
            self.assertEqual(tuple(self.lib.Test_Read(i) for i in ids), model)
            self.assertEqual(bytes(eso), before)
        old_j = self.lib.Test_Read(p.PARAM_MOTOR_J)
        self.assertFalse(self.lib.Test_Ident_Apply(3, 1e-40, 1, True))
        self.assertEqual(self.lib.Test_Read(p.PARAM_MOTOR_J), old_j)
        self.assertEqual(bytes(eso), before)
        self.assertTrue(self.lib.Test_Ident_Apply(3, 1e-5, 1e-5, True))
        self.assertAlmostEqual(eso.Para.B_Over_J, 1)
        self.assertTrue(self.lib.Test_Ident_Apply(2, .002, 0, True))
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_MOTOR_FLUX), .002)
        self.assertTrue(self.lib.Test_Ident_Apply(1, .1, .0001, True))
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_MOTOR_RS), .1)

    def test_identification_results_read_from_parameter_bindings(self):
        self.assertTrue(self.lib.Test_Ident_Apply(1, 0.12, 0.00008, True))
        self.assertEqual(self.lib.Test_Read(p.PARAM_IDENT_RS_LS_VALID), 1)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_IDENT_RS_RESULT), 0.12, places=6)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_IDENT_LS_RESULT), 0.00008, places=8)

        self.assertTrue(self.lib.Test_Ident_Apply(2, 0.002, 0, True))
        self.assertEqual(self.lib.Test_Read(p.PARAM_IDENT_FLUX_VALID), 1)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_IDENT_FLUX_RESULT), 0.002, places=7)

        self.assertTrue(self.lib.Test_Ident_Apply(3, 0.00001, 0.00001, True))
        self.assertEqual(self.lib.Test_Read(p.PARAM_IDENT_JB_VALID), 1)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_IDENT_J_RESULT), 0.00001, places=9)
        self.assertAlmostEqual(self.lib.Test_Read(p.PARAM_IDENT_B_RESULT), 0.00001, places=9)

    def save_ids(self, ids=()):
        data = (ct.c_uint16 * len(ids))(*ids) if ids else None
        return self.lib.NVS_Storage_Save_Ids(data, len(ids))

    def test_save_all_preserves_manual_in_bandwidth_mode(self):
        self.assertEqual(self.lib.Test_Read(SPEED[0]), 0)
        self.assertEqual(self.save_ids([SPEED[2][0]]), 0)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][0]), 4)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][1]), 0)
        self.assertEqual(self.lib.NVS_Storage_Save_All(), 0)
        for group in GROUPS:
            for gain in group[2]:
                self.assertEqual(self.lib.Test_Record_Size(gain), 4)

    def test_empty_and_duplicate_ids(self):
        self.assertEqual(self.save_ids(), 0)
        self.assertEqual(self.count("Test_Writes"), 0)
        self.assertEqual(self.save_ids([SPEED[2][0], SPEED[2][0]]), 0)
        self.assertEqual(self.count("Test_Writes"), 2)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][0]), 4)
        self.assertEqual(self.lib.Test_Record_Size(SPEED[2][1]), 0)

    def test_save_ids_stops_at_invalid_id_and_keeps_saved_values(self):
        gain, following = SPEED[2]
        for invalid in (0xFFFF, 0xEEEE, p.PARAM_ADC_IA, p.PARAM_TARGET_SPEED):
            with self.subTest(invalid=invalid):
                self.lib.Test_Setup()
                self.assertEqual(self.save_ids([invalid, gain]), -22)
                self.assertEqual(self.count("Test_Writes"), 0)
                self.write(gain, 17)
                self.assertEqual(self.save_ids([gain, invalid, following]), -22)
                self.assertEqual(self.count("Test_Writes"), 1)
                self.assertEqual(self.lib.Test_Record_Size(gain), 4)
                self.assertEqual(self.lib.Test_Record_Size(following), 0)
                self.lib.Test_Ram_Defaults()
                self.assertEqual(self.lib.Test_Boot(), 0)
                self.assertEqual(self.lib.Test_Read(gain), 17)
        self.assertEqual(self.lib.NVS_Storage_Save_Ids(None, 1), -22)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_full_load_skips_missing_and_invalid_gain_records(self):
        gain = SPEED[2][0]
        invalid = SPEED[2][1]
        old_invalid = self.lib.Test_Read(invalid)
        self.lib.Test_Record(gain, 17, 4)
        self.lib.Test_Record(invalid, -1, 4)
        self.assertEqual(self.lib.Test_Boot(), 0)
        self.assertEqual(self.lib.Test_Read(gain), 17)
        self.assertEqual(self.lib.Test_Read(invalid), old_invalid)
        self.assertEqual(self.lib.Test_Read(SPEED[0]), 0)

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
            self.assertEqual(self.effective(group), gains)
        self.assertEqual(self.count("Test_Reads_Masked"), 0)

    def test_retired_phase_parameter_is_not_readable_or_saved(self):
        param_type = ct.c_int()
        value = ct.create_string_buffer(8)
        self.assertEqual(self.lib.Parameter_Read(0x0411, ct.byref(param_type), value), 1)
        self.assertNotEqual(self.save_ids([0x0411]), 0)
        self.assertEqual(self.count("Test_Writes"), 0)

    def test_calibration_saves_ram_values_once_and_stops_on_write_error(self):
        class Calibration(ct.Structure):
            _fields_ = [("dir", ct.c_int8), ("offset", ct.c_float), ("valid", ct.c_uint8)]

        cal = Calibration.in_dll(self.lib, "Motor_Cal")
        cal.dir, cal.offset, cal.valid = -1, .25, 1
        expected = [p.PARAM_CAL_ENC_DIR, p.PARAM_CAL_THETA_OFF, p.PARAM_CAL_VALID]
        self.assertTrue(self.lib.Test_Calibration_Save())
        log = (ct.c_uint16 * 256).in_dll(self.lib, "Test_Write_Log")
        self.assertEqual(list(log[:self.count("Test_Writes")]), expected)
        self.assertEqual(self.lib.Test_Record_U8(p.PARAM_CAL_VALID), 1)
        self.assertEqual((cal.dir, cal.offset, cal.valid), (-1, .25, 1))
        for fail in (1, 2, 3):
            self.setUp()
            cal.dir, cal.offset, cal.valid = -1, .25, 1
            ct.c_int.in_dll(self.lib, "Test_Write_Fail").value = fail
            self.assertFalse(self.lib.Test_Calibration_Save())
            self.assertEqual(self.count("Test_Writes"), fail)
            self.assertEqual(list(log[:fail]), expected[:fail])
            self.assertEqual((cal.dir, cal.offset, cal.valid), (-1, .25, 1))
            self.assertEqual(self.lib.Test_Record_U8(p.PARAM_CAL_VALID), -1)
            ct.c_int.in_dll(self.lib, "Test_Write_Fail").value = 0
            self.assertTrue(self.lib.Test_Calibration_Save())
            self.assertEqual(self.lib.Test_Record_U8(p.PARAM_CAL_VALID), 1)


if __name__ == "__main__":
    unittest.main()
