/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "NVS_Storage.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "Encoder.h"
#include "Flash_Storage.h"
#include "Mechanical_ESO.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "Parameter.h"
#include <zephyr/kvss/nvs.h>

#define STORAGE_SMOKE_ID 0x7FF0U

static struct nvs_fs Storage_NVS;
static bool Storage_Mounted = false;

volatile int NVS_Storage_Smoke_Result = 0;
volatile uint32_t NVS_Storage_Smoke_Readback = 0U;

int NVS_Storage_Init(void)
{
    int Status;

    if (Storage_Mounted)
    {
        return 0;
    }

    if (!Flash_Storage_Geometry_Valid())
    {
        return -ENOTSUP;
    }

    Storage_NVS.offset = STORAGE_NVS_OFFSET;
    Storage_NVS.sector_size = STORAGE_NVS_SECTOR_SIZE;
    Storage_NVS.sector_count = STORAGE_NVS_SECTOR_COUNT;
    Storage_NVS.flash_device = NULL;

    Status = nvs_mount(&Storage_NVS);
    if (Status == 0)
    {
        Storage_Mounted = true;
    }

    return Status;
}

static bool Calibration_Load(void)
{
    Parameter_Value_T Enc_Dir = { 0 };
    Parameter_Value_T Theta_Off = { 0 };
    Parameter_Value_T Valid = { 0 };
    ssize_t Rc;

    Motor_Cal.Valid = 0U;

    Rc = nvs_read(&Storage_NVS, PARAM_CAL_ENC_DIR, &Enc_Dir.I8, sizeof(Enc_Dir.I8));
    if ((Rc != (ssize_t)sizeof(Enc_Dir.I8)) ||
        ((Enc_Dir.I8 != -1) && (Enc_Dir.I8 != 1)))
    {
        return false;
    }

    Rc = nvs_read(&Storage_NVS, PARAM_CAL_THETA_OFF, &Theta_Off.F32, sizeof(Theta_Off.F32));
    if ((Rc != (ssize_t)sizeof(Theta_Off.F32)) ||
        !__builtin_isfinite(Theta_Off.F32))
    {
        return false;
    }

    Rc = nvs_read(&Storage_NVS, PARAM_CAL_VALID, &Valid.U8, sizeof(Valid.U8));
    if ((Rc != (ssize_t)sizeof(Valid.U8)) || (Valid.U8 != 1U))
    {
        return false;
    }

    if ((Parameter_Restore(PARAM_CAL_ENC_DIR, PARAM_I8, Enc_Dir) != PARAM_OK) ||
        (Parameter_Restore(PARAM_CAL_THETA_OFF, PARAM_FLOAT, Theta_Off) != PARAM_OK) ||
        (Parameter_Restore(PARAM_CAL_VALID, PARAM_U8, Valid) != PARAM_OK))
    {
        Motor_Cal.Valid = 0U;
        return false;
    }

    return true;
}

int NVS_Storage_Load_All(void)
{
    Motor_Para_T Model = Motor_Para;
    float Eso_Bw_Hz = Mechanical_ESO_Bw_Hz;
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    uint32_t Index = 0U;
    uint16_t Id;
    uint8_t Size;
    ssize_t Rc;
    int Status;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    while (Parameter_Persistent_Next(&Index, &Id, &Type, &Value))
    {
        if ((Id == PARAM_CAL_VALID) ||
            (Id == PARAM_CAL_ENC_DIR) ||
            (Id == PARAM_CAL_THETA_OFF))
        {
            continue;
        }

        Size = Parameter_Value_Size(Type);
        if (Size == 0U)
        {
            continue;
        }

        memset(&Value, 0, sizeof(Value));
        Rc = nvs_read(&Storage_NVS, Id, &Value, Size);
        if ((Rc != (ssize_t)Size) ||
            (Parameter_Check(Id, Type, Value) != PARAM_OK))
        {
            continue;
        }

        /* Restore the complete motor model before updating dependent gains
         * and ESO coefficients; the persistent ID order is not a model update. */
        switch (Id)
        {
            case PARAM_MOTOR_PP: Model.Pp = Value.U8; break;
            case PARAM_MOTOR_RS: Model.Rs = Value.F32; break;
            case PARAM_MOTOR_LD: Model.Ld = Value.F32; break;
            case PARAM_MOTOR_LQ: Model.Lq = Value.F32; break;
            case PARAM_MOTOR_FLUX: Model.Flux = Value.F32; break;
            case PARAM_MOTOR_J: Model.J = Value.F32; break;
            case PARAM_MOTOR_B: Model.B = Value.F32; break;
            case PARAM_CTRL_MECH_ESO_BW_HZ: Eso_Bw_Hz = Value.F32; break;
            default:
                (void)Parameter_Restore(Id, Type, Value);
                break;
        }
    }

    if (!Motor_Para_Update(&Model, Eso_Bw_Hz))
    {
        /* Invalid numeric model/ESO coefficients do not replace the active
         * values, but unrelated persistent parameters have been restored. */
        Current_Tuning_Update();
        Speed_Tuning_Update();
        Motor_Cal.Valid = 0U;
        return -EINVAL;
    }

    Encoder_Config_Changed();
    (void)Calibration_Load();
    return 0;
}

static int Record_Save(uint16_t Id, Parameter_Type_e Type, Parameter_Value_T Value)
{
    uint8_t Size = Parameter_Value_Size(Type);
    ssize_t Rc;

    if (Size == 0U)
    {
        return -EINVAL;
    }

    Rc = nvs_write(&Storage_NVS, Id, &Value, Size);
    if (Rc < 0)
    {
        return (int)Rc;
    }

    /* Zephyr NVS also returns zero when the value is already stored. */
    return ((Rc == 0) || (Rc == (ssize_t)Size)) ? 0 : -EIO;
}

int NVS_Storage_Save_All(void)
{
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    uint32_t Index = 0U;
    uint16_t Id;
    int Status;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    while (Parameter_Persistent_Next(&Index, &Id, &Type, &Value))
    {
        Status = Record_Save(Id, Type, Value);
        if (Status != 0)
        {
            return Status;
        }
    }

    return 0;
}

int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count)
{
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    int Status;

    if ((Count != 0U) && (Ids == NULL))
    {
        return -EINVAL;
    }

    /* Validate the entire list before writing any records. */
    for (uint16_t n = 0U; n < Count; n++)
    {
        if (!Parameter_Persistent_Read(Ids[n], &Type, &Value))
        {
            return -EINVAL;
        }
    }

    if (Count == 0U)
    {
        return 0;
    }

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    /* Preserve caller order, notably the final Valid=1 calibration record. */
    for (uint16_t n = 0U; n < Count; n++)
    {
        (void)Parameter_Persistent_Read(Ids[n], &Type, &Value);
        Status = Record_Save(Ids[n], Type, Value);
        if (Status != 0)
        {
            return Status;
        }
    }

    return 0;
}

int NVS_Storage_Smoke_Test(void)
{
    const uint32_t Pattern1 = 0x13579BDFU;
    const uint32_t Pattern2 = 0x2468ACE0U;
    uint32_t Readback = 0U;
    ssize_t Rc;
    int Status;

    NVS_Storage_Smoke_Result = -100;
    NVS_Storage_Smoke_Readback = 0U;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -1;
        return -1;
    }

    Status = nvs_clear(&Storage_NVS);
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -2;
        return -2;
    }
    Storage_Mounted = false;

    /* Zephyr nvs_clear() deliberately marks the filesystem unmounted. */
    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -3;
        return -3;
    }

    Rc = nvs_write(&Storage_NVS, STORAGE_SMOKE_ID, &Pattern1, sizeof(Pattern1));
    if (Rc != (ssize_t)sizeof(Pattern1))
    {
        NVS_Storage_Smoke_Result = -4;
        return -4;
    }

    Rc = nvs_read(&Storage_NVS, STORAGE_SMOKE_ID, &Readback, sizeof(Readback));
    if ((Rc != (ssize_t)sizeof(Readback)) || (Readback != Pattern1))
    {
        NVS_Storage_Smoke_Readback = Readback;
        NVS_Storage_Smoke_Result = -5;
        return -5;
    }

    Rc = nvs_write(&Storage_NVS, STORAGE_SMOKE_ID, &Pattern2, sizeof(Pattern2));
    if (Rc != (ssize_t)sizeof(Pattern2))
    {
        NVS_Storage_Smoke_Result = -6;
        return -6;
    }

    Readback = 0U;
    Rc = nvs_read(&Storage_NVS, STORAGE_SMOKE_ID, &Readback, sizeof(Readback));
    if ((Rc != (ssize_t)sizeof(Readback)) || (Readback != Pattern2))
    {
        NVS_Storage_Smoke_Readback = Readback;
        NVS_Storage_Smoke_Result = -7;
        return -7;
    }

    Status = nvs_clear(&Storage_NVS);
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -8;
        return -8;
    }
    Storage_Mounted = false;

    NVS_Storage_Smoke_Readback = Readback;
    NVS_Storage_Smoke_Result = 0;
    return 0;
}
