/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Parameter.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "Encoder.h"
#include "Flux.h"
#include "Motion_Type.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "Protection.h"
#include "Sensorless.h"
#include "Servo_Phase.h"

#define PARAM_FLAG_HOST_WRITE    (1U << 0)
#define PARAM_FLAG_DISABLED_ONLY (1U << 1)

#define PARAM_CHANGE_NONE        0U
#define PARAM_CHANGE_MOTOR_RL    (1U << 0)
#define PARAM_CHANGE_MOTOR_FLUX  (1U << 1)
#define PARAM_CHANGE_MOTOR_JB    (1U << 2)
#define PARAM_CHANGE_MOTOR_PP    (1U << 3)
#define PARAM_CHANGE_ENCODER     (1U << 4)

typedef struct
{
    uint16_t Id;
    Parameter_Type_e Type;
    volatile void *Data;
    float Min;
    float Max;
    uint8_t Flags;
    uint32_t Change;

} Parameter_Entry_T;

static const Parameter_Entry_T Parameter_Table[] =
{
#include "Parameter_Table.generated.inc"
};

static const Parameter_Entry_T *Parameter_Find(uint16_t Id)
{
    uint32_t Index;

    for (Index = 0U; Index < (uint32_t)(sizeof(Parameter_Table) / sizeof(Parameter_Table[0])); Index++)
    {
        if (Parameter_Table[Index].Id == Id)
        {
            return &Parameter_Table[Index];
        }
    }

    return NULL;
}

static Parameter_Status_e Parameter_Value_Check(const Parameter_Entry_T *Entry,
                                                Parameter_Type_e Type,
                                                Parameter_Value_T Value)
{
    float Number;

    if (Type != Entry->Type)
    {
        return PARAM_ERR_TYPE;
    }

    switch (Type)
    {
        case PARAM_U8:
            Number = (float)Value.U8;
            break;

        case PARAM_I8:
            Number = (float)Value.I8;
            break;

        case PARAM_FLOAT:
            if (!__builtin_isfinite(Value.F32))
            {
                return PARAM_ERR_VALUE;
            }
            Number = Value.F32;
            break;

        case PARAM_I32:
            Number = (float)Value.I32;
            break;

        case PARAM_U32:
            Number = (float)Value.U32;
            break;

        default:
            return PARAM_ERR_TYPE;
    }

    if ((Number < Entry->Min) || (Number > Entry->Max))
    {
        return PARAM_ERR_VALUE;
    }

    if (((Entry->Id == PARAM_MOTOR_RS) ||
         (Entry->Id == PARAM_MOTOR_LD) ||
         (Entry->Id == PARAM_MOTOR_LQ) ||
         (Entry->Id == PARAM_MOTOR_FLUX) ||
         (Entry->Id == PARAM_MOTOR_J)) &&
        (Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }

    if ((Entry->Id == PARAM_MOTOR_DIR) &&
        (Value.I8 != -1) && (Value.I8 != 1))
    {
        return PARAM_ERR_VALUE;
    }

    if ((Entry->Id == PARAM_LIMIT_I_MAX) &&
        ((Number <= 0.0f) || (Number > Motor_Lim.I_Max)))
    {
        return PARAM_ERR_VALUE;
    }

    if ((Entry->Id == PARAM_LIMIT_WM_MAX) &&
        ((Number <= 0.0f) || (Number > Motor_Lim.Wm_Max)))
    {
        return PARAM_ERR_VALUE;
    }

    if ((Entry->Id == PARAM_PHASE_I_SEARCH) && (Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }

    if ((Entry->Id == PARAM_MOTION_WM_MAX) &&
        ((Number > User_Lim.Wm_Max) || (Number > Motor_Lim.Wm_Max)))
    {
        return PARAM_ERR_VALUE;
    }

    if (((Entry->Id == PARAM_MOTION_WM_ACC) ||
         (Entry->Id == PARAM_MOTION_WM_DEC)) &&
        (Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }

    if (Entry->Id == PARAM_MOTOR_MODE)
    {
        switch ((Motor_Mode_e)Value.U8)
        {
            case TORQUE:
            case SPEED:
            case POSITION:
            case OPEN_LOOP:
            case IDENT:
            case SENSORLESS_SPEED:
            case PHASE_SEARCH:
                break;

            default:
                return PARAM_ERR_VALUE;
        }
    }

    return PARAM_OK;
}

static bool Parameter_Value_Equal(const Parameter_Entry_T *Entry,
                                  Parameter_Value_T Value)
{
    switch (Entry->Type)
    {
        case PARAM_U8:
            return *(const volatile uint8_t *)Entry->Data == Value.U8;
        case PARAM_I8:
            return *(const volatile int8_t *)Entry->Data == Value.I8;
        case PARAM_FLOAT:
            return *(const volatile float *)Entry->Data == Value.F32;
        case PARAM_I32:
            return *(const volatile int32_t *)Entry->Data == Value.I32;
        case PARAM_U32:
            return *(const volatile uint32_t *)Entry->Data == Value.U32;
        default:
            return false;
    }
}

static void Parameter_Value_Write(const Parameter_Entry_T *Entry,
                                  Parameter_Value_T Value)
{
    switch (Entry->Type)
    {
        case PARAM_U8:
            *(volatile uint8_t *)Entry->Data = Value.U8;
            break;
        case PARAM_I8:
            *(volatile int8_t *)Entry->Data = Value.I8;
            break;
        case PARAM_FLOAT:
            *(volatile float *)Entry->Data = Value.F32;
            break;
        case PARAM_I32:
            *(volatile int32_t *)Entry->Data = Value.I32;
            break;
        case PARAM_U32:
            *(volatile uint32_t *)Entry->Data = Value.U32;
            break;
        default:
            break;
    }
}

static void Parameter_Change_Apply(uint32_t Change)
{
    if ((Change & PARAM_CHANGE_MOTOR_RL) != 0U)
    {
        Motor_Para_Changed(MOTOR_PARA_RL);
    }
    if ((Change & PARAM_CHANGE_MOTOR_FLUX) != 0U)
    {
        Motor_Para_Changed(MOTOR_PARA_FLUX);
    }
    if ((Change & PARAM_CHANGE_MOTOR_JB) != 0U)
    {
        Motor_Para_Changed(MOTOR_PARA_JB);
    }
    if ((Change & PARAM_CHANGE_MOTOR_PP) != 0U)
    {
        Motor_Para_Changed(MOTOR_PARA_PP);
    }
    if ((Change & PARAM_CHANGE_ENCODER) != 0U)
    {
        Encoder_Config_Changed();
    }
}

static Parameter_Status_e Parameter_Write_Common(uint16_t Id,
                                                 Parameter_Type_e Type,
                                                 Parameter_Value_T Value,
                                                 bool Host)
{
    const Parameter_Entry_T *Entry;
    Parameter_Status_e Status;

    Entry = Parameter_Find(Id);
    if (Entry == NULL)
    {
        return PARAM_ERR_ID;
    }

    if (Host && ((Entry->Flags & PARAM_FLAG_HOST_WRITE) == 0U))
    {
        return PARAM_ERR_READ_ONLY;
    }

    if (Entry->Data == NULL)
    {
        return PARAM_ERR_READ_ONLY;
    }

    if (((Entry->Flags & PARAM_FLAG_DISABLED_ONLY) != 0U) &&
        (Motor_State_Get() != DISABLED))
    {
        return PARAM_ERR_STATE;
    }

    Status = Parameter_Value_Check(Entry, Type, Value);
    if (Status != PARAM_OK)
    {
        return Status;
    }

    if (Parameter_Value_Equal(Entry, Value))
    {
        return PARAM_OK;
    }

    Parameter_Value_Write(Entry, Value);
    Parameter_Change_Apply(Entry->Change);

    return PARAM_OK;
}

static Parameter_Status_e Parameter_Read_Indirect(uint16_t Id,
                                                  Parameter_Value_T *Value)
{
    switch (Id)
    {
#include "Parameter_Read.generated.inc"
        default:
            return PARAM_ERR_ID;
    }
}

Parameter_Status_e Parameter_Read(uint16_t Id,
                                  Parameter_Type_e *Type,
                                  Parameter_Value_T *Value)
{
    const Parameter_Entry_T *Entry;

    if ((Type == NULL) || (Value == NULL))
    {
        return PARAM_ERR_VALUE;
    }

    Entry = Parameter_Find(Id);
    if (Entry == NULL)
    {
        return PARAM_ERR_ID;
    }

    *Type = Entry->Type;

    if (Entry->Data == NULL)
    {
        return Parameter_Read_Indirect(Id, Value);
    }

    switch (Entry->Type)
    {
        case PARAM_U8:
            Value->U8 = *(const volatile uint8_t *)Entry->Data;
            break;
        case PARAM_I8:
            Value->I8 = *(const volatile int8_t *)Entry->Data;
            break;
        case PARAM_FLOAT:
            Value->F32 = *(const volatile float *)Entry->Data;
            break;
        case PARAM_I32:
            Value->I32 = *(const volatile int32_t *)Entry->Data;
            break;
        case PARAM_U32:
            Value->U32 = *(const volatile uint32_t *)Entry->Data;
            break;
        default:
            return PARAM_ERR_TYPE;
    }

    return PARAM_OK;
}

Parameter_Status_e Parameter_Write(uint16_t Id,
                                   Parameter_Type_e Type,
                                   Parameter_Value_T Value)
{
    return Parameter_Write_Common(Id, Type, Value, true);
}

Parameter_Status_e Parameter_Write_Internal(uint16_t Id,
                                            Parameter_Type_e Type,
                                            Parameter_Value_T Value)
{
    return Parameter_Write_Common(Id, Type, Value, false);
}

uint8_t Parameter_Value_Size(Parameter_Type_e Type)
{
    switch (Type)
    {
        case PARAM_U8:
        case PARAM_I8:
            return 1U;
        case PARAM_FLOAT:
        case PARAM_I32:
        case PARAM_U32:
            return 4U;
        default:
            return 0U;
    }
}
