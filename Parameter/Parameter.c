/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Parameter.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "Current_Loop.h"
#include "Encoder.h"
#include "Flux.h"
#include "Identification.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motion_Loop.h"
#include "Motion_Type.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "Protection.h"
#include "Sensorless.h"
#include "Servo_Phase.h"
#include "Signal_Injection.h"

#define PARAM_FLAG_HOST_WRITE       (1U << 0)
#define PARAM_FLAG_DISABLED_ONLY    (1U << 1)
#define PARAM_FLAG_NOT_RUNNING      (1U << 2)
#define PARAM_FLAG_PERSISTENT       (1U << 3)

typedef struct
{
    uint16_t Id;
    Parameter_Type_e Type;
    volatile void *Data;
    float Min;
    float Max;
    uint8_t Flags;

} Parameter_Entry_T;

static Motor_Position_T Parameter_Run_Position_Get(void)
{
    Motor_Position_T Position;

    Motor_Position_Get(&Position.Turn, &Position.Theta);
    return Position;
}

static const Parameter_Entry_T Parameter_Table[] =
{
#define PARAM_GENERATE_TABLE
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_TABLE
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

    if (Type == PARAM_POSITION)
    {
        if (!__builtin_isfinite(Value.Position.Theta) ||
            (Value.Position.Theta < 0.0f) ||
            (Value.Position.Theta >= TWO_PI_F))
        {
            return PARAM_ERR_VALUE;
        }
        return PARAM_OK;
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

    switch (Entry->Id)
    {
#define PARAM_GENERATE_VALIDATE
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_VALIDATE
        default:
            break;
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
        case PARAM_POSITION:
        {
            const volatile Motor_Position_T *Position =
                (const volatile Motor_Position_T *)Entry->Data;
            return (Position->Turn == Value.Position.Turn) &&
                   (Position->Theta == Value.Position.Theta);
        }
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
        case PARAM_POSITION:
        {
            volatile Motor_Position_T *Position = (volatile Motor_Position_T *)Entry->Data;
            Position->Turn = Value.Position.Turn;
            Position->Theta = Value.Position.Theta;
            break;
        }
        default:
            break;
    }
}

static void Parameter_Value_Read(const Parameter_Entry_T *Entry,
                                 Parameter_Value_T *Value)
{
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
        case PARAM_POSITION:
        {
            const volatile Motor_Position_T *Position =
                (const volatile Motor_Position_T *)Entry->Data;
            Value->Position.Turn = Position->Turn;
            Value->Position.Theta = Position->Theta;
            break;
        }
        default:
            break;
    }
}

static void Parameter_On_Change(uint16_t Id)
{
    switch (Id)
    {
#define PARAM_GENERATE_ON_CHANGE
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_ON_CHANGE
        default:
            break;
    }
}

static Parameter_Status_e Parameter_Apply(const Parameter_Entry_T *Entry,
                                          Parameter_Type_e Type,
                                          Parameter_Value_T Value)
{
    Parameter_Status_e Status;

    if (Entry->Data == NULL)
    {
        return PARAM_ERR_READ_ONLY;
    }

    Status = Parameter_Value_Check(Entry, Type, Value);
    if (Status != PARAM_OK)
    {
        return Status;
    }

    /* A gain/bandwidth write also selects its source, even if the value matches. */
    if (Parameter_Value_Equal(Entry, Value))
    {
        switch (Entry->Id)
        {
            case PARAM_CTRL_ID_KP:
            case PARAM_CTRL_ID_KI:
            case PARAM_CTRL_IQ_KP:
            case PARAM_CTRL_IQ_KI:
            case PARAM_CTRL_SPEED_KP:
            case PARAM_CTRL_SPEED_KI:
            case PARAM_CTRL_CURRENT_BW_HZ:
            case PARAM_CTRL_SPEED_BW_HZ:
                Parameter_On_Change(Entry->Id);
                break;
            default:
                break;
        }
        return PARAM_OK;
    }

    /* Cross-parameter ESO validity belongs to the complete candidate model.
     * Do not write a model field and then try to repair a failed observer update. */
    Motor_Para_T Model = Motor_Para;
    float Eso_Bw_Hz = Mechanical_ESO_Bw_Hz;

    switch (Entry->Id)
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
            Parameter_Value_Write(Entry, Value);
            Parameter_On_Change(Entry->Id);
            return PARAM_OK;
    }
    return Motor_Para_Update(&Model, Eso_Bw_Hz) ? PARAM_OK : PARAM_ERR_VALUE;
}

static Parameter_Status_e Parameter_Write_Common(uint16_t Id,
                                                 Parameter_Type_e Type,
                                                 Parameter_Value_T Value)
{
    const Parameter_Entry_T *Entry;
    Motor_State_e State;

    Entry = Parameter_Find(Id);
    if (Entry == NULL)
    {
        return PARAM_ERR_ID;
    }

    if ((Entry->Flags & PARAM_FLAG_HOST_WRITE) == 0U)
    {
        return PARAM_ERR_READ_ONLY;
    }

    State = Motor_State_Get();

    if (((Entry->Flags & PARAM_FLAG_DISABLED_ONLY) != 0U) &&
        (State != DISABLED))
    {
        return PARAM_ERR_STATE;
    }

    if (((Entry->Flags & PARAM_FLAG_NOT_RUNNING) != 0U) &&
        (State == RUN))
    {
        return PARAM_ERR_STATE;
    }

    return Parameter_Apply(Entry, Type, Value);
}

static Parameter_Status_e Parameter_Read_Indirect(uint16_t Id,
                                                  Parameter_Value_T *Value)
{
    switch (Id)
    {
#define PARAM_GENERATE_READ
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_READ
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

    Parameter_Value_Read(Entry, Value);
    return PARAM_OK;
}

Parameter_Status_e Parameter_Write(uint16_t Id,
                                   Parameter_Type_e Type,
                                   Parameter_Value_T Value)
{
    return Parameter_Write_Common(Id, Type, Value);
}

bool Parameter_Persistent_Read(uint16_t Id,
                               Parameter_Type_e *Type,
                               Parameter_Value_T *Value)
{
    const Parameter_Entry_T *Entry;

    if ((Type == NULL) || (Value == NULL))
    {
        return false;
    }

    Entry = Parameter_Find(Id);
    if ((Entry == NULL) ||
        ((Entry->Flags & PARAM_FLAG_PERSISTENT) == 0U) ||
        (Entry->Data == NULL))
    {
        return false;
    }

    *Type = Entry->Type;
    Parameter_Value_Read(Entry, Value);
    return true;
}

bool Parameter_Persistent_Next(uint32_t *Index,
                               uint16_t *Id,
                               Parameter_Type_e *Type,
                               Parameter_Value_T *Value)
{
    const Parameter_Entry_T *Entry;
    uint32_t Count;

    if ((Index == NULL) || (Id == NULL) || (Type == NULL) || (Value == NULL))
    {
        return false;
    }

    Count = (uint32_t)(sizeof(Parameter_Table) / sizeof(Parameter_Table[0]));
    while (*Index < Count)
    {
        Entry = &Parameter_Table[*Index];
        (*Index)++;

        if (((Entry->Flags & PARAM_FLAG_PERSISTENT) != 0U) &&
            (Entry->Data != NULL))
        {
            *Id = Entry->Id;
            *Type = Entry->Type;
            Parameter_Value_Read(Entry, Value);
            return true;
        }
    }

    return false;
}

Parameter_Status_e Parameter_Check(uint16_t Id, Parameter_Type_e Type,
                                   Parameter_Value_T Value)
{
    const Parameter_Entry_T *Entry = Parameter_Find(Id);

    if (Entry == NULL)
    {
        return PARAM_ERR_ID;
    }
    return Parameter_Value_Check(Entry, Type, Value);
}

Parameter_Status_e Parameter_Restore(uint16_t Id,
                                     Parameter_Type_e Type,
                                     Parameter_Value_T Value)
{
    const Parameter_Entry_T *Entry;
    Parameter_Status_e Status;

    Entry = Parameter_Find(Id);
    if (Entry == NULL)
    {
        return PARAM_ERR_ID;
    }

    if (((Entry->Flags & PARAM_FLAG_PERSISTENT) == 0U) ||
        (Entry->Data == NULL))
    {
        return PARAM_ERR_READ_ONLY;
    }

    Status = Parameter_Value_Check(Entry, Type, Value);
    if (Status != PARAM_OK)
    {
        return Status;
    }

    Parameter_Value_Write(Entry, Value);
    return PARAM_OK;
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
        case PARAM_POSITION:
            return (uint8_t)sizeof(Motor_Position_T);
        default:
            return 0U;
    }
}
