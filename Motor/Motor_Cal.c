/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Cal.h"

#include "Encoder.h"
#include "Motor_Control.h"
#include "Motor_Type.h"
#include "stm32g4xx_hal.h"

bool Motor_Cal_Set(int8_t Enc_Dir, float Theta_Off)
{
    int8_t Dir_Changed;
    uint32_t Primask;

    if (Motor_State_Get() != DISABLED)
    {
        return false;
    }

    if ((Enc_Dir != 1) && (Enc_Dir != -1))
    {
        return false;
    }

    if (!__builtin_isfinite(Theta_Off))
    {
        return false;
    }

    /* Feedback and the ESO keep running while the power stage is disabled. */
    Primask = __get_PRIMASK();
    __disable_irq();
    Dir_Changed = (Enc_Dir != Motor_Cal.Enc_Dir);

    Motor_Cal.Enc_Dir = Enc_Dir;
    Motor_Cal.Theta_Off = Theta_Off;
    Motor_Cal.Valid = 1U;

    /*
     * Enc_Dir changes the encoder raw-angle mapping into the mechanical
     * coordinate system. Reset feedback history before accepting another
     * sample so turn counting and the ESO never mix coordinate directions.
     * ABZ must retain CNT: Theta_Off was measured in that counter coordinate.
     */
    if (Dir_Changed != 0)
    {
        if (Encoder_Config.Protocol == ENC_PROTOCOL_ABZ)
        {
            Encoder_Feedback_Reset();
        }
        else
        {
            Encoder_Config_Apply();
        }
    }
    __set_PRIMASK(Primask);

    return true;
}
