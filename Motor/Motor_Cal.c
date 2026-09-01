/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Cal.h"

#include "Encoder.h"
#include "Motor_Control.h"
#include "Motor_Type.h"

bool Motor_Cal_Set(int8_t Enc_Dir, float Theta_Off)
{
    int8_t Dir_Changed;

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

    Dir_Changed = (Enc_Dir != Motor_Cal.Enc_Dir);

    Motor_Cal.Enc_Dir = Enc_Dir;
    Motor_Cal.Theta_Off = Theta_Off;
    Motor_Cal.Valid = 1U;

    /*
     * Enc_Dir changes the encoder raw-angle mapping into the mechanical
     * coordinate system. Reset feedback history before accepting another
     * sample so Theta_Pre, Turn and Wm are never mixed across directions.
     * Encoder_DMA_Config() also restarts the Ready validation sequence.
     */
    if (Dir_Changed != 0)
    {
        Encoder_DMA_Config();
    }

    return true;
}

void Motor_Cal_Invalidate(void)
{
    Motor_Cal.Valid = 0U;
}
