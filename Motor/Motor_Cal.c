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

    /* 禁能时采样仍在运行，更新寻相结果需要中断保护。 */
    Primask = __get_PRIMASK();
    __disable_irq();
    Dir_Changed = (Enc_Dir != Motor_Cal.Enc_Dir);

    Motor_Cal.Enc_Dir = Enc_Dir;
    Motor_Cal.Theta_Off = Theta_Off;
    if (Motor_Cal.Valid != 2U)
    {
        Motor_Cal.Valid = 1U;
    }

    /* 方向变化只重建软件反馈历史，硬件采样配置和计数原点保持不变。 */
    if (Dir_Changed != 0)
    {
        Encoder_Feedback_Reset();
    }
    __set_PRIMASK(Primask);

    return true;
}
