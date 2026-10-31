/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Enc_Home.h"

#include "Enc_Control.h"
#include "Encoder.h"
#include "Math.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motion_Type.h"
#include "main.h"

Enc_Home_T Enc_Home = { .Wm = 1.0f };

bool Enc_Home_Start(void)
{
    if ((Motor_Cal.Valid == 0U) || (Encoder.Ready == 0U) ||
        (Encoder.Fault != 0U) || (Encoder_Config.Protocol != ENC_PROTOCOL_ABZ) ||
        !__builtin_isfinite(Enc_Home.Wm) || (Enc_Home.Wm == 0.0f) ||
        (__builtin_fabsf(Enc_Home.Wm) > Motor_Wm_Limit_Effective_Get()) ||
        (Motion_Config.Wm_Acc <= 0.0f) || (Motion_Config.Wm_Dec <= 0.0f))
    {
        return false;
    }

    Enc_Home.Wm_Save = Motor_Cmd.Wm_Target;
    Enc_Home.Position_Save = Motor_Cmd.Position_Target;
    Enc_Home.Valid = 0U;
    Motor_Position_Get(&Motor_Cmd.Position_Target.Turn, &Motor_Cmd.Position_Target.Theta);
    /* Z 每圈一次，最多搜索两圈。 */
    Motor_Cmd.Position_Target.Turn += (Enc_Home.Wm > 0.0f) ? 2 : -2;
    Motor_Cmd.Wm_Target = __builtin_fabsf(Enc_Home.Wm);
    return true;
}

void Enc_Home_Trigger(float Theta_Native)
{
    float Theta;
    float Delta;
    uint32_t Primask;

    if ((Enc_Step != ENC_HOME_SEEK) || (Motor_State_Get() != RUN))
    {
        return;
    }

    Theta = Encoder_Angle_Correct(Theta_Native, Motor_Cal.Valid == 2U);
    if (Motor_Cal.Enc_Dir < 0)
    {
        Theta = Angle_Wrap(-Theta);
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    Enc_Home.Zero.Turn = Motor_Run.Turn;
    Enc_Home.Zero.Theta = Theta;
    /* 触发点与最近一次采样可能位于跨圈两侧。 */
    Delta = Theta - Motor_Run.Theta_m;
    if (Delta > PI_F)
    {
        Enc_Home.Zero.Turn--;
    }
    else if (Delta < -PI_F)
    {
        Enc_Home.Zero.Turn++;
    }
    Enc_Step = ENC_HOME_RETURN;
    __set_PRIMASK(Primask);
}

void Enc_Home_Run(void)
{
    Enc_Step_e Step = Enc_Step;
    uint32_t Primask;

    if ((Motor_State_Get() != RUN) || (Motor_Cal.Valid == 0U) ||
        (Encoder.Valid == 0U) || (Encoder.Fault != 0U))
    {
        Enc_Step = ENC_FAILED;
        return;
    }

    if (Step == ENC_HOME_RETURN)
    {
        /* 更新目标，保留轨迹的速度和加速度，允许越过后返回。 */
        Motor_Position_Internal_To_User(Enc_Home.Zero.Turn, Enc_Home.Zero.Theta,
                                        &Motor_Cmd.Position_Target.Turn,
                                        &Motor_Cmd.Position_Target.Theta);
    }

    if (!Motor_Position_Plan_Done())
    {
        return;
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    if (Enc_Step != Step)
    {
        __set_PRIMASK(Primask);
        return;
    }
    if (Step == ENC_HOME_RETURN)
    {
        /* 只切换用户零点，内部目标仍为 Z 触发位置，位置环可继续保持。 */
        Enc_Home.Valid = 1U;
        Motor_Cmd.Position_Target = (Motor_Position_T){ 0 };
        Enc_Step = ENC_DONE;
    }
    else
    {
        /* 到达搜索终点仍没有触发信号。 */
        Enc_Step = ENC_FAILED;
    }
    __set_PRIMASK(Primask);
}
