/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include "Math.h"

/* 电角速度按给定斜坡变化，Iq 渐变到设定值；反馈不参与 I/F 轨迹调整。 */

static void IF_Fail(IF_T *IF)
{
    IF->State.Failed = true;
    IF->State.We = 0.0f;
    IF->State.Iq = 0.0f;
}

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    IF->State = (IF_State_T){ 0 };
    IF->State.Theta_e = Angle_Wrap(Theta_Start);
    IF->State.We = We_Start;
    IF->State.We_Target = We_Start;

    if ((IF->Para.Acc <= 0.0f) ||
        !__builtin_isfinite(IF->Para.Acc))
    {
        IF_Fail(IF);
    }
}

void IF_Target_Set(IF_T *IF, float We_Target)
{
    if (!__builtin_isfinite(We_Target) || IF->State.Failed)
    {
        return;
    }

    IF->State.We_Target = We_Target;
}

void IF_Run(IF_T *IF,
            float *Theta_e,
            float *Id_Ref,
            float *Iq_Ref,
            float Ts)
{
    float Iq_Target;
    float Iq_Step;
    float We_Step;

    if (!__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(IF->State.We) ||
        !__builtin_isfinite(IF->State.Iq))
    {
        IF_Fail(IF);
    }

    if (IF->State.Failed)
    {
        *Theta_e = __builtin_isfinite(IF->State.Theta_e) ? IF->State.Theta_e : 0.0f;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (IF->State.We != IF->State.We_Target)
    {
        We_Step = IF->Para.Acc * Ts;
        if (IF->State.We < IF->State.We_Target)
        {
            IF->State.We += We_Step;
            if (IF->State.We >= IF->State.We_Target)
            {
                IF->State.We = IF->State.We_Target;
            }
        }
        else
        {
            IF->State.We -= We_Step;
            if (IF->State.We <= IF->State.We_Target)
            {
                IF->State.We = IF->State.We_Target;
            }
        }
    }

    Iq_Target = (IF->State.We_Target == 0.0f)
                    ? 0.0f
                    : ((IF->State.We_Target < 0.0f) ? -1.0f : 1.0f) * IF->Para.Iq_Min_A;
    Iq_Step = IF->Para.Iq_Slew_A_S * Ts;

    if (IF->State.Iq < Iq_Target)
    {
        IF->State.Iq += Iq_Step;
        if (IF->State.Iq > Iq_Target)
        {
            IF->State.Iq = Iq_Target;
        }
    }
    else if (IF->State.Iq > Iq_Target)
    {
        IF->State.Iq -= Iq_Step;
        if (IF->State.Iq < Iq_Target)
        {
            IF->State.Iq = Iq_Target;
        }
    }

    *Theta_e = IF->State.Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = IF->State.Iq;
    IF->State.Theta_e = Angle_Wrap(IF->State.Theta_e + IF->State.We * Ts);
}
