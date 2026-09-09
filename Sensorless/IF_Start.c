/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stddef.h>

#include "Math.h"

/* Adaptive startup keeps the fast loop simple:
 * - ramp electrical speed continuously instead of stepping between test points;
 * - sample the q-axis back-EMF proxy in low/high portions of one complete ramp;
 * - decide only after the full ramp reaches the check speed;
 * - on failure, ramp back to zero before increasing current and retrying.
 */
#define IF_KICK_CHECK_WE_RATIO      0.25f
#define IF_KICK_I_STEP_RATIO        0.10f
#define IF_KICK_LOW_BEGIN_RATIO     0.40f
#define IF_KICK_LOW_END_RATIO       0.60f
#define IF_KICK_HIGH_BEGIN_RATIO    0.80f
#define IF_KICK_BEMF_GROWTH_RATIO   0.25f

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Sign_F(float X)
{
    return (X < 0.0f) ? -1.0f : 1.0f;
}

static void IF_Fail(IF_T *IF)
{
    IF->State.Mode = IF_FAILED;
    IF->State.We = 0.0f;
    IF->State.Iq = 0.0f;
}

static void Kick_Sample_Reset(IF_T *IF)
{
    IF->State.Kick_Bemf_Low_Sum = 0.0f;
    IF->State.Kick_Bemf_High_Sum = 0.0f;
    IF->State.Kick_Bemf_Low_Cnt = 0U;
    IF->State.Kick_Bemf_High_Cnt = 0U;
}

static void Iq_Slew_Run(IF_T *IF, float Target, float Ts)
{
    float Step;

    Step = IF->Para.Iq_Slew_A_S * Ts;
    if (IF->State.Iq < Target)
    {
        IF->State.Iq += Step;
        if (IF->State.Iq > Target)
        {
            IF->State.Iq = Target;
        }
    }
    else if (IF->State.Iq > Target)
    {
        IF->State.Iq -= Step;
        if (IF->State.Iq < Target)
        {
            IF->State.Iq = Target;
        }
    }
}

static bool Kick_Check_We_Build(IF_T *IF)
{
    float Target_Abs;
    float Check_Abs;

    Target_Abs = Abs_F(IF->State.We_Target);
    if (Target_Abs <= 0.0f)
    {
        return false;
    }

    Check_Abs = IF_KICK_CHECK_WE_RATIO * IF->Para.We_Base;
    if (Check_Abs > Target_Abs)
    {
        Check_Abs = Target_Abs;
    }

    if (Check_Abs <= 0.0f)
    {
        return false;
    }

    IF->State.Kick_Check_We = Sign_F(IF->State.We_Target) * Check_Abs;
    return true;
}

static bool Kick_Current_Increase(IF_T *IF)
{
    float I_Span;
    float I_Step;

    I_Span = IF->Para.Iq_Max_A - IF->Para.Iq_Min_A;
    if ((I_Span <= 0.0f) || (IF->State.Kick_I_Target_A >= IF->Para.Iq_Max_A))
    {
        return false;
    }

    I_Step = IF_KICK_I_STEP_RATIO * I_Span;
    if (I_Step <= 0.0f)
    {
        return false;
    }

    IF->State.Kick_I_Target_A += I_Step;
    if (IF->State.Kick_I_Target_A > IF->Para.Iq_Max_A)
    {
        IF->State.Kick_I_Target_A = IF->Para.Iq_Max_A;
    }

    IF->State.Iq_Work_A = IF->State.Kick_I_Target_A;
    return true;
}

static bool Kick_Bemf_Sample(IF_T *IF, float Iq_A, float Uq_V)
{
    float Dir;
    float Bemf;
    float We_Abs;
    float Check_Abs;

    if (!__builtin_isfinite(Iq_A) || !__builtin_isfinite(Uq_V))
    {
        return false;
    }

    Dir = Sign_F(IF->State.We_Target);
    Bemf = Dir * (Uq_V - IF->Para.Rs_Ohm * Iq_A);
    if (!__builtin_isfinite(Bemf))
    {
        return false;
    }

    We_Abs = Abs_F(IF->State.We);
    Check_Abs = Abs_F(IF->State.Kick_Check_We);

    if ((We_Abs >= IF_KICK_LOW_BEGIN_RATIO * Check_Abs) &&
        (We_Abs <= IF_KICK_LOW_END_RATIO * Check_Abs))
    {
        IF->State.Kick_Bemf_Low_Sum += Bemf;
        IF->State.Kick_Bemf_Low_Cnt++;
    }
    else if (We_Abs >= IF_KICK_HIGH_BEGIN_RATIO * Check_Abs)
    {
        IF->State.Kick_Bemf_High_Sum += Bemf;
        IF->State.Kick_Bemf_High_Cnt++;
    }

    return true;
}

static bool Kick_Motion_Confirmed(IF_T *IF)
{
    float Bemf_Low;
    float Bemf_High;
    float Growth;

    if ((IF->State.Kick_Bemf_Low_Cnt == 0U) || (IF->State.Kick_Bemf_High_Cnt == 0U))
    {
        return false;
    }

    Bemf_Low = IF->State.Kick_Bemf_Low_Sum / (float)IF->State.Kick_Bemf_Low_Cnt;
    Bemf_High = IF->State.Kick_Bemf_High_Sum / (float)IF->State.Kick_Bemf_High_Cnt;
    if (!__builtin_isfinite(Bemf_Low) || !__builtin_isfinite(Bemf_High) ||
        (Bemf_High <= 0.0f))
    {
        return false;
    }

    Growth = Bemf_High - Bemf_Low;
    return (Growth > 0.0f) &&
           (Growth >= IF_KICK_BEMF_GROWTH_RATIO * Abs_F(Bemf_High));
}

static void Kick_Run(IF_T *IF, float Iq_A, float Uq_V, float Ts)
{
    float Dir;
    float We_Step;
    float Check_Abs;

    if (Abs_F(IF->State.We_Target) <= 0.0f)
    {
        IF->State.We = 0.0f;
        Iq_Slew_Run(IF, 0.0f, Ts);
        return;
    }

    Dir = Sign_F(IF->State.We_Target);
    Iq_Slew_Run(IF, Dir * IF->State.Kick_I_Target_A, Ts);
    We_Step = IF->Para.Acc * Ts;

    switch (IF->State.Kick_Mode)
    {
        case IF_KICK_CURRENT:
            IF->State.We = 0.0f;
            if (Abs_F(IF->State.Iq) < IF->State.Kick_I_Target_A)
            {
                return;
            }
            if ((IF->State.Kick_Check_We == 0.0f) && !Kick_Check_We_Build(IF))
            {
                IF_Fail(IF);
                return;
            }
            Kick_Sample_Reset(IF);
            IF->State.Kick_Mode = IF_KICK_RAMP_UP;
            return;

        case IF_KICK_RAMP_UP:
            IF->State.We += Dir * We_Step;
            Check_Abs = Abs_F(IF->State.Kick_Check_We);
            if (Abs_F(IF->State.We) > Check_Abs)
            {
                IF->State.We = IF->State.Kick_Check_We;
            }

            if (!Kick_Bemf_Sample(IF, Iq_A, Uq_V))
            {
                IF_Fail(IF);
                return;
            }

            if (Abs_F(IF->State.We) < Check_Abs)
            {
                return;
            }

            if (Kick_Motion_Confirmed(IF))
            {
                IF->State.Iq_Work_A = IF->State.Kick_I_Target_A;
                IF->State.Mode = (IF->State.We == IF->State.We_Target) ? IF_HOLD : IF_RAMP;
                return;
            }

            IF->State.Kick_Mode = IF_KICK_RAMP_DOWN;
            return;

        case IF_KICK_RAMP_DOWN:
            if (IF->State.We > We_Step)
            {
                IF->State.We -= We_Step;
                return;
            }
            if (IF->State.We < -We_Step)
            {
                IF->State.We += We_Step;
                return;
            }

            IF->State.We = 0.0f;
            if (!Kick_Current_Increase(IF))
            {
                IF_Fail(IF);
                return;
            }
            Kick_Sample_Reset(IF);
            IF->State.Kick_Mode = IF_KICK_CURRENT;
            return;

        default:
            IF_Fail(IF);
            return;
    }
}

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    IF_Para_T Para;
    float Iq_Work_Pre;

    if (IF == NULL)
    {
        return;
    }

    Para = IF->Para;
    Iq_Work_Pre = IF->State.Iq_Work_A;
    IF->State = (IF_State_T){ 0 };
    IF->Para = Para;
    IF->State.Theta_e = Angle_Wrap(Theta_Start);
    IF->State.We = We_Start;
    IF->State.We_Target = We_Start;
    IF->State.Kick_Mode = IF_KICK_CURRENT;
    IF->State.Kick_I_Target_A = IF->Para.Iq_Min_A;

    if ((IF->Para.Iq_Min_A <= 0.0f) ||
        (IF->Para.Iq_Max_A < IF->Para.Iq_Min_A) ||
        (IF->Para.We_Base <= 0.0f) ||
        (IF->Para.Acc <= 0.0f) ||
        (IF->Para.Iq_Slew_A_S <= 0.0f) ||
        (IF->Para.Rs_Ohm < 0.0f) ||
        !__builtin_isfinite(IF->Para.Iq_Min_A) ||
        !__builtin_isfinite(IF->Para.Iq_Max_A) ||
        !__builtin_isfinite(IF->Para.We_Base) ||
        !__builtin_isfinite(IF->Para.Acc) ||
        !__builtin_isfinite(IF->Para.Iq_Slew_A_S) ||
        !__builtin_isfinite(IF->Para.Rs_Ohm) ||
        !__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(We_Start))
    {
        IF_Fail(IF);
        return;
    }

    if (Abs_F(We_Start) <= 0.0f)
    {
        IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
        IF->State.Mode = IF_KICK;
    }
    else
    {
        IF->State.Iq_Work_A = Iq_Work_Pre;
        if (IF->State.Iq_Work_A < IF->Para.Iq_Min_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
        }
        else if (IF->State.Iq_Work_A > IF->Para.Iq_Max_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Max_A;
        }
        IF->State.Mode = IF_RAMP;
    }
}

void IF_Target_Set(IF_T *IF, float We_Target)
{
    if ((IF == NULL) || !__builtin_isfinite(We_Target) ||
        (IF->State.We_Target == We_Target))
    {
        return;
    }

    IF->State.We_Target = We_Target;
    if ((IF->State.Mode == IF_FAILED) || (IF->State.Mode == IF_KICK))
    {
        return;
    }
    IF->State.Mode = (IF->State.We == We_Target) ? IF_HOLD : IF_RAMP;
}

void IF_Run(IF_T *IF,
            float Id_A,
            float Iq_A,
            float Ud_V,
            float Uq_V,
            float *Theta_e,
            float *Id_Ref,
            float *Iq_Ref,
            float Ts)
{
    float Iq_Target;
    float We_Step;

    (void)Id_A;
    (void)Ud_V;

    if ((IF == NULL) || (Theta_e == NULL) || (Id_Ref == NULL) || (Iq_Ref == NULL) ||
        (Ts <= 0.0f) || !__builtin_isfinite(Ts))
    {
        return;
    }

    if (!__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(IF->State.We) ||
        !__builtin_isfinite(IF->State.We_Target) ||
        !__builtin_isfinite(IF->State.Iq) ||
        !__builtin_isfinite(IF->State.Iq_Work_A))
    {
        IF_Fail(IF);
    }

    if (IF->State.Mode == IF_FAILED)
    {
        *Theta_e = __builtin_isfinite(IF->State.Theta_e) ? IF->State.Theta_e : 0.0f;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (IF->State.Mode == IF_KICK)
    {
        Kick_Run(IF, Iq_A, Uq_V, Ts);
    }
    else
    {
        if (IF->State.Mode == IF_RAMP)
        {
            We_Step = IF->Para.Acc * Ts;
            if (IF->State.We < IF->State.We_Target)
            {
                IF->State.We += We_Step;
                if (IF->State.We >= IF->State.We_Target)
                {
                    IF->State.We = IF->State.We_Target;
                    IF->State.Mode = IF_HOLD;
                }
            }
            else
            {
                IF->State.We -= We_Step;
                if (IF->State.We <= IF->State.We_Target)
                {
                    IF->State.We = IF->State.We_Target;
                    IF->State.Mode = IF_HOLD;
                }
            }
        }

        if (IF->State.We > 0.0f)
        {
            Iq_Target = IF->State.Iq_Work_A;
        }
        else if (IF->State.We < 0.0f)
        {
            Iq_Target = -IF->State.Iq_Work_A;
        }
        else
        {
            Iq_Target = 0.0f;
        }
        Iq_Slew_Run(IF, Iq_Target, Ts);
    }

    if ((IF->State.Mode == IF_FAILED) ||
        !__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(IF->State.We) ||
        !__builtin_isfinite(IF->State.Iq))
    {
        IF_Fail(IF);
        *Theta_e = 0.0f;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    *Theta_e = IF->State.Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = IF->State.Iq;
    IF->State.Theta_e = Angle_Wrap(IF->State.Theta_e + IF->State.We * Ts);
}
