/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stddef.h>

#include "Math.h"

/*
 * I/F is only an open-loop actuator. It ramps electrical speed and current;
 * rotor-motion qualification belongs to the observer/identification layer,
 * where back-EMF is observable enough to make that decision reliably.
 *
 * Startup current follows electrical-speed progress continuously. The normal
 * working current stays below the user protection limit so the closed current
 * loop retains margin for ripple and transient tracking error.
 */
#define IF_IQ_WORK_RATIO 0.80f

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Sign_F(float X)
{
    return (X < 0.0f) ? -1.0f : 1.0f;
}

static float Clamp_F(float X, float Min, float Max)
{
    if (X < Min)
    {
        return Min;
    }
    if (X > Max)
    {
        return Max;
    }
    return X;
}

static void IF_Fail(IF_T *IF)
{
    IF->State.Mode = IF_FAILED;
    IF->State.We = 0.0f;
    IF->State.Iq = 0.0f;
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

static float Startup_Iq_Abs(const IF_T *IF)
{
    float Progress;

    if (IF->Para.We_Base <= 0.0f)
    {
        return IF->Para.Iq_Min_A;
    }

    Progress = Abs_F(IF->State.We) / IF->Para.We_Base;
    Progress = Clamp_F(Progress, 0.0f, 1.0f);

    return IF->Para.Iq_Min_A +
           (IF->State.Iq_Work_A - IF->Para.Iq_Min_A) * Progress;
}

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    IF_Para_T Para;
    float Iq_Work_Pre;
    float Iq_Work_Max;

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

    if ((IF->Para.Iq_Min_A <= 0.0f) ||
        (IF->Para.Iq_Max_A < IF->Para.Iq_Min_A) ||
        (IF->Para.We_Base <= 0.0f) ||
        (IF->Para.Acc <= 0.0f) ||
        (IF->Para.Iq_Slew_A_S <= 0.0f) ||
        !__builtin_isfinite(IF->Para.Iq_Min_A) ||
        !__builtin_isfinite(IF->Para.Iq_Max_A) ||
        !__builtin_isfinite(IF->Para.We_Base) ||
        !__builtin_isfinite(IF->Para.Acc) ||
        !__builtin_isfinite(IF->Para.Iq_Slew_A_S) ||
        !__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(We_Start))
    {
        IF_Fail(IF);
        return;
    }

    Iq_Work_Max = IF_IQ_WORK_RATIO * IF->Para.Iq_Max_A;
    Iq_Work_Max = Clamp_F(Iq_Work_Max,
                          IF->Para.Iq_Min_A,
                          IF->Para.Iq_Max_A);

    if (Abs_F(We_Start) <= 0.0f)
    {
        IF->State.Iq_Work_A = Iq_Work_Max;
    }
    else
    {
        IF->State.Iq_Work_A = Clamp_F(Iq_Work_Pre,
                                      IF->Para.Iq_Min_A,
                                      Iq_Work_Max);
    }

    IF->State.Mode = IF_HOLD;
}

void IF_Target_Set(IF_T *IF, float We_Target)
{
    if ((IF == NULL) || !__builtin_isfinite(We_Target) ||
        (IF->State.Mode == IF_FAILED))
    {
        return;
    }

    IF->State.We_Target = We_Target;
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
    float Iq_Abs;
    float We_Step;

    (void)Id_A;
    (void)Iq_A;
    (void)Ud_V;
    (void)Uq_V;

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

    if (IF->State.We_Target == 0.0f)
    {
        Iq_Target = 0.0f;
    }
    else
    {
        Iq_Abs = Startup_Iq_Abs(IF);
        Iq_Target = Sign_F(IF->State.We_Target) * Iq_Abs;
    }
    Iq_Slew_Run(IF, Iq_Target, Ts);

    if (!__builtin_isfinite(IF->State.Theta_e) ||
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
