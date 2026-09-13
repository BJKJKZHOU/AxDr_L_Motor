/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stddef.h>

#include "Math.h"

/*
 * Standard I/F actuator.
 *
 * Electrical speed follows the predetermined ramp to We_Target while the
 * torque-producing current reference stays fixed after its commanded slew.
 * Runtime current, voltage, observer, PLL and back-EMF signals never modify
 * the I/F trajectory. Measured current below the startup current is not a
 * failure condition; phase-current protection remains the only current
 * validity guard.
 */

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

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    IF_Para_T Para;

    if (IF == NULL)
    {
        return;
    }

    Para = IF->Para;
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

    IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
    IF->State.Iq = 0.0f;
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
    float Iq_Step;
    float We_Step;

    /* Standard I/F does not adapt its trajectory from runtime feedback. */
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
        !__builtin_isfinite(IF->State.Iq))
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

    Iq_Target = (IF->State.We_Target == 0.0f)
                    ? 0.0f
                    : Sign_F(IF->State.We_Target) * IF->Para.Iq_Min_A;
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
