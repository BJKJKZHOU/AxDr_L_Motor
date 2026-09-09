/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stddef.h>

#include "Math.h"

/* Kick policy is expressed with motor-relative or dimensionless quantities.
 * The only absolute current bounds are Para.Iq_Min_A and Para.Iq_Max_A. */
#define IF_KICK_WE_RATIO            0.125f
#define IF_KICK_I_STEP_RATIO        0.10f
#define IF_KICK_SLIP_LOCK_RATIO     0.20f
#define IF_KICK_LOCK_STEP_COUNT     3U
#define IF_KICK_PHASE_TRAVEL_RAD    TWO_PI_F
#define IF_KICK_EMF_TAU_S           0.001f
#define IF_KICK_EMF_MIN_RATIO       0.05f
#define IF_KICK_VALID_SAMPLE_RATIO  0.50f
#define IF_KICK_PHASE_COHERENCE_MIN 0.50f

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Sign_F(float X)
{
    return (X < 0.0f) ? -1.0f : 1.0f;
}

static float Angle_Diff(float A, float B)
{
    float Diff;

    Diff = A - B;
    while (Diff > PI_F)
    {
        Diff -= TWO_PI_F;
    }
    while (Diff < -PI_F)
    {
        Diff += TWO_PI_F;
    }
    return Diff;
}

static void Kick_Observe_Reset(IF_T *IF)
{
    IF->State.Kick_Current_Valid = false;
    IF->State.Kick_Phase_Valid = false;
    IF->State.Kick_Ed_F = 0.0f;
    IF->State.Kick_Eq_F = 0.0f;
    IF->State.Kick_Phase_Drift = 0.0f;
    IF->State.Kick_IF_Phase_Travel = 0.0f;
    IF->State.Kick_Phase_X_Sum = 0.0f;
    IF->State.Kick_Phase_Y_Sum = 0.0f;
    IF->State.Kick_Observe_Cnt = 0U;
    IF->State.Kick_Valid_Cnt = 0U;
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

static bool Kick_Observe_Run(IF_T *IF,
                             float Id_A,
                             float Iq_A,
                             float Ud_V,
                             float Uq_V,
                             float Ts,
                             bool *Locked)
{
    float Alpha;
    float dId;
    float dIq;
    float Ed;
    float Eq;
    float E_Mag;
    float I_Mag;
    float U_Mag;
    float Model_Scale;
    float Phase;
    float Phase_Diff;
    float Slip_Ratio;
    float Valid_Ratio;
    float Coherence;

    if ((Locked == NULL) || (Abs_F(IF->State.We) <= 0.0f) || (Ts <= 0.0f))
    {
        return false;
    }

    *Locked = false;
    if (!IF->State.Kick_Current_Valid)
    {
        IF->State.Kick_Id_Last = Id_A;
        IF->State.Kick_Iq_Last = Iq_A;
        IF->State.Kick_Current_Valid = true;
        return false;
    }

    dId = (Id_A - IF->State.Kick_Id_Last) / Ts;
    dIq = (Iq_A - IF->State.Kick_Iq_Last) / Ts;
    IF->State.Kick_Id_Last = Id_A;
    IF->State.Kick_Iq_Last = Iq_A;

    /* dq voltage-model residual in the I/F frame. It removes stator
     * resistance, current dynamics and frame-rotation terms, leaving the PM
     * back-EMF vector without assuming that rotor speed already equals We. */
    Ed = Ud_V - IF->Para.Rs_Ohm * Id_A - IF->Para.Ld_H * dId +
         IF->State.We * IF->Para.Lq_H * Iq_A;
    Eq = Uq_V - IF->Para.Rs_Ohm * Iq_A - IF->Para.Lq_H * dIq -
         IF->State.We * IF->Para.Ld_H * Id_A;

    Alpha = Ts / (IF_KICK_EMF_TAU_S + Ts);
    IF->State.Kick_Ed_F += Alpha * (Ed - IF->State.Kick_Ed_F);
    IF->State.Kick_Eq_F += Alpha * (Eq - IF->State.Kick_Eq_F);
    IF->State.Kick_IF_Phase_Travel += Abs_F(IF->State.We) * Ts;
    IF->State.Kick_Observe_Cnt++;

    if (__builtin_isfinite(IF->State.Kick_Ed_F) && __builtin_isfinite(IF->State.Kick_Eq_F))
    {
        E_Mag = __builtin_sqrtf(IF->State.Kick_Ed_F * IF->State.Kick_Ed_F +
                                IF->State.Kick_Eq_F * IF->State.Kick_Eq_F);
        I_Mag = __builtin_sqrtf(Id_A * Id_A + Iq_A * Iq_A);
        U_Mag = __builtin_sqrtf(Ud_V * Ud_V + Uq_V * Uq_V);
        Model_Scale = U_Mag + IF->Para.Rs_Ohm * I_Mag +
                      Abs_F(IF->State.We) *
                          (IF->Para.Ld_H * Abs_F(Id_A) + IF->Para.Lq_H * Abs_F(Iq_A));

        if ((Model_Scale > 0.0f) && (E_Mag >= IF_KICK_EMF_MIN_RATIO * Model_Scale))
        {
            Phase = __builtin_atan2f(IF->State.Kick_Eq_F, IF->State.Kick_Ed_F);
            if (!IF->State.Kick_Phase_Valid)
            {
                IF->State.Kick_Phase_Last = Phase;
                IF->State.Kick_Phase_Valid = true;
            }
            else
            {
                Phase_Diff = Angle_Diff(Phase, IF->State.Kick_Phase_Last);
                IF->State.Kick_Phase_Last = Phase;
                IF->State.Kick_Phase_Drift += Phase_Diff;
            }

            if (E_Mag > 0.0f)
            {
                IF->State.Kick_Phase_X_Sum += IF->State.Kick_Ed_F / E_Mag;
                IF->State.Kick_Phase_Y_Sum += IF->State.Kick_Eq_F / E_Mag;
                IF->State.Kick_Valid_Cnt++;
            }
        }
    }

    if (IF->State.Kick_IF_Phase_Travel < IF_KICK_PHASE_TRAVEL_RAD)
    {
        return false;
    }
    if (IF->State.Kick_Observe_Cnt == 0U)
    {
        return true;
    }

    Valid_Ratio = (float)IF->State.Kick_Valid_Cnt / (float)IF->State.Kick_Observe_Cnt;
    if ((IF->State.Kick_Valid_Cnt == 0U) ||
        (Valid_Ratio < IF_KICK_VALID_SAMPLE_RATIO) ||
        !IF->State.Kick_Phase_Valid)
    {
        return true;
    }

    Slip_Ratio = Abs_F(IF->State.Kick_Phase_Drift) / IF->State.Kick_IF_Phase_Travel;
    Coherence = __builtin_sqrtf(IF->State.Kick_Phase_X_Sum * IF->State.Kick_Phase_X_Sum +
                                IF->State.Kick_Phase_Y_Sum * IF->State.Kick_Phase_Y_Sum) /
                (float)IF->State.Kick_Valid_Cnt;
    if (!__builtin_isfinite(Slip_Ratio) || !__builtin_isfinite(Coherence))
    {
        return true;
    }

    *Locked = (Slip_Ratio <= IF_KICK_SLIP_LOCK_RATIO) &&
              (Coherence >= IF_KICK_PHASE_COHERENCE_MIN);
    return true;
}

static void Kick_Run(IF_T *IF,
                     float Id_A,
                     float Iq_A,
                     float Ud_V,
                     float Uq_V,
                     float Ts)
{
    float Dir;
    float I_Span;
    float I_Step;
    bool Locked;

    if (Abs_F(IF->State.We_Target) <= 0.0f)
    {
        IF->State.We = 0.0f;
        Iq_Slew_Run(IF, 0.0f, Ts);
        return;
    }

    Dir = Sign_F(IF->State.We_Target);
    if (Abs_F(IF->State.We) <= 0.0f)
    {
        IF->State.Kick_Base_We = IF_KICK_WE_RATIO * IF->Para.We_Base;
        IF->State.Kick_We = Dir * IF->State.Kick_Base_We;
        IF->State.We = IF->State.Kick_We;
    }

    if (IF->State.Kick_Mode == IF_KICK_CURRENT)
    {
        Iq_Slew_Run(IF, Dir * IF->State.Kick_I_Target_A, Ts);
        if (Abs_F(IF->State.Iq) >= IF->State.Kick_I_Target_A)
        {
            Kick_Observe_Reset(IF);
            IF->State.Kick_Mode = IF_KICK_OBSERVE;
        }
        return;
    }

    Iq_Slew_Run(IF, Dir * IF->State.Kick_I_Target_A, Ts);
    if (!Kick_Observe_Run(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts, &Locked))
    {
        return;
    }

    if (Locked)
    {
        IF->State.Kick_Lock_Steps++;
        IF->State.Iq_Work_A = IF->State.Kick_I_Target_A;
        if (IF->State.Kick_Lock_Steps >= IF_KICK_LOCK_STEP_COUNT)
        {
            IF->State.Mode = (IF->State.We == IF->State.We_Target) ? IF_HOLD : IF_RAMP;
            return;
        }

        IF->State.Kick_We += Dir * IF->State.Kick_Base_We;
        if (Abs_F(IF->State.Kick_We) >= Abs_F(IF->State.We_Target))
        {
            IF->State.We = IF->State.We_Target;
            IF->State.Mode = IF_HOLD;
            return;
        }

        IF->State.We = IF->State.Kick_We;
        Kick_Observe_Reset(IF);
        return;
    }

    IF->State.Kick_Lock_Steps = 0U;
    I_Span = IF->Para.Iq_Max_A - IF->Para.Iq_Min_A;
    if ((I_Span <= 0.0f) || (IF->State.Kick_I_Target_A >= IF->Para.Iq_Max_A))
    {
        IF->State.Mode = IF_FAILED;
        return;
    }

    I_Step = IF_KICK_I_STEP_RATIO * I_Span;
    if (I_Step <= 0.0f)
    {
        IF->State.Mode = IF_FAILED;
        return;
    }

    IF->State.Kick_I_Target_A += I_Step;
    if (IF->State.Kick_I_Target_A > IF->Para.Iq_Max_A)
    {
        IF->State.Kick_I_Target_A = IF->Para.Iq_Max_A;
    }
    IF->State.Iq_Work_A = IF->State.Kick_I_Target_A;
    IF->State.Kick_Mode = IF_KICK_CURRENT;
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
    IF->State.Kick_Base_We = IF_KICK_WE_RATIO * IF->Para.We_Base;
    IF->State.Kick_I_Target_A = IF->Para.Iq_Min_A;
    Kick_Observe_Reset(IF);

    if ((IF->Para.Iq_Min_A <= 0.0f) ||
        (IF->Para.Iq_Max_A < IF->Para.Iq_Min_A) ||
        (IF->Para.We_Base <= 0.0f) ||
        (IF->Para.Acc <= 0.0f) ||
        (IF->Para.Iq_Slew_A_S <= 0.0f) ||
        (IF->Para.Rs_Ohm < 0.0f) ||
        (IF->Para.Ld_H <= 0.0f) ||
        (IF->Para.Lq_H <= 0.0f))
    {
        IF->State.Mode = IF_FAILED;
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
    if ((IF == NULL) || (IF->State.We_Target == We_Target))
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

    if ((IF == NULL) || (Theta_e == NULL) || (Id_Ref == NULL) || (Iq_Ref == NULL) || (Ts <= 0.0f))
    {
        return;
    }

    if (IF->State.Mode == IF_FAILED)
    {
        *Theta_e = IF->State.Theta_e;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (IF->State.Mode == IF_KICK)
    {
        Kick_Run(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts);
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

    *Theta_e = IF->State.Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = IF->State.Iq;
    IF->State.Theta_e = Angle_Wrap(IF->State.Theta_e + IF->State.We * Ts);
}
