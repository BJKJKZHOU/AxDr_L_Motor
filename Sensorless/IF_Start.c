/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stdbool.h>
#include <stddef.h>

#include "Math.h"
#include "control_params.h"

/* Kick policy is expressed with motor-relative or dimensionless quantities.
 * The only absolute current bounds are Iq_Min_A and the caller/user Iq_Max_A. */
#define IF_KICK_WE_RATIO            0.125f
#define IF_KICK_I_STEP_RATIO        0.10f
#define IF_KICK_SLIP_LOCK_RATIO     0.20f
#define IF_KICK_LOCK_STEP_COUNT     3U
#define IF_KICK_PHASE_TRAVEL_RAD    TWO_PI_F
#define IF_KICK_EMF_TAU_S           0.001f
#define IF_KICK_EMF_ALPHA           (CUR_TS / (IF_KICK_EMF_TAU_S + CUR_TS))
#define IF_KICK_EMF_MIN_RATIO       0.05f
#define IF_KICK_VALID_SAMPLE_RATIO  0.50f
#define IF_KICK_PHASE_COHERENCE_MIN 0.50f

typedef enum
{
    KICK_CURRENT = 0,
    KICK_OBSERVE,

} Kick_State_e;

static IF_State_e State = IF_RAMP;
static Kick_State_e Kick_State = KICK_CURRENT;

static float Theta_e = 0.0f;
static float We = 0.0f;
static float We_Target = IF_WE_TARGET_RAD_S;
static float Iq = 0.0f;
static float Iq_Min_A = IF_IQ_START_A;
static float Iq_Max_A = IF_IQ_TARGET_A;
static float Iq_Work_A = IF_IQ_START_A;
static float We_Base = IF_WE_TARGET_RAD_S;
static float Acc = IF_ACC_RAD_S2;
static float Rs_Ohm = 0.0f;
static float Ld_H = 0.0f;
static float Lq_H = 0.0f;

static float Kick_Base_We = 0.0f;
static float Kick_We = 0.0f;
static float Kick_I_Target_A = 0.0f;
static uint8_t Kick_Lock_Steps = 0U;

static bool Kick_Current_Valid = false;
static bool Kick_Phase_Valid = false;
static float Kick_Id_Last = 0.0f;
static float Kick_Iq_Last = 0.0f;
static float Kick_Ed_F = 0.0f;
static float Kick_Eq_F = 0.0f;
static float Kick_Phase_Last = 0.0f;
static float Kick_Phase_Drift = 0.0f;
static float Kick_IF_Phase_Travel = 0.0f;
static float Kick_Phase_X_Sum = 0.0f;
static float Kick_Phase_Y_Sum = 0.0f;
static uint32_t Kick_Observe_Cnt = 0U;
static uint32_t Kick_Valid_Cnt = 0U;

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

static void Kick_Observe_Reset(void)
{
    Kick_Current_Valid = false;
    Kick_Phase_Valid = false;
    Kick_Ed_F = 0.0f;
    Kick_Eq_F = 0.0f;
    Kick_Phase_Drift = 0.0f;
    Kick_IF_Phase_Travel = 0.0f;
    Kick_Phase_X_Sum = 0.0f;
    Kick_Phase_Y_Sum = 0.0f;
    Kick_Observe_Cnt = 0U;
    Kick_Valid_Cnt = 0U;
}

static void Iq_Slew_Run(float Target)
{
    float Step;

    Step = IF_IQ_SLEW_A_S * CUR_TS;

    if (Iq < Target)
    {
        Iq += Step;
        if (Iq > Target)
        {
            Iq = Target;
        }
    }
    else if (Iq > Target)
    {
        Iq -= Step;
        if (Iq < Target)
        {
            Iq = Target;
        }
    }
}

static bool Kick_Observe_Run(float Id_A, float Iq_A, float Ud_V, float Uq_V, bool *Locked)
{
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

    if ((Locked == NULL) || (Abs_F(We) <= 0.0f))
    {
        return false;
    }

    *Locked = false;

    if (!Kick_Current_Valid)
    {
        Kick_Id_Last = Id_A;
        Kick_Iq_Last = Iq_A;
        Kick_Current_Valid = true;
        return false;
    }

    dId = (Id_A - Kick_Id_Last) / CUR_TS;
    dIq = (Iq_A - Kick_Iq_Last) / CUR_TS;
    Kick_Id_Last = Id_A;
    Kick_Iq_Last = Iq_A;

    /* dq voltage-model residual in the I/F frame. It removes stator
     * resistance, current dynamics and frame-rotation terms, leaving the PM
     * back-EMF vector without assuming that rotor speed already equals We. */
    Ed = Ud_V - Rs_Ohm * Id_A - Ld_H * dId + We * Lq_H * Iq_A;
    Eq = Uq_V - Rs_Ohm * Iq_A - Lq_H * dIq - We * Ld_H * Id_A;

    Kick_Ed_F += IF_KICK_EMF_ALPHA * (Ed - Kick_Ed_F);
    Kick_Eq_F += IF_KICK_EMF_ALPHA * (Eq - Kick_Eq_F);

    Kick_IF_Phase_Travel += Abs_F(We) * CUR_TS;
    Kick_Observe_Cnt++;

    if (__builtin_isfinite(Kick_Ed_F) && __builtin_isfinite(Kick_Eq_F))
    {
        E_Mag = __builtin_sqrtf(Kick_Ed_F * Kick_Ed_F + Kick_Eq_F * Kick_Eq_F);
        I_Mag = __builtin_sqrtf(Id_A * Id_A + Iq_A * Iq_A);
        U_Mag = __builtin_sqrtf(Ud_V * Ud_V + Uq_V * Uq_V);
        Model_Scale = U_Mag + Rs_Ohm * I_Mag +
                      Abs_F(We) * (Ld_H * Abs_F(Id_A) + Lq_H * Abs_F(Iq_A));

        if ((Model_Scale > 0.0f) && (E_Mag >= IF_KICK_EMF_MIN_RATIO * Model_Scale))
        {
            Phase = __builtin_atan2f(Kick_Eq_F, Kick_Ed_F);

            if (!Kick_Phase_Valid)
            {
                Kick_Phase_Last = Phase;
                Kick_Phase_Valid = true;
            }
            else
            {
                Phase_Diff = Angle_Diff(Phase, Kick_Phase_Last);
                Kick_Phase_Last = Phase;
                Kick_Phase_Drift += Phase_Diff;
            }

            if (E_Mag > 0.0f)
            {
                Kick_Phase_X_Sum += Kick_Ed_F / E_Mag;
                Kick_Phase_Y_Sum += Kick_Eq_F / E_Mag;
                Kick_Valid_Cnt++;
            }
        }
    }

    if (Kick_IF_Phase_Travel < IF_KICK_PHASE_TRAVEL_RAD)
    {
        return false;
    }

    if (Kick_Observe_Cnt == 0U)
    {
        return true;
    }

    Valid_Ratio = (float)Kick_Valid_Cnt / (float)Kick_Observe_Cnt;
    if ((Kick_Valid_Cnt == 0U) || (Valid_Ratio < IF_KICK_VALID_SAMPLE_RATIO) || !Kick_Phase_Valid)
    {
        return true;
    }

    Slip_Ratio = Abs_F(Kick_Phase_Drift) / Kick_IF_Phase_Travel;
    Coherence = __builtin_sqrtf(Kick_Phase_X_Sum * Kick_Phase_X_Sum +
                                Kick_Phase_Y_Sum * Kick_Phase_Y_Sum) /
                (float)Kick_Valid_Cnt;

    if (!__builtin_isfinite(Slip_Ratio) || !__builtin_isfinite(Coherence))
    {
        return true;
    }

    *Locked = (Slip_Ratio <= IF_KICK_SLIP_LOCK_RATIO) &&
              (Coherence >= IF_KICK_PHASE_COHERENCE_MIN);
    return true;
}

static void Kick_Run(float Id_A, float Iq_A, float Ud_V, float Uq_V)
{
    float Dir;
    float I_Span;
    float I_Step;
    bool Locked;

    if (Abs_F(We_Target) <= 0.0f)
    {
        We = 0.0f;
        Iq_Slew_Run(0.0f);
        return;
    }

    Dir = Sign_F(We_Target);

    if (Abs_F(We) <= 0.0f)
    {
        Kick_Base_We = IF_KICK_WE_RATIO * We_Base;
        Kick_We = Dir * Kick_Base_We;
        We = Kick_We;
    }

    if (Kick_State == KICK_CURRENT)
    {
        Iq_Slew_Run(Dir * Kick_I_Target_A);

        if (Abs_F(Iq) >= Kick_I_Target_A)
        {
            Kick_Observe_Reset();
            Kick_State = KICK_OBSERVE;
        }
        return;
    }

    Iq_Slew_Run(Dir * Kick_I_Target_A);

    if (!Kick_Observe_Run(Id_A, Iq_A, Ud_V, Uq_V, &Locked))
    {
        return;
    }

    if (Locked)
    {
        Kick_Lock_Steps++;
        Iq_Work_A = Kick_I_Target_A;

        if (Kick_Lock_Steps >= IF_KICK_LOCK_STEP_COUNT)
        {
            State = (We == We_Target) ? IF_HOLD : IF_RAMP;
            return;
        }

        Kick_We += Dir * Kick_Base_We;
        if (Abs_F(Kick_We) >= Abs_F(We_Target))
        {
            We = We_Target;
            State = IF_HOLD;
            return;
        }

        We = Kick_We;
        Kick_Observe_Reset();
        return;
    }

    Kick_Lock_Steps = 0U;
    I_Span = Iq_Max_A - Iq_Min_A;

    if ((I_Span <= 0.0f) || (Kick_I_Target_A >= Iq_Max_A))
    {
        State = IF_FAILED;
        return;
    }

    I_Step = IF_KICK_I_STEP_RATIO * I_Span;
    if (I_Step <= 0.0f)
    {
        State = IF_FAILED;
        return;
    }

    Kick_I_Target_A += I_Step;
    if (Kick_I_Target_A > Iq_Max_A)
    {
        Kick_I_Target_A = Iq_Max_A;
    }

    Iq_Work_A = Kick_I_Target_A;
    Kick_State = KICK_CURRENT;
}

void IF_Start_Reset(float Theta_Start, float We_Start)
{
    Theta_e = Angle_Wrap(Theta_Start);
    We = We_Start;
    We_Target = We_Start;
    Iq = 0.0f;

    Kick_State = KICK_CURRENT;
    Kick_Lock_Steps = 0U;
    Kick_Base_We = IF_KICK_WE_RATIO * We_Base;
    Kick_We = 0.0f;
    Kick_I_Target_A = Iq_Min_A;
    Kick_Observe_Reset();

    if (Abs_F(We_Start) <= 0.0f)
    {
        Iq_Work_A = Iq_Min_A;
        State = IF_KICK;
    }
    else
    {
        if (Iq_Work_A < Iq_Min_A)
        {
            Iq_Work_A = Iq_Min_A;
        }
        else if (Iq_Work_A > Iq_Max_A)
        {
            Iq_Work_A = Iq_Max_A;
        }

        State = IF_RAMP;
    }
}

void IF_Start_Para_Set(float Iq_Min,
                       float Iq_Max,
                       float We_Base_In,
                       float Acc_In,
                       float Rs,
                       float Ld,
                       float Lq)
{
    if ((Iq_Min <= 0.0f) || (Iq_Max < Iq_Min) || (We_Base_In <= 0.0f) ||
        (Acc_In <= 0.0f) || (Rs < 0.0f) || (Ld <= 0.0f) || (Lq <= 0.0f))
    {
        return;
    }

    Iq_Min_A = Iq_Min;
    Iq_Max_A = Iq_Max;
    We_Base = We_Base_In;
    Acc = Acc_In;
    Rs_Ohm = Rs;
    Ld_H = Ld;
    Lq_H = Lq;

    if (Iq_Work_A < Iq_Min_A)
    {
        Iq_Work_A = Iq_Min_A;
    }
    else if (Iq_Work_A > Iq_Max_A)
    {
        Iq_Work_A = Iq_Max_A;
    }
}

void IF_Start_Target_Set(float We_Target_In)
{
    if (We_Target == We_Target_In)
    {
        return;
    }

    We_Target = We_Target_In;

    if ((State == IF_FAILED) || (State == IF_KICK))
    {
        return;
    }

    State = (We == We_Target) ? IF_HOLD : IF_RAMP;
}

void IF_Start_Run(float Id_A,
                  float Iq_A,
                  float Ud_V,
                  float Uq_V,
                  float *Theta_e_Out,
                  float *Id_Ref,
                  float *Iq_Ref)
{
    float Iq_Target;
    float We_Step;

    if ((Theta_e_Out == NULL) || (Id_Ref == NULL) || (Iq_Ref == NULL))
    {
        return;
    }

    if (State == IF_FAILED)
    {
        *Theta_e_Out = Theta_e;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (State == IF_KICK)
    {
        Kick_Run(Id_A, Iq_A, Ud_V, Uq_V);
    }
    else
    {
        if (State == IF_RAMP)
        {
            We_Step = Acc * CUR_TS;

            if (We < We_Target)
            {
                We += We_Step;
                if (We >= We_Target)
                {
                    We = We_Target;
                    State = IF_HOLD;
                }
            }
            else
            {
                We -= We_Step;
                if (We <= We_Target)
                {
                    We = We_Target;
                    State = IF_HOLD;
                }
            }
        }

        if (We > 0.0f)
        {
            Iq_Target = Iq_Work_A;
        }
        else if (We < 0.0f)
        {
            Iq_Target = -Iq_Work_A;
        }
        else
        {
            Iq_Target = 0.0f;
        }

        Iq_Slew_Run(Iq_Target);
    }

    *Theta_e_Out = Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = Iq;

    Theta_e += We * CUR_TS;
    Theta_e = Angle_Wrap(Theta_e);
}

IF_State_e IF_Start_State_Get(void)
{
    return State;
}

float IF_Start_We_Get(void)
{
    return We;
}
