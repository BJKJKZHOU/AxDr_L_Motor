/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stdbool.h>

#include "Math.h"
#include "Motor_Type.h"
#include "control_params.h"

#define IF_KICK_WE_RATIO         0.125f
#define IF_KICK_I_STEP_RATIO     0.10f
#define IF_KICK_SLIP_LOCK_RATIO  0.20f
#define IF_KICK_LOCK_STEP_COUNT  3U
#define IF_KICK_PHASE_TRAVEL_RAD TWO_PI_F
#define IF_KICK_EMF_LPF_ALPHA    0.05f

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
static float Kick_Phase_Unwrap = 0.0f;
static float Kick_Phase_Start = 0.0f;
static float Kick_IF_Phase_Travel = 0.0f;

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
    Kick_Phase_Unwrap = 0.0f;
    Kick_Phase_Start = 0.0f;
    Kick_IF_Phase_Travel = 0.0f;
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

static bool Kick_Phase_Run(float *Slip_Ratio)
{
    float dId;
    float dIq;
    float Ed;
    float Eq;
    float Phase;
    float Phase_Diff;
    float Phase_Drift;

    if ((Slip_Ratio == 0) || (Abs_F(We) <= 0.0f))
    {
        return false;
    }

    if (!Kick_Current_Valid)
    {
        Kick_Id_Last = Motor_Run.Id;
        Kick_Iq_Last = Motor_Run.Iq;
        Kick_Current_Valid = true;
        return false;
    }

    dId = (Motor_Run.Id - Kick_Id_Last) / CUR_TS;
    dIq = (Motor_Run.Iq - Kick_Iq_Last) / CUR_TS;
    Kick_Id_Last = Motor_Run.Id;
    Kick_Iq_Last = Motor_Run.Iq;

    /* dq voltage-model residual in the I/F frame. It removes the stator
     * resistance, current dynamics and frame-rotation terms, leaving the PM
     * back-EMF vector. Unlike the steady-state flux formula, this does not
     * assume that rotor speed already equals the I/F speed. */
    Ed = Motor_Run.Ud - Motor_Para.Rs * Motor_Run.Id - Motor_Para.Ld * dId + We * Motor_Para.Lq * Motor_Run.Iq;
    Eq = Motor_Run.Uq - Motor_Para.Rs * Motor_Run.Iq - Motor_Para.Lq * dIq - We * Motor_Para.Ld * Motor_Run.Id;

    Kick_Ed_F += IF_KICK_EMF_LPF_ALPHA * (Ed - Kick_Ed_F);
    Kick_Eq_F += IF_KICK_EMF_LPF_ALPHA * (Eq - Kick_Eq_F);

    if (!__builtin_isfinite(Kick_Ed_F) || !__builtin_isfinite(Kick_Eq_F))
    {
        return false;
    }

    if ((Kick_Ed_F * Kick_Ed_F + Kick_Eq_F * Kick_Eq_F) <= 1.0e-12f)
    {
        return false;
    }

    Phase = __builtin_atan2f(Kick_Eq_F, Kick_Ed_F);

    if (!Kick_Phase_Valid)
    {
        Kick_Phase_Last = Phase;
        Kick_Phase_Unwrap = Phase;
        Kick_Phase_Start = Phase;
        Kick_IF_Phase_Travel = 0.0f;
        Kick_Phase_Valid = true;
        return false;
    }

    Phase_Diff = Angle_Diff(Phase, Kick_Phase_Last);
    Kick_Phase_Last = Phase;
    Kick_Phase_Unwrap += Phase_Diff;
    Kick_IF_Phase_Travel += Abs_F(We) * CUR_TS;

    if (Kick_IF_Phase_Travel < IF_KICK_PHASE_TRAVEL_RAD)
    {
        return false;
    }

    Phase_Drift = Kick_Phase_Unwrap - Kick_Phase_Start;
    *Slip_Ratio = Abs_F(Phase_Drift) / Kick_IF_Phase_Travel;
    return __builtin_isfinite(*Slip_Ratio);
}

static void Kick_Run(void)
{
    float Dir;
    float I_Span;
    float I_Step;
    float Slip_Ratio;

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

    if (!Kick_Phase_Run(&Slip_Ratio))
    {
        return;
    }

    if (Slip_Ratio <= IF_KICK_SLIP_LOCK_RATIO)
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

void IF_Start_Para_Set(float Iq_Min, float Iq_Max, float We_Base_In, float Acc_In)
{
    if ((Iq_Min <= 0.0f) || (Iq_Max < Iq_Min) || (We_Base_In <= 0.0f) || (Acc_In <= 0.0f))
    {
        return;
    }

    Iq_Min_A = Iq_Min;
    Iq_Max_A = Iq_Max;
    We_Base = We_Base_In;
    Acc = Acc_In;

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

void IF_Start_Run(float *Theta_e_Out, float *Id_Ref, float *Iq_Ref)
{
    float Iq_Target;
    float We_Step;

    if (State == IF_FAILED)
    {
        *Theta_e_Out = Theta_e;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (State == IF_KICK)
    {
        Kick_Run();
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
