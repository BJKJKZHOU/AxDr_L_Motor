/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include "Math.h"
#include "control_params.h"

#define IF_BREAKAWAY_WE_RATIO  0.50f
#define IF_BREAKAWAY_ACC_RATIO 3.0f

static IF_State_e State = IF_RAMP;
static float Theta_e = 0.0f;
static float We = 0.0f;
static float We_Target = IF_WE_TARGET_RAD_S;
static float Iq = 0.0f;
static float Iq_Start_A = IF_IQ_START_A;
static float Iq_Target_A = IF_IQ_TARGET_A;
static float We_Base = IF_WE_TARGET_RAD_S;
static float Acc = IF_ACC_RAD_S2;
static float Acc_Command = IF_ACC_RAD_S2;
static float We_Breakaway = IF_BREAKAWAY_WE_RATIO * IF_WE_TARGET_RAD_S;
static float Acc_Breakaway = IF_BREAKAWAY_ACC_RATIO * IF_ACC_RAD_S2;
static bool Breakaway_Active = false;

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static void Breakaway_Profile_Update(void)
{
    float Total_Time;
    float Breakaway_Time;
    float Normal_Time;

    We_Breakaway = IF_BREAKAWAY_WE_RATIO * We_Base;
    Acc_Breakaway = IF_BREAKAWAY_ACC_RATIO * Acc_Command;

    Total_Time = We_Base / Acc_Command;
    Breakaway_Time = We_Breakaway / Acc_Breakaway;
    Normal_Time = Total_Time - Breakaway_Time;

    if ((Normal_Time > 0.0f) && (We_Base > We_Breakaway))
    {
        Acc = (We_Base - We_Breakaway) / Normal_Time;
    }
    else
    {
        Acc = Acc_Command;
    }
}

void IF_Start_Reset(float Theta_Start, float We_Start)
{
    State = IF_RAMP;
    Theta_e = Angle_Wrap(Theta_Start);
    We = We_Start;
    We_Target = We_Start;
    Iq = 0.0f;

    Breakaway_Active = Abs_F(We_Start) < We_Breakaway;
    if (Breakaway_Active)
    {
        Breakaway_Profile_Update();
    }
    else
    {
        Acc = Acc_Command;
    }
}

void IF_Start_Para_Set(float Iq_Start, float Iq_Target, float We_Base_In, float Acc_In)
{
    if ((Iq_Start <= 0.0f) || (Iq_Target < Iq_Start) || (We_Base_In <= 0.0f) || (Acc_In <= 0.0f))
    {
        return;
    }

    Iq_Start_A = Iq_Start;
    Iq_Target_A = Iq_Target;
    We_Base = We_Base_In;
    Acc_Command = Acc_In;

    if (Breakaway_Active)
    {
        Breakaway_Profile_Update();
    }
    else
    {
        Acc = Acc_Command;
        We_Breakaway = IF_BREAKAWAY_WE_RATIO * We_Base;
        Acc_Breakaway = IF_BREAKAWAY_ACC_RATIO * Acc_Command;
    }
}

void IF_Start_Target_Set(float We_Target_In)
{
    if (We_Target == We_Target_In)
    {
        return;
    }

    We_Target = We_Target_In;

    if (We == We_Target)
    {
        State = IF_HOLD;
    }
    else
    {
        State = IF_RAMP;
    }
}

void IF_Start_Run(float *Theta_e_Out, float *Id_Ref, float *Iq_Ref)
{
    float Acc_Use;
    float Iq_Target;
    float Iq_Step;
    float We_Step;

    if (Breakaway_Active && (Abs_F(We) >= We_Breakaway))
    {
        Breakaway_Active = false;
    }

    Acc_Use = Breakaway_Active ? Acc_Breakaway : Acc;

    if (State == IF_RAMP)
    {
        We_Step = Acc_Use * CUR_TS;

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
        Iq_Target = Iq_Target_A;
    }
    else if (We < 0.0f)
    {
        Iq_Target = -Iq_Target_A;
    }
    else if (We_Target > 0.0f)
    {
        Iq_Target = Iq_Target_A;
    }
    else if (We_Target < 0.0f)
    {
        Iq_Target = -Iq_Target_A;
    }
    else
    {
        Iq_Target = 0.0f;
    }

    Iq_Step = IF_IQ_SLEW_A_S * CUR_TS;

    if (Iq < Iq_Target)
    {
        Iq += Iq_Step;

        if (Iq > Iq_Target)
        {
            Iq = Iq_Target;
        }
    }
    else if (Iq > Iq_Target)
    {
        Iq -= Iq_Step;

        if (Iq < Iq_Target)
        {
            Iq = Iq_Target;
        }
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
