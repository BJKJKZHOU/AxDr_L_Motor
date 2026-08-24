/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include "Math.h"
#include "control_params.h"

static IF_State_e State = IF_RAMP;
static uint32_t Hold_Cnt = 0U;
static float Theta_e = 0.0f;
static float We = 0.0f;
static float We_Target = IF_WE_TARGET_RAD_S;
static float Iq = 0.0f;

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

void IF_Start_Reset(float Theta_Start, float We_Start)
{
    State = IF_RAMP;
    Hold_Cnt = 0U;
    Theta_e = Angle_Wrap(Theta_Start);
    We = We_Start;
    We_Target = We_Start;
    Iq = 0.0f;
}

void IF_Start_Target_Set(float We_Target_In)
{
    if (We_Target == We_Target_In)
    {
        return;
    }

    We_Target = We_Target_In;
    Hold_Cnt = 0U;

    if (We == We_Target)
    {
        State = IF_HOLD;
    }
    else
    {
        State = IF_RAMP;
    }
}

bool IF_Start_Run(float *Theta_e_Out, float *Id_Ref, float *Iq_Ref)
{
    float Ratio;
    float Iq_Abs;
    float Iq_Target;
    float Iq_Step;
    float We_Step;

    Ratio = Abs_F(We) / IF_WE_TARGET_RAD_S;

    if (Ratio > 1.0f)
    {
        Ratio = 1.0f;
    }

    Iq_Abs = IF_IQ_START_A + (IF_IQ_TARGET_A - IF_IQ_START_A) * Ratio;

    if (State == IF_RAMP)
    {
        We_Step = IF_ACC_RAD_S2 * CUR_TS;

        if (We < We_Target)
        {
            We += We_Step;

            if (We >= We_Target)
            {
                We = We_Target;
                Hold_Cnt = 0U;
                State = IF_HOLD;
            }
        }
        else
        {
            We -= We_Step;

            if (We <= We_Target)
            {
                We = We_Target;
                Hold_Cnt = 0U;
                State = IF_HOLD;
            }
        }
    }
    else if (Hold_Cnt < IF_HOLD_CNT)
    {
        Hold_Cnt++;
    }

    if (We > 0.0f)
    {
        Iq_Target = Iq_Abs;
    }
    else if (We < 0.0f)
    {
        Iq_Target = -Iq_Abs;
    }
    else if (We_Target > 0.0f)
    {
        Iq_Target = Iq_Abs;
    }
    else if (We_Target < 0.0f)
    {
        Iq_Target = -Iq_Abs;
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

    return (State == IF_HOLD) && (Hold_Cnt >= IF_HOLD_CNT);
}

IF_State_e IF_Start_State_Get(void)
{
    return State;
}

float IF_Start_We_Get(void)
{
    return We;
}
