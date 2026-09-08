/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include "Math.h"

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static void IF_Profile_Build(IF_T *IF, float We_Start)
{
    float Total_Time;
    float Breakaway_Time;
    float Normal_Time;

    IF->State.We_Breakaway = IF->Para.Breakaway_We_Ratio * IF->Para.We_Base;
    IF->State.Acc_Breakaway = IF->Para.Breakaway_Acc_Ratio * IF->Para.Acc;
    IF->State.Acc_Run = IF->Para.Acc;
    IF->State.Initial_Profile_Active = false;
    IF->State.Breakaway_Active = false;

    if ((IF->Para.We_Base <= 0.0f) || (IF->Para.Acc <= 0.0f) ||
        (IF->Para.Breakaway_We_Ratio <= 0.0f) || (IF->Para.Breakaway_We_Ratio >= 1.0f) ||
        (IF->Para.Breakaway_Acc_Ratio <= 1.0f) ||
        (IF->State.Acc_Breakaway <= 0.0f) ||
        (Abs_F(We_Start) >= IF->State.We_Breakaway))
    {
        return;
    }

    Total_Time = IF->Para.We_Base / IF->Para.Acc;
    Breakaway_Time = IF->State.We_Breakaway / IF->State.Acc_Breakaway;
    Normal_Time = Total_Time - Breakaway_Time;
    if ((Normal_Time <= 0.0f) || (IF->Para.We_Base <= IF->State.We_Breakaway))
    {
        return;
    }

    IF->State.Acc_Run = (IF->Para.We_Base - IF->State.We_Breakaway) / Normal_Time;
    IF->State.Initial_Profile_Active = true;
    IF->State.Breakaway_Active = true;
}

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    if (IF == 0)
    {
        return;
    }

    IF->State = (IF_State_T){ 0 };
    IF->State.Mode = IF_RAMP;
    IF->State.Theta_e = Angle_Wrap(Theta_Start);
    IF->State.We = We_Start;
    IF->State.We_Target = We_Start;
    IF_Profile_Build(IF, We_Start);
}

void IF_Target_Set(IF_T *IF, float We_Target)
{
    if (IF == 0)
    {
        return;
    }

    if (IF->State.We_Target == We_Target)
    {
        return;
    }

    IF->State.We_Target = We_Target;
    IF->State.Mode = (IF->State.We == We_Target) ? IF_HOLD : IF_RAMP;
}

void IF_Run(IF_T *IF, float *Theta_e, float *Id_Ref, float *Iq_Ref, float Ts)
{
    float Acc_Use;
    float Iq_Target;
    float Iq_Step;
    float We_Step;

    if ((IF == 0) || (Theta_e == 0) || (Id_Ref == 0) || (Iq_Ref == 0) || (Ts <= 0.0f))
    {
        return;
    }

    if (IF->State.Breakaway_Active && (Abs_F(IF->State.We) >= IF->State.We_Breakaway))
    {
        IF->State.Breakaway_Active = false;
    }

    if (IF->State.Initial_Profile_Active)
    {
        Acc_Use = IF->State.Breakaway_Active ? IF->State.Acc_Breakaway : IF->State.Acc_Run;
    }
    else
    {
        Acc_Use = IF->Para.Acc;
    }

    if ((IF->State.Mode == IF_RAMP) && (Acc_Use > 0.0f))
    {
        We_Step = Acc_Use * Ts;

        if (IF->State.We < IF->State.We_Target)
        {
            IF->State.We += We_Step;
            if (IF->State.We >= IF->State.We_Target)
            {
                IF->State.We = IF->State.We_Target;
                IF->State.Mode = IF_HOLD;
                IF->State.Initial_Profile_Active = false;
                IF->State.Breakaway_Active = false;
            }
        }
        else
        {
            IF->State.We -= We_Step;
            if (IF->State.We <= IF->State.We_Target)
            {
                IF->State.We = IF->State.We_Target;
                IF->State.Mode = IF_HOLD;
                IF->State.Initial_Profile_Active = false;
                IF->State.Breakaway_Active = false;
            }
        }
    }

    if (IF->State.We > 0.0f)
    {
        Iq_Target = IF->Para.Iq_Run_A;
    }
    else if (IF->State.We < 0.0f)
    {
        Iq_Target = -IF->Para.Iq_Run_A;
    }
    else if (IF->State.We_Target > 0.0f)
    {
        Iq_Target = IF->Para.Iq_Run_A;
    }
    else if (IF->State.We_Target < 0.0f)
    {
        Iq_Target = -IF->Para.Iq_Run_A;
    }
    else
    {
        Iq_Target = 0.0f;
    }

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
