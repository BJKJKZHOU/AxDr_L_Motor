/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "PID.h"

void PID_Run(PID_T *Pid, float Ts)
{
    float P;
    float D;
    float Int_Pre;
    float Int;
    float Out;

    Pid->Sig.Err = Pid->Sig.Ref - Pid->Sig.Fbk;

    P = Pid->Para.Kp * Pid->Sig.Err;

    Int_Pre = Pid->State.Int;
    Int = Int_Pre + Pid->Para.Ki * Pid->Sig.Err * Ts;

    if (Int > Pid->Para.Int_Max)
    {
        Int = Pid->Para.Int_Max;
    }
    else if (Int < Pid->Para.Int_Min)
    {
        Int = Pid->Para.Int_Min;
    }

    D = -Pid->Para.Kd * (Pid->Sig.Fbk - Pid->State.Fbk_Pre) / Ts;

    Out = P + Int + D;

    if ((Out > Pid->Para.Out_Max) && (Int > Int_Pre))
    {
        Int = Int_Pre;
        Out = P + Int + D;
    }
    else if ((Out < Pid->Para.Out_Min) && (Int < Int_Pre))
    {
        Int = Int_Pre;
        Out = P + Int + D;
    }

    if (Out > Pid->Para.Out_Max)
    {
        Out = Pid->Para.Out_Max;
    }
    else if (Out < Pid->Para.Out_Min)
    {
        Out = Pid->Para.Out_Min;
    }

    Pid->State.Int = Int;
    Pid->State.Fbk_Pre = Pid->Sig.Fbk;
    Pid->Sig.Out = Out;
}