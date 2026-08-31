/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motion_Loop.h"

#include "Math.h"
#include "Motor_Type.h"
#include "control_params.h"

PID_T Pos_Ctrl = POSITION_CTRL_DEFAULT;
PID_T Speed_Ctrl = SPEED_CTRL_DEFAULT;

void Speed_Loop_State_Reset(float We_Fbk)
{
    Speed_Ctrl.State.Int = 0.0f;
    Speed_Ctrl.State.Fbk_Pre = We_Fbk;
}

float Position_Loop(int32_t Turn_Ref, float Theta_Ref, float Wm_Min, float Wm_Max)
{
    int32_t Turn_Err;

    Turn_Err = Turn_Ref - Motor_Run.Turn;
    Pos_Ctrl.Sig.Ref = (float)Turn_Err * TWO_PI_F + Theta_Ref;
    Pos_Ctrl.Sig.Fbk = Motor_Run.Theta_m;

    Pos_Ctrl.Para.Out_Min = Wm_Min;
    Pos_Ctrl.Para.Out_Max = Wm_Max;

    PID_Run(&Pos_Ctrl, POS_TS);

    return Pos_Ctrl.Sig.Out;
}

float Speed_Loop(float We_Ref, float We_Fbk, float Iq_Min, float Iq_Max)
{
    Speed_Ctrl.Sig.Ref = We_Ref;
    Speed_Ctrl.Sig.Fbk = We_Fbk;

    Speed_Ctrl.Para.Out_Min = Iq_Min;
    Speed_Ctrl.Para.Out_Max = Iq_Max;
    Speed_Ctrl.Para.Int_Min = Iq_Min;
    Speed_Ctrl.Para.Int_Max = Iq_Max;

    PID_Run(&Speed_Ctrl, SPD_TS);

    return Speed_Ctrl.Sig.Out;
}
