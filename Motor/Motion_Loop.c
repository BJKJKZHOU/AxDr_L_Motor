/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motion_Loop.h"

#include "Encoder.h"
#include "LADRC.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motor_Para.h"
#include "control_params.h"
#include "main.h"

PID_T Pos_Ctrl = POSITION_CTRL_DEFAULT;
PID_T Speed_Ctrl = SPEED_CTRL_DEFAULT;
uint8_t Speed_Controller = SPEED_CTRL_PI;
float Control_Speed_LADRC_Bw_Hz = SPD_LADRC_BW_HZ_DEFAULT;

void Speed_Loop_Track(float We_Ref, float We_Fbk, float Iq, float Iq_Min, float Iq_Max)
{
    float Err;
    float Int;

    Speed_Ctrl.Para.Out_Min = Iq_Min;
    Speed_Ctrl.Para.Out_Max = Iq_Max;
    Speed_Ctrl.Para.Int_Min = Iq_Min;
    Speed_Ctrl.Para.Int_Max = Iq_Max;

    Speed_Ctrl.Sig.Ref = We_Ref;
    Speed_Ctrl.Sig.Fbk = We_Fbk;
    Err = We_Ref - We_Fbk;
    Int = Iq - Speed_Ctrl.Para.Kp * Err;
    Limit_Value(&Int, Iq_Min, Iq_Max);

    Speed_Ctrl.Sig.Err = Err;
    Speed_Ctrl.Sig.Out = Iq;
    Speed_Ctrl.State.Int = Int;
    Speed_Ctrl.State.Fbk_Pre = We_Fbk;
}

float Position_Loop(int32_t Turn_Ref, float Theta_Ref, float Wm_Min, float Wm_Max)
{
    int32_t Turn;
    int32_t Turn_Err;
    float Theta;

    Encoder_Position_Read(&Turn, &Theta);
    Turn_Err = Turn_Ref - Turn;
    Pos_Ctrl.Sig.Ref = (float)Turn_Err * TWO_PI_F + Theta_Ref;
    Pos_Ctrl.Sig.Fbk = Theta;

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

float Speed_LADRC(float We_Ref, float Iq_Min, float Iq_Max)
{
    float Wm;
    float Td;
    float Iq;
    float Wc = TWO_PI_F * Control_Speed_LADRC_Bw_Hz;
    float Pp = (float)Motor_Para.Pp;
    uint32_t Primask;

    /* ADC updates both states at 20 kHz. Consume one coherent sample at 2 kHz;
     * the validated model is published by this same Motor thread. */
    Primask = __get_PRIMASK();
    __disable_irq();
    Wm = Mechanical_ESO.State.Wm;
    Td = Mechanical_ESO.State.Td;
    __set_PRIMASK(Primask);

    Speed_Ctrl.Sig.Ref = We_Ref;
    Speed_Ctrl.Sig.Fbk = Pp * Wm;
    Speed_Ctrl.Sig.Err = We_Ref - Speed_Ctrl.Sig.Fbk;

    /* Telemetry uses electrical speed; the algorithm uses mechanical speed. */
    Iq = LADRC_Speed(Speed_Ctrl.Sig.Err / Pp, Wm, Td, Wc,
                     Mechanical_ESO.Para.B_Over_J,
                     Mechanical_ESO.Para.Inv_J,
                     Mechanical_ESO.Para.Kt_Over_J);
    Limit_Value(&Iq, Iq_Min, Iq_Max);
    Speed_Ctrl.Sig.Out = Iq;
    return Iq;
}
