/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Current_Loop.h"

#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"

PID_T Id_Ctrl = ID_CTRL_DEFAULT;
PID_T Iq_Ctrl = IQ_CTRL_DEFAULT;

extern Motor_Run_T Motor_Run;

void Current_Loop_State_Reset(void)
{
    Id_Ctrl.State.Int = 0.0f;
    Iq_Ctrl.State.Int = 0.0f;
    Id_Ctrl.Sig.Out = 0.0f;
    Iq_Ctrl.Sig.Out = 0.0f;
}

void Current_Loop_Gain_Save(Current_Loop_Gain_T *Gain)
{
    if (Gain == 0)
    {
        return;
    }

    Gain->Id_Kp = Id_Ctrl.Para.Kp;
    Gain->Id_Ki = Id_Ctrl.Para.Ki;
    Gain->Iq_Kp = Iq_Ctrl.Para.Kp;
    Gain->Iq_Ki = Iq_Ctrl.Para.Ki;
}

void Current_Loop_Gain_Set_RL(float Rs, float Ld, float Lq)
{
    Id_Ctrl.Para.Kp = Ld * CUR_WC_DEFAULT;
    Id_Ctrl.Para.Ki = Rs * CUR_WC_DEFAULT;
    Iq_Ctrl.Para.Kp = Lq * CUR_WC_DEFAULT;
    Iq_Ctrl.Para.Ki = Rs * CUR_WC_DEFAULT;
    Current_Loop_State_Reset();
}

void Current_Loop_Gain_Restore(const Current_Loop_Gain_T *Gain)
{
    if (Gain == 0)
    {
        return;
    }

    Id_Ctrl.Para.Kp = Gain->Id_Kp;
    Id_Ctrl.Para.Ki = Gain->Id_Ki;
    Iq_Ctrl.Para.Kp = Gain->Iq_Kp;
    Iq_Ctrl.Para.Ki = Gain->Iq_Ki;
    Current_Loop_State_Reset();
}

void Current_Loop(float Id_Ref, float Iq_Ref, float *Ualpha, float *Ubeta)
{
    float Ialpha;
    float Ibeta;
    float Sin;
    float Cos;
    float Ud;
    float Uq;
    float U_Lim;

    SinCos(Motor_Run.Theta_e, &Sin, &Cos);

    Ialpha = ADC.Ia_A;
    Ibeta = (ADC.Ia_A + 2.0f * ADC.Ib_A) * INV_SQRT3_F;

    Motor_Run.Id = Ialpha * Cos + Ibeta * Sin;
    Motor_Run.Iq = -Ialpha * Sin + Ibeta * Cos;

    /*
     * Internal sign invariant for the logical ABC phase convention:
     *   +Iq -> +Te -> internal positive mechanical direction.
     * Servo phase search establishes this internal relationship. User-facing
     * direction reversal is handled only by Motor_Config.Dir and must not
     * change Enc_Dir, phase order or Theta_Off.
     */
    Id_Ctrl.Sig.Ref = Id_Ref;
    Id_Ctrl.Sig.Fbk = Motor_Run.Id;
    Iq_Ctrl.Sig.Ref = Iq_Ref;
    Iq_Ctrl.Sig.Fbk = Motor_Run.Iq;

    U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    Id_Ctrl.Para.Out_Max = U_Lim;
    Id_Ctrl.Para.Out_Min = -U_Lim;
    Id_Ctrl.Para.Int_Max = U_Lim;
    Id_Ctrl.Para.Int_Min = -U_Lim;

    Iq_Ctrl.Para.Out_Max = U_Lim;
    Iq_Ctrl.Para.Out_Min = -U_Lim;
    Iq_Ctrl.Para.Int_Max = U_Lim;
    Iq_Ctrl.Para.Int_Min = -U_Lim;

    PID_Run(&Id_Ctrl, CUR_TS);
    PID_Run(&Iq_Ctrl, CUR_TS);

    Ud = Id_Ctrl.Sig.Out;
    Uq = Iq_Ctrl.Sig.Out;

    Vector2_Limit(&Ud, &Uq, U_Lim);

    Motor_Run.Ud = Ud;
    Motor_Run.Uq = Uq;

    *Ualpha = Ud * Cos - Uq * Sin;
    *Ubeta = Ud * Sin + Uq * Cos;
}
