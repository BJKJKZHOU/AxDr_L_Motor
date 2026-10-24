/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Current_Loop.h"

#include "Motor_Para.h"
#include "Motor_Type.h"
#include "control_params.h"

PID_T Id_Ctrl = ID_CTRL_DEFAULT;
PID_T Iq_Ctrl = IQ_CTRL_DEFAULT;
LADRC_T Id_LADRC = { 0 };
LADRC_T Iq_LADRC = { 0 };

uint8_t Current_Controller = CURRENT_CTRL_PI;
float Control_LADRC_Bw_Hz = CUR_LADRC_BW_HZ_DEFAULT;
float Control_LADRC_Eso_Bw_Hz = CUR_LADRC_ESO_BW_HZ_DEFAULT;

static FAST_CODE void Current_Loop_Limits_Update(float U_Lim)
{
    Id_Ctrl.Para.Out_Max = U_Lim;
    Id_Ctrl.Para.Out_Min = -U_Lim;
    Id_Ctrl.Para.Int_Max = U_Lim;
    Id_Ctrl.Para.Int_Min = -U_Lim;

    Iq_Ctrl.Para.Out_Max = U_Lim;
    Iq_Ctrl.Para.Out_Min = -U_Lim;
    Iq_Ctrl.Para.Int_Max = U_Lim;
    Iq_Ctrl.Para.Int_Min = -U_Lim;
}

void Current_Loop_State_Reset(void)
{
    Id_Ctrl.State.Int = 0.0f;
    Iq_Ctrl.State.Int = 0.0f;
    Id_Ctrl.Sig.Out = 0.0f;
    Iq_Ctrl.Sig.Out = 0.0f;

    Id_LADRC.State.Z1 = 0.0f;
    Id_LADRC.State.Z2 = 0.0f;
    Iq_LADRC.State.Z1 = 0.0f;
    Iq_LADRC.State.Z2 = 0.0f;
    Id_LADRC.Sig.Out = 0.0f;
    Iq_LADRC.Sig.Out = 0.0f;
    Id_LADRC.Sig.Ref_Dot = 0.0f;
    Iq_LADRC.Sig.Ref_Dot = 0.0f;
}

/* Follow the last VF voltage with the already selected current controller.
 * PI needs its integral, LADRC needs the measured state and disturbance. */
void Current_Loop_Track(float Id, float Iq, float Ud, float Uq,
                        float Ud_FF, float Uq_FF)
{
    if (Current_Controller == CURRENT_CTRL_PI)
    {
        Id_Ctrl.Sig.Ref = Id;
        Id_Ctrl.Sig.Fbk = Id;
        Id_Ctrl.Sig.Err = 0.0f;
        Id_Ctrl.Sig.Out = Ud;
        Id_Ctrl.State.Int = Ud;
        Id_Ctrl.State.Fbk_Pre = Id;

        Iq_Ctrl.Sig.Ref = Iq;
        Iq_Ctrl.Sig.Fbk = Iq;
        Iq_Ctrl.Sig.Err = 0.0f;
        Iq_Ctrl.Sig.Out = Uq;
        Iq_Ctrl.State.Int = Uq;
        Iq_Ctrl.State.Fbk_Pre = Iq;
        return;
    }

    Id_LADRC.State.Z1 = Id;
    Iq_LADRC.State.Z1 = Iq;
    Id_LADRC.Sig.F0 = (-Motor_Para.Rs * Id - Ud_FF) * Id_LADRC.Para.B0;
    Iq_LADRC.Sig.F0 = (-Motor_Para.Rs * Iq - Uq_FF) * Iq_LADRC.Para.B0;
    Id_LADRC.State.Z2 = -(Ud + Ud_FF) * Id_LADRC.Para.B0 - Id_LADRC.Sig.F0;
    Iq_LADRC.State.Z2 = -(Uq + Uq_FF) * Iq_LADRC.Para.B0 - Iq_LADRC.Sig.F0;
    Id_LADRC.Sig.Out = Ud + Ud_FF;
    Iq_LADRC.Sig.Out = Uq + Uq_FF;
}

void Current_Loop(float Id_Ref, float Iq_Ref, float U_Lim,
                  bool Sensored,
                  float Ud_FF, float Uq_FF,
                  float *Ud, float *Uq)
{
    /* Shared FRF/Plot channels report the active controller. */
    Id_Ctrl.Sig.Ref = Id_Ref;
    Id_Ctrl.Sig.Fbk = Motor_Run.Id;
    Iq_Ctrl.Sig.Ref = Iq_Ref;
    Iq_Ctrl.Sig.Fbk = Motor_Run.Iq;

    if ((Current_Controller == CURRENT_CTRL_LADRC) && Sensored)
    {
        Id_LADRC.Sig.Ref = Id_Ref;
        Id_LADRC.Sig.Fbk = Motor_Run.Id;
        Id_LADRC.Sig.F0 = (-Motor_Para.Rs * Motor_Run.Id - Ud_FF) * Id_LADRC.Para.B0;
        Id_LADRC.Sig.U_Applied = Motor_Run.Ud;

        Iq_LADRC.Sig.Ref = Iq_Ref;
        Iq_LADRC.Sig.Fbk = Motor_Run.Iq;
        Iq_LADRC.Sig.F0 = (-Motor_Para.Rs * Motor_Run.Iq - Uq_FF) * Iq_LADRC.Para.B0;
        Iq_LADRC.Sig.U_Applied = Motor_Run.Uq;

        LADRC_Run(&Id_LADRC, CUR_TS);
        LADRC_Run(&Iq_LADRC, CUR_TS);

        /* LSEF gives total voltage. Remove the model feedforward here,
         * since the shared FOC path adds it once after this controller. */
        *Ud = Id_LADRC.Sig.Out - Ud_FF;
        *Uq = Iq_LADRC.Sig.Out - Uq_FF;

        /* Plot reports the selected controller output before shared feedforward. */
        Id_Ctrl.Sig.Out = *Ud;
        Iq_Ctrl.Sig.Out = *Uq;
        return;
    }

    Current_Loop_Limits_Update(U_Lim);

    PID_Run(&Id_Ctrl, CUR_TS);
    PID_Run(&Iq_Ctrl, CUR_TS);

    *Ud = Id_Ctrl.Sig.Out;
    *Uq = Iq_Ctrl.Sig.Out;
}
