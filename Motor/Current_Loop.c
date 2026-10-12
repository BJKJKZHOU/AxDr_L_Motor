/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Current_Loop.h"

#include "Motor_Type.h"
#include "control_params.h"

#ifdef MATH_TIME
#include "Math_Time.h"
#endif

PID_T Id_Ctrl = ID_CTRL_DEFAULT;
PID_T Iq_Ctrl = IQ_CTRL_DEFAULT;

extern Motor_Run_T Motor_Run;

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
}

void Current_Loop(float Id_Ref, float Iq_Ref, float U_Lim, float *Ud, float *Uq)
{
#ifdef MATH_TIME
    uint32_t Math_T0;
#endif
    /* Id/Iq are the current fast-loop Park feedback, shared with the ESO
     * when both use the encoder electrical angle. */
    Id_Ctrl.Sig.Ref = Id_Ref;
    Id_Ctrl.Sig.Fbk = Motor_Run.Id;
    Iq_Ctrl.Sig.Ref = Iq_Ref;
    Iq_Ctrl.Sig.Fbk = Motor_Run.Iq;

    Current_Loop_Limits_Update(U_Lim);

#ifdef MATH_TIME
    Math_T0 = Math_Clock();
#endif
    PID_Run(&Id_Ctrl, CUR_TS);
#ifdef MATH_TIME
    Math_Time[MATH_PID_D].Cyc = Math_Clock() - Math_T0;
    Math_Time[MATH_PID_D].Alt = Id_Ctrl.Para.Kd != 0.0f;
    Math_T0 = Math_Clock();
#endif
    PID_Run(&Iq_Ctrl, CUR_TS);
#ifdef MATH_TIME
    Math_Time[MATH_PID_Q].Cyc = Math_Clock() - Math_T0;
    Math_Time[MATH_PID_Q].Alt = Iq_Ctrl.Para.Kd != 0.0f;
#endif

    *Ud = Id_Ctrl.Sig.Out;
    *Uq = Iq_Ctrl.Sig.Out;
}
