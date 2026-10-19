/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Open_Loop.h"

#include "Align.h"
#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motion_Type.h"
#include "VF.h"
#include "control_params.h"

typedef enum
{
    OPEN_ALIGN = 0,
    OPEN_RUN,

} Open_State_e;

uint8_t Open_Loop_Method = OPEN_LOOP_IF;

static Open_State_e Open_State = OPEN_ALIGN;
static float Theta_Open = 0.0f;
static VF_T VF = { 0 };

void Open_Loop_Reset(void)
{
    Open_State = OPEN_ALIGN;
    Theta_Open = 0.0f;
    VF = (VF_T){ 0 };
    Align_Reset();
}

Motor_Fast_Mode_e Open_Loop(float We_Ref, float *Theta_e,
                           float *Id_Ref, float *Iq_Ref,
                           float *Ualpha, float *Ubeta)
{
    if (Open_State == OPEN_ALIGN)
    {
        if (Align_Current(OPEN_ALIGN_ID_A, OPEN_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Open_State = OPEN_RUN;
            /* VF outputs q-axis voltage. Start with its voltage vector aligned
             * to the d-axis used by the initial current alignment. */
            VF.State.Theta_e = (We_Ref < 0.0f) ? 0.5f * PI_F : -0.5f * PI_F;
        }
        *Theta_e = 0.0f;
        return FAST_CURRENT;
    }

    if (Open_Loop_Method == OPEN_LOOP_VF)
    {
        float We = VF.State.We;
        float U_Target = __builtin_fabsf(We) * Motor_Para.Flux +
                         Motor_Para.Rs * OPEN_IQ_A;

        VF.Para.U_Max_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
        VF.Para.U_Slew_V_S = 30.0f;
        VF.Para.We_Acc = (float)Motor_Para.Pp * Motion_Config.Wm_Acc;
        *Theta_e = VF.State.Theta_e;
        VF_Run(&VF, We_Ref, U_Target, Ualpha, Ubeta, CUR_TS);
        return FAST_VOLTAGE;
    }

    Theta_Open += We_Ref * CUR_TS;
    Theta_Open = Angle_Wrap(Theta_Open);
    *Theta_e = Theta_Open;
    *Id_Ref = 0.0f;
    *Iq_Ref = OPEN_IQ_A;
    return FAST_CURRENT;
}
