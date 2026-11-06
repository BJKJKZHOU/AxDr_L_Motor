/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Open_Loop.h"

#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motion_Type.h"
#include "VF.h"
#include "control_params.h"

uint8_t Open_Loop_Method = OPEN_LOOP_IF;

static float Theta_Open = 0.0f;
static VF_T VF = { 0 };
static uint32_t Align_Cnt = 0U;

void Open_Loop_Reset(void)
{
    Theta_Open = 0.0f;
    VF = (VF_T){ .Para.U_Slew_V_S = 30.0f };
    Align_Cnt = 0U;
}

Motor_Fast_Mode_e Open_Loop(float We_Ref, float *Theta_e,
                           float *Id_Ref, float *Iq_Ref,
                           float *Ualpha, float *Ubeta)
{
    if (Align_Cnt < OPEN_ALIGN_CNT)
    {
        *Id_Ref = OPEN_ALIGN_ID_A;

        if (++Align_Cnt >= OPEN_ALIGN_CNT)
        {
            /* VF 输出 q 轴电压，初始电压矢量与对齐时的 d 轴同向。 */
            VF.State.Theta_e = (We_Ref < 0.0f) ? 0.5f * PI_F : -0.5f * PI_F;
        }
        return FAST_CURRENT;
    }

    if (Open_Loop_Method == OPEN_LOOP_VF)
    {
        float We = VF.State.We;
        float U_Target = __builtin_fabsf(We) * Motor_Para.Flux +
                         Motor_Para.Rs * OPEN_IQ_A;

        VF.Para.U_Max_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
        VF.Para.We_Acc = (float)Motor_Para.Pp * Motion_Config.Wm_Acc;
        *Theta_e = VF.State.Theta_e;
        VF_Run(&VF, We_Ref, U_Target, Ualpha, Ubeta, CUR_TS);
        return FAST_VOLTAGE;
    }

    Theta_Open += We_Ref * CUR_TS;
    Theta_Open = Angle_Wrap(Theta_Open);
    *Theta_e = Theta_Open;
    *Iq_Ref = OPEN_IQ_A;
    return FAST_CURRENT;
}
