/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Para.h"

#include "Current_Loop.h"
#include "Motion_Loop.h"
#include "control_params.h"
#include "motor_params.h"

Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;

void Motor_Para_Changed(uint32_t Changed)
{
    float Kt;
    float Den;

    if ((Changed & MOTOR_PARA_RL) != 0U)
    {
        Id_Ctrl.Para.Kp = Motor_Para.Ld * CUR_WC_DEFAULT;
        Id_Ctrl.Para.Ki = Motor_Para.Rs * CUR_WC_DEFAULT;
        Iq_Ctrl.Para.Kp = Motor_Para.Lq * CUR_WC_DEFAULT;
        Iq_Ctrl.Para.Ki = Motor_Para.Rs * CUR_WC_DEFAULT;
    }

    if ((Changed & (MOTOR_PARA_PP | MOTOR_PARA_FLUX | MOTOR_PARA_JB)) != 0U)
    {
        Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
        Den = (float)Motor_Para.Pp * Kt;

        if (Den > 0.0f)
        {
            Speed_Ctrl.Para.Kp = Motor_Para.J * SPD_WC_DEFAULT / Den;
            Speed_Ctrl.Para.Ki = Motor_Para.B * SPD_WC_DEFAULT / Den;
        }
        else
        {
            Speed_Ctrl.Para.Kp = 0.0f;
            Speed_Ctrl.Para.Ki = 0.0f;
        }
    }
}
