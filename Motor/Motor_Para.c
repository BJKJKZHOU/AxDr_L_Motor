/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Para.h"

#include <stddef.h>

#include "Current_Loop.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "control_params.h"
#include "motor_params.h"

#define MOTOR_IF_U_BUDGET_RATIO  0.65f
#define MOTOR_IF_I_START_RATIO   0.15f
#define MOTOR_IF_I_MAX_RATIO     0.35f
#define MOTOR_IF_R_START_RATIO   0.40f
#define MOTOR_IF_R_MAX_RATIO     0.60f
#define MOTOR_IF_WE_RL_RATIO     0.02f
#define MOTOR_IF_WE_MARGIN_RATIO 0.25f
#define MOTOR_IF_RAMP_TIME_S     6.0f

Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;

static float IF_We_RL_Base = MOTOR_IF_WE_RL_RATIO * MOTOR_RS_DEFAULT / MOTOR_LD_DEFAULT;

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

        IF_We_RL_Base = (Motor_Para.Ld > 0.0f) ? MOTOR_IF_WE_RL_RATIO * Motor_Para.Rs / Motor_Para.Ld : 0.0f;
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

bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para)
{
    float U_Available_V;
    float I_R_Start;
    float I_R_Max;
    float U_Per_I;
    float We_RL_Max;
    float We_Base;

    if (Para == NULL)
    {
        return false;
    }

    *Para = (Motor_IF_Para_T){ 0 };

    if ((Vbus_V <= 0.0f) || (I_Max_A <= 0.0f) || (Motor_Para.Rs <= 0.0f) || (Motor_Para.Ld <= 0.0f) ||
        (IF_We_RL_Base <= 0.0f))
    {
        return false;
    }

    U_Available_V = Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    Para->U_Budget_V = MOTOR_IF_U_BUDGET_RATIO * U_Available_V;

    I_R_Start = MOTOR_IF_R_START_RATIO * Para->U_Budget_V / Motor_Para.Rs;
    I_R_Max = MOTOR_IF_R_MAX_RATIO * Para->U_Budget_V / Motor_Para.Rs;

    Para->Iq_Start_A = MOTOR_IF_I_START_RATIO * I_Max_A;
    if (Para->Iq_Start_A > I_R_Start)
    {
        Para->Iq_Start_A = I_R_Start;
    }

    Para->Iq_Max_A = MOTOR_IF_I_MAX_RATIO * I_Max_A;
    if (Para->Iq_Max_A > I_R_Max)
    {
        Para->Iq_Max_A = I_R_Max;
    }

    if ((Para->Iq_Start_A <= 0.0f) || (Para->Iq_Max_A < Para->Iq_Start_A))
    {
        return false;
    }

    We_Base = IF_We_RL_Base;
    U_Per_I = Para->U_Budget_V / Para->Iq_Max_A;

    if (U_Per_I <= Motor_Para.Rs)
    {
        return false;
    }

    We_RL_Max = __builtin_sqrtf(U_Per_I * U_Per_I - Motor_Para.Rs * Motor_Para.Rs) / Motor_Para.Ld;
    if (We_Base > MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max)
    {
        We_Base = MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max;
    }

    if (We_Base < 1.0f)
    {
        We_Base = 1.0f;
    }

    Para->We_Base = We_Base;
    Para->Acc = We_Base / MOTOR_IF_RAMP_TIME_S;
    Para->Valid = Para->Acc > 0.0f;

    return Para->Valid;
}
