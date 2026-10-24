/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Para.h"

#include <stddef.h>

#include "Current_Loop.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motion_Loop.h"
#include "control_params.h"
#include "motor_params.h"
#include "main.h"

#define MOTOR_IF_U_BUDGET_RATIO  0.65f
#define MOTOR_IF_WE_RL_RATIO     0.15f
#define MOTOR_IF_WE_MARGIN_RATIO 0.25f
#define MOTOR_IF_WE_MIN_RAD_S    1.0f

Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;
float Control_Current_Bw_Hz = CUR_BW_HZ_DEFAULT;
float Control_Speed_Bw_Hz = SPD_BW_HZ_DEFAULT;
float Control_Flux_Obs_Bw_Hz = FLUX_OBS_BW_HZ_DEFAULT;
float Control_PLL_Bw_Hz = PLL_BW_HZ_DEFAULT;
float Control_PLL_Damp = PLL_DAMP_DEFAULT;
uint8_t Control_Current_Tune_Source = CTRL_TUNE_BANDWIDTH;
uint8_t Current_FF_Enable = 1U;
uint8_t Control_Speed_Tune_Source = CTRL_TUNE_BANDWIDTH;

Current_Manual_T Current_Manual = {
    .Id_Kp = ID_KP_DEFAULT, .Id_Ki = ID_KI_DEFAULT,
    .Iq_Kp = IQ_KP_DEFAULT, .Iq_Ki = IQ_KI_DEFAULT,
};
Speed_Manual_T Speed_Manual = { .Kp = SPD_KP_DEFAULT, .Ki = SPD_KI_DEFAULT };

void Current_Tuning_Update(void)
{
    Current_Manual_T Gain = Current_Manual;
    uint32_t Primask;

    if (Control_Current_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        float Wc = TWO_PI_F * Control_Current_Bw_Hz;

        Gain.Id_Kp = Motor_Para.Ld * Wc;
        Gain.Id_Ki = Motor_Para.Rs * Wc;
        Gain.Iq_Kp = Motor_Para.Lq * Wc;
        Gain.Iq_Ki = Motor_Para.Rs * Wc;
    }
    float Kc_LADRC = TWO_PI_F * Control_LADRC_Bw_Hz;
    float Wo_LADRC = TWO_PI_F * Control_LADRC_Eso_Bw_Hz;

    /* Publish PI and LADRC coefficients together outside the fast ISR. */
    Primask = __get_PRIMASK();
    __disable_irq();
    Id_Ctrl.Para.Kp = Gain.Id_Kp;
    Id_Ctrl.Para.Ki = Gain.Id_Ki;
    Iq_Ctrl.Para.Kp = Gain.Iq_Kp;
    Iq_Ctrl.Para.Ki = Gain.Iq_Ki;
    Id_LADRC.Para.Kp = Kc_LADRC;
    Iq_LADRC.Para.Kp = Kc_LADRC;
    Id_LADRC.Para.Wo = Wo_LADRC;
    Iq_LADRC.Para.Wo = Wo_LADRC;
    Id_LADRC.Para.B0 = 1.0f / Motor_Para.Ld;
    Iq_LADRC.Para.B0 = 1.0f / Motor_Para.Lq;
    __set_PRIMASK(Primask);
}

void Speed_Tuning_Update(void)
{
    float Kp = Speed_Manual.Kp;
    float Ki = Speed_Manual.Ki;

    if (Control_Speed_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        float Wc = TWO_PI_F * Control_Speed_Bw_Hz;
        float Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
        float Den = (float)Motor_Para.Pp * Kt;

        Kp = (Den > 0.0f) ? Motor_Para.J * Wc / Den : 0.0f;
        Ki = (Den > 0.0f) ? Motor_Para.B * Wc / Den : 0.0f;
    }
    Speed_Ctrl.Para.Kp = Kp;
    Speed_Ctrl.Para.Ki = Ki;
}

bool Motor_Para_Update(const Motor_Para_T *Para, float Eso_Bw_Hz)
{
    Mechanical_ESO_Para_T Eso;
    DPCC_Para_T Dpcc;
    float Kt = 1.5f * (float)Para->Pp * Para->Flux;
    uint32_t Primask;

    if (!Mechanical_ESO_Para_Build(&Eso, Para->J, Para->B, Kt,
                                  TWO_PI_F * Eso_Bw_Hz))
    {
        return false;
    }
    Dpcc = DPCC_Para_Calc(Para->Rs, Para->Ld, Para->Lq, CUR_TS);

    /* Build outside the critical section. ESO runs even while disabled;
     * publish its model and coefficients together without touching State. */
    Primask = __get_PRIMASK();
    __disable_irq();
    Motor_Para = *Para;
    Mechanical_ESO_Bw_Hz = Eso_Bw_Hz;
    Mechanical_ESO.Para = Eso;
    Current_DPCC = Dpcc;
    __set_PRIMASK(Primask);

    Current_Tuning_Update();
    Speed_Tuning_Update();
    return true;
}

bool Motor_IF_We_Base_Calc(float Vbus_V, float I_Max_A, float *We_Base)
{
    float U_Available_V;
    float U_Budget_V;
    float U_Per_I;
    float We_RL_Max;
    float We;

    if (We_Base == NULL)
    {
        return false;
    }

    *We_Base = 0.0f;

    if ((Vbus_V <= 0.0f) || (I_Max_A <= 0.0f) ||
        (Motor_Para.Rs <= 0.0f) || (Motor_Para.Lq <= 0.0f))
    {
        return false;
    }

    U_Available_V = Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    U_Budget_V = MOTOR_IF_U_BUDGET_RATIO * U_Available_V;
    U_Per_I = U_Budget_V / I_Max_A;

    if (U_Per_I <= Motor_Para.Rs)
    {
        return false;
    }

    We = MOTOR_IF_WE_RL_RATIO * Motor_Para.Rs / Motor_Para.Lq;
    We_RL_Max = __builtin_sqrtf(U_Per_I * U_Per_I - Motor_Para.Rs * Motor_Para.Rs) / Motor_Para.Lq;
    if (We > MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max)
    {
        We = MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max;
    }

    if (!(We >= MOTOR_IF_WE_MIN_RAD_S))
    {
        return false;
    }

    *We_Base = We;
    return true;
}
