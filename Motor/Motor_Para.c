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
#include "Motor_Cal.h"
#include "control_params.h"
#include "motor_params.h"

#define MOTOR_IF_U_BUDGET_RATIO  0.65f
#define MOTOR_IF_I_START_A       1.0f
#define MOTOR_IF_WE_RL_RATIO     0.02f
#define MOTOR_IF_WE_MARGIN_RATIO 0.25f
#define MOTOR_IF_WE_MIN_RAD_S    1.0f
#define MOTOR_IF_RAMP_TIME_S     6.0f

Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;
float Control_Current_Bw_Hz = CUR_BW_HZ_DEFAULT;
float Control_Speed_Bw_Hz = SPD_BW_HZ_DEFAULT;
uint8_t Control_Current_Tune_Source = CTRL_TUNE_BANDWIDTH;
uint8_t Control_Speed_Tune_Source = CTRL_TUNE_BANDWIDTH;

static float IF_We_RL_Base = MOTOR_IF_WE_RL_RATIO * MOTOR_RS_DEFAULT / MOTOR_LQ_DEFAULT;

void Current_Tuning_Update(void)
{
    float Wc;

    Wc = TWO_PI_F * Control_Current_Bw_Hz;

    Id_Ctrl.Para.Kp = Motor_Para.Ld * Wc;
    Id_Ctrl.Para.Ki = Motor_Para.Rs * Wc;
    Iq_Ctrl.Para.Kp = Motor_Para.Lq * Wc;
    Iq_Ctrl.Para.Ki = Motor_Para.Rs * Wc;
}

void Speed_Tuning_Update(void)
{
    float Wc;
    float Kt;
    float Den;

    Wc = TWO_PI_F * Control_Speed_Bw_Hz;
    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
    Den = (float)Motor_Para.Pp * Kt;

    if (Den > 0.0f)
    {
        Speed_Ctrl.Para.Kp = Motor_Para.J * Wc / Den;
        Speed_Ctrl.Para.Ki = Motor_Para.B * Wc / Den;
    }
    else
    {
        Speed_Ctrl.Para.Kp = 0.0f;
        Speed_Ctrl.Para.Ki = 0.0f;
    }
}

void Current_Tuning_Source_Changed(void)
{
    if (Control_Current_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        Current_Tuning_Update();
    }
}

void Speed_Tuning_Source_Changed(void)
{
    if (Control_Speed_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        Speed_Tuning_Update();
    }
}

void Mechanical_ESO_Tuning_Update(void)
{
    float Kt;

    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
    (void)Mechanical_ESO_Config(Motor_Para.J,
                                Motor_Para.B,
                                Kt,
                                TWO_PI_F * Mechanical_ESO_Bw_Hz);
}

/*
 * Refresh runtime values derived from the active motor model.
 *
 * NVS restores persistent Parameter values without firing per-Parameter
 * on-change hooks; after the complete record set is restored, storage calls
 * this once so runtime state is built from one coherent configuration.
 *
 * Current and speed gains follow their persisted tuning source. Bandwidth
 * mode tracks Motor_Para changes; Manual mode keeps the user-written gains.
 */
void Motor_Para_Update(void)
{
    IF_We_RL_Base = (Motor_Para.Lq > 0.0f) ?
                        MOTOR_IF_WE_RL_RATIO * Motor_Para.Rs / Motor_Para.Lq :
                        0.0f;

    if (Control_Current_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        Current_Tuning_Update();
    }

    if (Control_Speed_Tune_Source == CTRL_TUNE_BANDWIDTH)
    {
        Speed_Tuning_Update();
    }

    Mechanical_ESO_Tuning_Update();
}

void Motor_Pp_Changed(void)
{
    Motor_Cal_Invalidate();
    Motor_Para_Update();
}

bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para)
{
    float U_Available_V;
    float U_Per_I;
    float We_RL_Max;
    float We_Base;

    if (Para == NULL)
    {
        return false;
    }

    *Para = (Motor_IF_Para_T){ 0 };

    if ((Vbus_V <= 0.0f) || (I_Max_A <= 0.0f) || (Motor_Para.Rs <= 0.0f) || (Motor_Para.Lq <= 0.0f) ||
        (IF_We_RL_Base <= 0.0f))
    {
        return false;
    }

    U_Available_V = Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    Para->U_Budget_V = MOTOR_IF_U_BUDGET_RATIO * U_Available_V;

    Para->Iq_Start_A = (I_Max_A < MOTOR_IF_I_START_A) ? I_Max_A : MOTOR_IF_I_START_A;
    Para->Iq_Max_A = I_Max_A;

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

    We_RL_Max = __builtin_sqrtf(U_Per_I * U_Per_I - Motor_Para.Rs * Motor_Para.Rs) / Motor_Para.Lq;
    if (We_Base > MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max)
    {
        We_Base = MOTOR_IF_WE_MARGIN_RATIO * We_RL_Max;
    }

    if (We_Base < MOTOR_IF_WE_MIN_RAD_S)
    {
        return false;
    }

    Para->We_Base = We_Base;
    Para->Acc = We_Base / MOTOR_IF_RAMP_TIME_S;
    Para->Valid = Para->Acc > 0.0f;

    return Para->Valid;
}
