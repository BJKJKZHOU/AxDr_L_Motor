/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Identification.h"

#include "Flux.h"
#include "JB.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Para.h"
#include "Rs_Ls.h"
#include "control_params.h"
#include "main.h"

#define IDENT_U_MAX_RATIO 0.80f

Flux_Observer_T Ident_Observer = { 0 };
PLL_T Ident_PLL = { 0 };
volatile float Ident_JB_Excite_Ratio = 0.20f;
volatile float Ident_JB_Excite_Hz = 3.0f;

static volatile Ident_Mode_e Ident_Mode = IDENT_NONE;
static volatile Ident_State_e Ident_State = IDENT_IDLE;
volatile uint8_t Ident_Fail_Reason = IDENT_FAIL_NONE;
static Ident_Envelope_T Ident_Envelope = { 0 };
static uint32_t Finish_Cyc = 0U;

static void Envelope_Voltage_Update(void)
{
    Ident_Envelope.U_Available = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    if (Ident_Envelope.U_Available < 0.0f)
    {
        Ident_Envelope.U_Available = 0.0f;
    }

    Ident_Envelope.U_Max = IDENT_U_MAX_RATIO * Ident_Envelope.U_Available;
}

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target)
{
    if (Identification_Active())
    {
        return false;
    }

    /* 新的一次辨识只重置流程；各项结果由对应算法在开始时更新。 */
    Ident_Mode = IDENT_NONE;
    Ident_State = IDENT_IDLE;
    Ident_Fail_Reason = IDENT_FAIL_NONE;

    Ident_Envelope.I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    Envelope_Voltage_Update();
    if (!__builtin_isfinite(Ident_Envelope.I_Max) ||
        (Ident_Envelope.I_Max <= 0.0f) || (Ident_Envelope.U_Max <= 0.0f))
    {
        Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
        return false;
    }

    if (Mode == IDENT_RS_LS)
    {
        if (!__builtin_isfinite(Motor_Config.RL_I_Peak_A) ||
            (Motor_Config.RL_I_Peak_A * IDENT_RL_IAC_RATIO < IDENT_RL_IAC_MIN_A) ||
            (Motor_Config.RL_I_Peak_A >= Ident_Envelope.I_Max))
        {
            Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
            return false;
        }

        Rs_Ls_Start();
    }
    else if (Mode == IDENT_FLUX)
    {
        if (!Flux_Start(Wm_Target))
        {
            Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
            return false;
        }
    }
    else if (!JB_Start(Wm_Target))
    {
        Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
        return false;
    }

    Ident_Mode = Mode;
    Ident_State = IDENT_RUNNING;
    return true;
}

void Identification_Abort(void)
{
    if (!Identification_Active())
    {
        return;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Abort();
    }
    else if (Ident_Mode == IDENT_JB)
    {
        JB_Abort();
    }

    Ident_Mode = IDENT_NONE;
    Ident_State = IDENT_IDLE;
}

void Identification_Control(void)
{
    if (Ident_State == IDENT_FINISH)
    {
        if ((uint32_t)(DWT->CYCCNT - Finish_Cyc) >= SystemCoreClock / 2U)
        {
            Ident_State = IDENT_DONE;
        }
        return;
    }

    if (Ident_State != IDENT_RUNNING)
    {
        return;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        if (Rs_Ls_Active())
        {
            return;
        }

        Ident_State = Rs_Ls_Result.Valid ? IDENT_FINISH : IDENT_FAILED;
    }
    else if (Ident_Mode == IDENT_FLUX)
    {
        Flux_Control();

        if (Flux_Active())
        {
            return;
        }

        if (Flux_Result.Valid)
        {
            Ident_State = IDENT_FINISH;
        }
        else
        {
            if (Ident_Fail_Reason == IDENT_FAIL_NONE)
            {
                Ident_Fail_Reason = IDENT_FAIL_FLUX_INTERNAL;
            }
            Ident_State = IDENT_FAILED;
        }
    }
    else if (Ident_Mode == IDENT_JB)
    {
        JB_Control();

        if (JB_Active())
        {
            return;
        }

        if (JB_Result.Valid)
        {
            Ident_State = IDENT_FINISH;
        }
        else
        {
            if (Ident_Fail_Reason == IDENT_FAIL_NONE)
            {
                Ident_Fail_Reason = IDENT_FAIL_JB_INTERNAL;
            }
            Ident_State = IDENT_FAILED;
        }
    }

    if (Ident_State == IDENT_FINISH)
    {
        Finish_Cyc = DWT->CYCCNT;
    }
}

bool Identification_Apply(void)
{
    Motor_Para_T Model = Motor_Para;
    if (Ident_State != IDENT_DONE)
    {
        return false;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        Model.Rs = Rs_Ls_Result.Rs_Ohm;
        Model.Ld = Rs_Ls_Result.Ls_H;
        Model.Lq = Rs_Ls_Result.Ls_H;
    }
    else if (Ident_Mode == IDENT_FLUX)
    {
        Model.Flux = Flux_Result.Flux_Wb;
    }
    else if (Ident_Mode == IDENT_JB)
    {
        Model.J = JB_Result.J_Kgm2;
        Model.B = JB_Result.B_Nms;
    }
    else
    {
        return false;
    }

    return Motor_Para_Update(&Model, Mechanical_ESO_Bw_Hz);
}

bool Identification_Active(void)
{
    return (Ident_State == IDENT_RUNNING) || (Ident_State == IDENT_FINISH);
}

bool Identification_Result_Valid(void)
{
    /* 只有算法结果有效并完成收尾等待，流程才会进入 DONE。 */
    return Ident_State == IDENT_DONE;
}

Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float Ic_A,
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V)
{
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (Ident_State != IDENT_RUNNING)
    {
        return FAST_OFF;
    }

    Envelope_Voltage_Update();

    if (Ident_Mode == IDENT_RS_LS)
    {
        return Rs_Ls_Run(Ia_A, Theta_e, Id_Ref, Iq_Ref, Ualpha_V, Ubeta_V);
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return Flux_Fast_Run(Ia_A,
                             Ib_A,
                             Theta_e,
                             Id_Ref,
                             Iq_Ref,
                             Ualpha_V,
                             Ubeta_V);
    }

    if (Ident_Mode == IDENT_JB)
    {
        return JB_Fast_Run(Ia_A,
                           Ib_A,
                           Ic_A,
                           Theta_e,
                           Id_Ref,
                           Iq_Ref,
                           Ualpha_V,
                           Ubeta_V);
    }

    return FAST_OFF;
}

const Ident_Envelope_T *Identification_Envelope_Get(void)
{
    return &Ident_Envelope;
}

