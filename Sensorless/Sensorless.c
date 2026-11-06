/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Sensorless.h"

#include "Current_Loop.h"
#include "Flux_Observer.h"
#include "Handover.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "Motion_Type.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "control_params.h"

#define BLEND_S      0.15f
#define BLEND_CNT    ((uint32_t)(BLEND_S / CUR_TS + 0.5f))
#define ID_RAMP_A_S  5.0f
#define ID_RAMP_STEP (ID_RAMP_A_S * CUR_TS)

#define OBS_ENTER_BEMF_RATIO 0.10f
#define OBS_EXIT_BEMF_RATIO  0.05f
#define IF_ACC_TORQUE_RATIO   0.05f

volatile Sensorless_State_e Sensorless_State = SL_IDLE;
volatile bool Sensorless_Stop_Requested = false;

static Flux_Observer_T Sensorless_Observer = { 0 };
PLL_T Sensorless_PLL = { 0 };

static Handover_T Handover = { 0 };
static IF_T Sensorless_IF = { 0 };
static uint32_t Align_Cnt = 0U;
static bool Flux_Obs_U_Valid = false;
static float We_Obs_Enter = 0.0f;
static float We_Obs_Exit = 0.0f;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Theta_Use_Last = 0.0f;

static FAST_CODE float IF_Target(float We_Ref)
{
    float Target;

    if (Sensorless_Stop_Requested)
    {
        return We_Ref;
    }

    Target = __builtin_fabsf(We_Ref);
    if (Target > We_Obs_Enter)
    {
        Target = We_Obs_Enter;
    }

    return (We_Ref < 0.0f) ? -Target : Target;
}

bool Sensorless_Begin(void)
{
    float I_Max;
    float Kt;
    float Am_IF;
    float Pll_Wn;

    Sensorless_State = SL_IDLE;
    Sensorless_Stop_Requested = false;
    Theta_Use_Last = 0.0f;
    Sensorless_IF = (IF_T){ 0 };

    Align_Cnt = 0U;
    Current_Loop_State_Reset();

    I_Max = User_Lim.I_Max;
    if ((ADC.Vbus_V <= 0.0f) ||
        (Motor_Config.Align_Current_A > I_Max) ||
        (Motor_Config.IF_Current_A > I_Max))
    {
        Sensorless_State = SL_FAILED;
        return false;
    }

    We_Obs_Enter = OBS_ENTER_BEMF_RATIO *
                   (ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX) /
                   Motor_Para.Flux;
    We_Obs_Exit = OBS_EXIT_BEMF_RATIO *
                  (ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX) /
                  Motor_Para.Flux;
    if (!__builtin_isfinite(We_Obs_Enter) || (We_Obs_Enter <= 0.0f) ||
        !__builtin_isfinite(We_Obs_Exit) || (We_Obs_Exit <= 0.0f))
    {
        Sensorless_State = SL_FAILED;
        return false;
    }

    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
    Am_IF = IF_ACC_TORQUE_RATIO *
            Kt * Motor_Config.IF_Current_A /
            Motor_Para.J;
    if (Am_IF > Motion_Config.Wm_Acc)
    {
        Am_IF = Motion_Config.Wm_Acc;
    }

    Sensorless_IF.Para.Iq_Min_A = Motor_Config.IF_Current_A;
    Sensorless_IF.Para.Acc = (float)Motor_Para.Pp * Am_IF;
    Sensorless_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

    if (!__builtin_isfinite(Sensorless_IF.Para.Acc) ||
        (Sensorless_IF.Para.Acc <= 0.0f))
    {
        Sensorless_State = SL_FAILED;
        return false;
    }

    Sensorless_Observer.Para.Rs = Motor_Para.Rs;
    Sensorless_Observer.Para.Ld = Motor_Para.Ld;
    Sensorless_Observer.Para.Lq = Motor_Para.Lq;
    Sensorless_Observer.Para.Flux = Motor_Para.Flux;
    Sensorless_Observer.Para.BW_Hz = Control_Flux_Obs_Bw_Hz;

    Pll_Wn = TWO_PI_F * Control_PLL_Bw_Hz;
    Sensorless_PLL.Para.Kp = 2.0f * Control_PLL_Damp * Pll_Wn;
    Sensorless_PLL.Para.Ki = Pll_Wn * Pll_Wn;

    Sensorless_State = SL_ALIGN;
    return true;
}

bool Sensorless_Speed_Control_Active(void)
{
    return (Sensorless_State == SL_OBS) || (Sensorless_State == SL_IF_TO_OBS_CURRENT);
}

float Sensorless_Wm_Get(void)
{
    float We;

    We = Sensorless_Speed_Control_Active() ?
             Sensorless_PLL.State.We :
             Sensorless_IF.State.We;
    return We / (float)Motor_Para.Pp;
}

void Sensorless_Control(float We_Ref, float Iq_Min, float Iq_Max)
{
    if ((Sensorless_State == SL_OBS) || (Sensorless_State == SL_IF_TO_OBS_CURRENT))
    {
        Obs_Iq_Ref = Speed_Loop(We_Ref, Sensorless_PLL.State.We, Iq_Min, Iq_Max);
    }
}

void Sensorless_Run(float Ialpha, float Ibeta, float We_Target, float *Theta_e, float *Id_Ref, float *Iq_Ref)
{
    float Theta_Start;
    float Theta_IF;
    float Theta_Obs;
    float Theta_Use;
    float Id_IF;
    float Iq_IF;
    float Blend;
    float We_IF_Target;
    float I_Max;
    int8_t Dir;
    bool Obs_Valid = false;

    if ((Sensorless_State == SL_IDLE) || (Sensorless_State == SL_FAILED))
    {
        *Theta_e = Theta_Use_Last;
        return;
    }

    if (Sensorless_State == SL_ALIGN)
    {
        *Id_Ref = Motor_Config.Align_Current_A;

        if (++Align_Cnt >= IF_ALIGN_CNT)
        {
            float Ud_Int = Id_Ctrl.State.Int;

            Dir = (We_Target < 0.0f) ? -1 : 1;
            We_IF_Target = IF_Target(We_Target);
            Theta_Start = -(float)Dir * (0.5f * PI_F);

            IF_Init(&Sensorless_IF, Theta_Start, 0.0f);
            Sensorless_IF.State.Iq = (float)Dir * Motor_Config.Align_Current_A;
            IF_Target_Set(&Sensorless_IF, We_IF_Target);
            if (Sensorless_IF.State.Mode == IF_FAILED)
            {
                Sensorless_State = SL_FAILED;
                return;
            }

            /* 参考和积分同拍换轴，继承预定位电流及其积分电压。 */
            *Theta_e = Sensorless_IF.State.Theta_e;
            *Id_Ref = 0.0f;
            *Iq_Ref = Sensorless_IF.State.Iq;
            Id_Ctrl.State.Int = -(float)Dir * Iq_Ctrl.State.Int;
            Iq_Ctrl.State.Int = (float)Dir * Ud_Int;
            Theta_Use_Last = *Theta_e;

            Flux_Observer_Reset(&Sensorless_Observer, Theta_Start, Ialpha, Ibeta);
            Sensorless_PLL.State = (PLL_State_T){ .Theta = Angle_Wrap(Theta_Start), .We = 0.0f };
            Flux_Obs_U_Valid = false;

            Sensorless_State = SL_IF;
        }

        return;
    }

    if (Flux_Obs_U_Valid)
    {
        Obs_Valid = Flux_Observer_Run(&Sensorless_Observer,
                                       Motor_Run.Ualpha, Motor_Run.Ubeta,
                                       Ialpha, Ibeta, CUR_TS) &&
                    PLL_Run(&Sensorless_PLL,
                            Sensorless_Observer.State.PsiAlpha,
                            Sensorless_Observer.State.PsiBeta,
                            CUR_TS);
    }

    if ((Sensorless_State != SL_OBS) && (Sensorless_State != SL_IF_TO_OBS_CURRENT))
    {
        IF_Target_Set(&Sensorless_IF, IF_Target(We_Target));
        IF_Run(&Sensorless_IF,
               &Theta_IF,
               &Id_IF,
               &Iq_IF,
               CUR_TS);
        if (Sensorless_IF.State.Mode == IF_FAILED)
        {
            Sensorless_State = SL_FAILED;
            *Theta_e = Theta_Use_Last;
            return;
        }
    }

    Flux_Obs_U_Valid = true;
    Theta_Obs = Sensorless_PLL.State.Theta;

    if (Sensorless_State == SL_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (Obs_Valid && !Sensorless_Stop_Requested &&
            (__builtin_fabsf(We_Target) > We_Obs_Enter) &&
            (__builtin_fabsf(Sensorless_IF.State.We) >= We_Obs_Enter))
        {
            Handover.Blend_Cnt = 0U;
            Sensorless_State = SL_IF_TO_OBS;
        }
    }
    else if (Sensorless_State == SL_IF_TO_OBS)
    {
        if (Handover_Blend_Run(&Handover,
                               BLEND_CNT,
                               Theta_IF,
                               Theta_Obs,
                               Id_IF,
                               Iq_IF,
                               &Theta_Use,
                               Id_Ref,
                               Iq_Ref))
        {
            Obs_Id_Ref = *Id_Ref;
            Obs_Iq_Ref = *Iq_Ref;
            I_Max = User_Lim.I_Max;
            Speed_Loop_Track(Sensorless_PLL.State.We,
                             Sensorless_PLL.State.We,
                             Obs_Iq_Ref,
                             -I_Max,
                             I_Max);
            Sensorless_State = SL_IF_TO_OBS_CURRENT;
        }
    }
    else if (Sensorless_State == SL_IF_TO_OBS_CURRENT)
    {
        Theta_Use = Theta_Obs;
        Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, ID_RAMP_STEP);
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;

        if (Obs_Id_Ref == 0.0f)
        {
            Sensorless_State = SL_OBS;
        }
    }
    else if (Sensorless_State == SL_OBS)
    {
        Theta_Use = Theta_Obs;
        *Iq_Ref = Obs_Iq_Ref;

        if (__builtin_fabsf(Sensorless_PLL.State.We) <= We_Obs_Exit)
        {
            IF_Init(&Sensorless_IF, Theta_Obs, Sensorless_PLL.State.We);
            Sensorless_IF.State.Iq = Obs_Iq_Ref;
            IF_Target_Set(&Sensorless_IF, We_Target);
            if (Sensorless_IF.State.Mode == IF_FAILED)
            {
                Sensorless_State = SL_FAILED;
                *Theta_e = Theta_Use_Last;
                *Iq_Ref = 0.0f;
                return;
            }
            Handover.Blend_Cnt = 0U;
            Sensorless_State = SL_OBS_TO_IF;
        }
    }
    else
    {
        Blend = (float)(Handover.Blend_Cnt + 1U) / (float)BLEND_CNT;

        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Obs_Iq_Ref + Blend * (Iq_IF - Obs_Iq_Ref);

        if (++Handover.Blend_Cnt >= BLEND_CNT)
        {
            Sensorless_State = SL_IF;
        }
    }

    *Theta_e = Theta_Use;
    Theta_Use_Last = Theta_Use;
}
