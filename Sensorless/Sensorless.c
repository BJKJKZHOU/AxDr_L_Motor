/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Sensorless.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Flux_Observer.h"
#include "Handover.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "Motion_Type.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "control_params.h"

#define FLUX_OBS_BW_HZ 200.0f
#define PLL_BW_HZ      65.0f
#define PLL_DAMP       0.707f
#define PLL_WN         (TWO_PI_F * PLL_BW_HZ)
#define PLL_KP         (2.0f * PLL_DAMP * PLL_WN)
#define PLL_KI         (PLL_WN * PLL_WN)

#define BLEND_S      0.15f
#define BLEND_CNT    ((uint32_t)(BLEND_S / CUR_TS + 0.5f))
#define ID_RAMP_A_S  5.0f
#define ID_RAMP_STEP (ID_RAMP_A_S * CUR_TS)

#define OBS_ENTER_BEMF_RATIO 0.10f
#define OBS_EXIT_BEMF_RATIO  0.05f
#define IF_ACC_TORQUE_RATIO   0.05f

typedef enum
{
    TO_OBS_BLEND = 0,
    TO_OBS_I_TRANS,

} To_Obs_State_e;

static volatile Sensorless_State_e State = SL_IDLE;
static volatile To_Obs_State_e To_Obs_State = TO_OBS_BLEND;
static bool Stop_Requested = false;

static Flux_Observer_T Sensorless_Observer = { 0 };
PLL_T Sensorless_PLL = { 0 };

static Handover_T Handover = { 0 };
static IF_T Sensorless_IF = { 0 };
static bool Flux_Obs_U_Valid = false;
static float We_Obs_Enter = 0.0f;
static float We_Obs_Exit = 0.0f;
static uint32_t Blend_Cnt = 0U;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Theta_Use_Last = 0.0f;

static FAST_CODE float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static FAST_CODE float Current_Limit_Get(void)
{
    float I_Max;

    I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    return (I_Max > 0.0f) ? I_Max : 0.0f;
}

static FAST_CODE float IF_Target(float We_Ref)
{
    float Target;

    if (Stop_Requested)
    {
        return We_Ref;
    }

    Target = Abs_F(We_Ref);
    if (Target > We_Obs_Enter)
    {
        Target = We_Obs_Enter;
    }

    return (We_Ref < 0.0f) ? -Target : Target;
}

bool Sensorless_Begin(void)
{
    float Kt;
    float Am_IF;

    State = SL_IDLE;
    To_Obs_State = TO_OBS_BLEND;
    Stop_Requested = false;
    Flux_Obs_U_Valid = false;
    Blend_Cnt = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Theta_Use_Last = 0.0f;
    Handover_Blend_Reset(&Handover);
    Sensorless_IF = (IF_T){ 0 };

    Align_Reset();
    Current_Loop_State_Reset();

    Sensorless_IF.Para.Iq_Max_A = Current_Limit_Get();
    if ((Motor_Para.Pp == 0U) ||
        !__builtin_isfinite(Motor_Para.Flux) || (Motor_Para.Flux <= 0.0f) ||
        !__builtin_isfinite(Motor_Para.J) || (Motor_Para.J <= 0.0f) ||
        !__builtin_isfinite(ADC.Vbus_V) || (ADC.Vbus_V <= 0.0f) ||
        !__builtin_isfinite(Motor_Config.Align_Current_A) ||
        (Motor_Config.Align_Current_A <= 0.0f) ||
        (Motor_Config.Align_Current_A > Sensorless_IF.Para.Iq_Max_A) ||
        !__builtin_isfinite(Motor_Config.IF_Current_A) ||
        (Motor_Config.IF_Current_A <= 0.0f) ||
        (Motor_Config.IF_Current_A > Sensorless_IF.Para.Iq_Max_A))
    {
        State = SL_FAILED;
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
        State = SL_FAILED;
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
    Sensorless_IF.Para.We_Base = We_Obs_Enter;
    Sensorless_IF.Para.Acc = (float)Motor_Para.Pp * Am_IF;
    Sensorless_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

    if (!__builtin_isfinite(Sensorless_IF.Para.Acc) ||
        (Sensorless_IF.Para.Acc <= 0.0f))
    {
        State = SL_FAILED;
        return false;
    }

    Sensorless_Observer.Para.Rs = Motor_Para.Rs;
    Sensorless_Observer.Para.Ls = Motor_Para.Ld;
    Sensorless_Observer.Para.Flux = Motor_Para.Flux;
    Sensorless_Observer.Para.BW_Hz = FLUX_OBS_BW_HZ;

    Sensorless_PLL.Para.Kp = PLL_KP;
    Sensorless_PLL.Para.Ki = PLL_KI;

    State = SL_ALIGN;
    return true;
}

void Sensorless_Stop_Request(void)
{
    Stop_Requested = true;
}

void Sensorless_Stop(void)
{
    Stop_Requested = false;
    State = SL_IDLE;
}

bool Sensorless_Active(void)
{
    return (State != SL_IDLE) && (State != SL_FAILED);
}

bool Sensorless_Speed_Control_Active(void)
{
    return (State == SL_OBS) ||
           ((State == SL_IF_TO_OBS) && (To_Obs_State == TO_OBS_I_TRANS));
}

float Sensorless_Wm_Get(void)
{
    float We;

    if (Motor_Para.Pp == 0U)
    {
        return 0.0f;
    }

    We = Sensorless_Speed_Control_Active() ?
             Sensorless_PLL.State.We :
             Sensorless_IF.State.We;
    return We / (float)Motor_Para.Pp;
}

void Sensorless_Control(float We_Ref, float Iq_Min, float Iq_Max)
{
    if ((State == SL_OBS) || ((State == SL_IF_TO_OBS) && (To_Obs_State == TO_OBS_I_TRANS)))
    {
        Obs_Iq_Ref = Speed_Loop(We_Ref, Sensorless_PLL.State.We, Iq_Min, Iq_Max);
    }
}

void Sensorless_Run(float Ia_A, float Ib_A, float We_Target, float *Theta_e, float *Id_Ref, float *Iq_Ref)
{
    float Ialpha;
    float Ibeta;
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

    if (!Sensorless_Active())
    {
        *Theta_e = Theta_Use_Last;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Theta_IF = 0.0f;
    Id_IF = 0.0f;
    Iq_IF = 0.0f;

    if (State == SL_ALIGN)
    {
        *Theta_e = 0.0f;
        Theta_Use_Last = 0.0f;

        if (Align_Current(Motor_Config.Align_Current_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();

            Dir = (We_Target < 0.0f) ? -1 : 1;
            We_IF_Target = IF_Target(We_Target);
            Theta_Start = -(float)Dir * (0.5f * PI_F);

            IF_Init(&Sensorless_IF, Theta_Start, 0.0f);
            Sensorless_IF.State.Iq = (float)Dir * Motor_Config.IF_Current_A;
            IF_Target_Set(&Sensorless_IF, We_IF_Target);

            Flux_Observer_Reset(&Sensorless_Observer, Theta_Start, Ialpha, Ibeta);
            PLL_Reset(&Sensorless_PLL, Theta_Start, 0.0f);
            Flux_Obs_U_Valid = false;

            State = (Sensorless_IF.State.Mode == IF_FAILED) ? SL_FAILED : SL_IF;
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

    if (State != SL_OBS)
    {
        IF_Target_Set(&Sensorless_IF, IF_Target(We_Target));
        IF_Run(&Sensorless_IF,
               &Theta_IF,
               &Id_IF,
               &Iq_IF,
               CUR_TS);
        if (Sensorless_IF.State.Mode == IF_FAILED)
        {
            State = SL_FAILED;
            *Theta_e = Theta_Use_Last;
            *Id_Ref = 0.0f;
            *Iq_Ref = 0.0f;
            return;
        }
    }

    Flux_Obs_U_Valid = true;
    Theta_Obs = Sensorless_PLL.State.Theta;

    if (State == SL_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (Obs_Valid && !Stop_Requested &&
            (Abs_F(We_Target) > We_Obs_Enter) &&
            (Abs_F(Sensorless_IF.State.We) >= We_Obs_Enter))
        {
            Handover_Blend_Reset(&Handover);
            To_Obs_State = TO_OBS_BLEND;
            State = SL_IF_TO_OBS;
        }
    }
    else if (State == SL_IF_TO_OBS)
    {
        if (To_Obs_State == TO_OBS_BLEND)
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
                I_Max = Current_Limit_Get();
                Speed_Loop_Track(Sensorless_PLL.State.We,
                                 Sensorless_PLL.State.We,
                                 Obs_Iq_Ref,
                                 -I_Max,
                                 I_Max);
                To_Obs_State = TO_OBS_I_TRANS;
            }
        }
        else
        {
            Theta_Use = Theta_Obs;
            Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, ID_RAMP_STEP);
            *Id_Ref = Obs_Id_Ref;
            *Iq_Ref = Obs_Iq_Ref;

            if (Obs_Id_Ref == 0.0f)
            {
                State = SL_OBS;
            }
        }
    }
    else if (State == SL_OBS)
    {
        Theta_Use = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;

        if (Abs_F(Sensorless_PLL.State.We) <= We_Obs_Exit)
        {
            IF_Init(&Sensorless_IF, Theta_Obs, Sensorless_PLL.State.We);
            Sensorless_IF.State.Iq = Obs_Iq_Ref;
            IF_Target_Set(&Sensorless_IF, We_Target);
            if (Sensorless_IF.State.Mode == IF_FAILED)
            {
                State = SL_FAILED;
                *Theta_e = Theta_Use_Last;
                *Id_Ref = 0.0f;
                *Iq_Ref = 0.0f;
                return;
            }
            Blend_Cnt = 0U;
            State = SL_OBS_TO_IF;
        }
    }
    else if (State == SL_OBS_TO_IF)
    {
        if (BLEND_CNT > 0U)
        {
            Blend = (float)(Blend_Cnt + 1U) / (float)BLEND_CNT;
        }
        else
        {
            Blend = 1.0f;
        }

        if (Blend > 1.0f)
        {
            Blend = 1.0f;
        }

        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Obs_Iq_Ref + Blend * (Iq_IF - Obs_Iq_Ref);

        if (Blend_Cnt < BLEND_CNT)
        {
            Blend_Cnt++;
        }

        if (Blend_Cnt >= BLEND_CNT)
        {
            State = SL_IF;
        }
    }
    else
    {
        *Theta_e = Theta_Use_Last;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    *Theta_e = Theta_Use;
    Theta_Use_Last = Theta_Use;
}

Sensorless_State_e Sensorless_State_Get(void)
{
    return State;
}
