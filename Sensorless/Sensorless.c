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
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "control_params.h"

#define FLUX_OBS_BW_HZ 200.0f
#define PLL_BW_HZ      50.0f
#define PLL_DAMP       0.707f
#define PLL_WN         (TWO_PI_F * PLL_BW_HZ)
#define PLL_KP         (2.0f * PLL_DAMP * PLL_WN)
#define PLL_KI         (PLL_WN * PLL_WN)

#define OBS_WAIT_S   0.20f
#define OBS_WAIT_CNT ((uint32_t)(OBS_WAIT_S / CUR_TS + 0.5f))
#define BLEND_S      0.15f
#define BLEND_CNT    ((uint32_t)(BLEND_S / CUR_TS + 0.5f))
#define ID_RAMP_A_S  5.0f
#define ID_RAMP_STEP (ID_RAMP_A_S * CUR_TS)

#define OBS_WE_TAU_S    0.020f
#define OBS_WE_ALPHA    (CUR_TS / (OBS_WE_TAU_S + CUR_TS))
#define WE_ERR_MAX      5.0f
#define PLL_ERR_MAX     0.08f
#define FLUX_MIN_RATIO2 0.64f
#define FLUX_MAX_RATIO2 1.44f

#define IF_BREAKAWAY_WE_RATIO  0.50f
#define IF_BREAKAWAY_ACC_RATIO 3.0f

/* Temporary handover boundary. Replace with motor-dependent observer-quality
 * criteria after low-speed hardware characterization. */
#define OBS_TO_IF_WE_RAD_S (0.75f * IF_WE_TARGET_RAD_S)

typedef enum
{
    TO_OBS_WAIT = 0,
    TO_OBS_BLEND,
    TO_OBS_I_TRANS,

} To_Obs_State_e;

static volatile Sensorless_State_e State = SL_IDLE;
static volatile To_Obs_State_e To_Obs_State = TO_OBS_WAIT;
static bool Initial_IF = true;

Flux_Observer_T Flux_Obs = { 0 };
PLL_T Flux_PLL = { 0 };

static Handover_T Handover = { 0 };
static IF_T Sensorless_IF = { 0 };
static bool Flux_Obs_U_Valid = false;
static Motor_IF_Para_T IF_Para = { 0 };
static uint32_t Obs_Wait_Cnt = 0U;
static uint32_t Blend_Cnt = 0U;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Theta_Use_Last = 0.0f;
static float We_Obs_F = 0.0f;

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Current_Limit_Get(void)
{
    float I_Max;

    I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    return (I_Max > 0.0f) ? I_Max : 0.0f;
}

static bool Obs_Stable(void)
{
    float We_Err;
    float Flux2;
    float Flux_Ref2;

    We_Obs_F += OBS_WE_ALPHA * (Flux_PLL.State.We - We_Obs_F);
    We_Err = We_Obs_F - Sensorless_IF.State.We;
    Flux2 = Flux_Obs.State.PsiAlpha * Flux_Obs.State.PsiAlpha + Flux_Obs.State.PsiBeta * Flux_Obs.State.PsiBeta;
    Flux_Ref2 = Flux_Obs.Para.Flux * Flux_Obs.Para.Flux;

    if (Flux_Ref2 <= 0.0f)
    {
        return false;
    }

    if (Abs_F(We_Err) > WE_ERR_MAX)
    {
        return false;
    }

    if (Abs_F(Flux_PLL.State.Err) > PLL_ERR_MAX)
    {
        return false;
    }

    if ((Flux2 < FLUX_MIN_RATIO2 * Flux_Ref2) || (Flux2 > FLUX_MAX_RATIO2 * Flux_Ref2))
    {
        return false;
    }

    return true;
}

static float IF_Target(float We_Ref)
{
    float Target;

    if (!Initial_IF)
    {
        return We_Ref;
    }

    Target = Abs_F(We_Ref);
    if (Target < IF_WE_TARGET_RAD_S)
    {
        Target = IF_WE_TARGET_RAD_S;
    }

    return (We_Ref < 0.0f) ? -Target : Target;
}

bool Sensorless_Begin(void)
{
    State = SL_IDLE;
    To_Obs_State = TO_OBS_WAIT;
    Initial_IF = true;
    Flux_Obs_U_Valid = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Theta_Use_Last = 0.0f;
    We_Obs_F = 0.0f;
    Handover_Reset(&Handover);
    Sensorless_IF = (IF_T){ 0 };

    Align_Reset();
    Current_Loop_State_Reset();

    if (!Motor_IF_Para_Build(ADC.Vbus_V, Current_Limit_Get(), &IF_Para))
    {
        State = SL_FAILED;
        return false;
    }

    Sensorless_IF.Para.Iq_Run_A = IF_Para.Iq_Max_A;
    Sensorless_IF.Para.We_Base = IF_Para.We_Base;
    Sensorless_IF.Para.Acc = IF_Para.Acc;
    Sensorless_IF.Para.Breakaway_We_Ratio = IF_BREAKAWAY_WE_RATIO;
    Sensorless_IF.Para.Breakaway_Acc_Ratio = IF_BREAKAWAY_ACC_RATIO;
    Sensorless_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

    Flux_Obs.Para.Rs = Motor_Para.Rs;
    Flux_Obs.Para.Ls = Motor_Para.Ld;
    Flux_Obs.Para.Flux = Motor_Para.Flux;
    Flux_Obs.Para.BW_Hz = FLUX_OBS_BW_HZ;

    Flux_PLL.Para.Kp = PLL_KP;
    Flux_PLL.Para.Ki = PLL_KI;

    State = SL_ALIGN;
    return true;
}

void Sensorless_Stop(void)
{
    State = SL_IDLE;
}

bool Sensorless_Active(void)
{
    return (State != SL_IDLE) && (State != SL_FAILED);
}

void Sensorless_Control(float We_Ref, float Iq_Min, float Iq_Max)
{
    if ((State == SL_OBS) || ((State == SL_IF_TO_OBS) && (To_Obs_State == TO_OBS_I_TRANS)))
    {
        Obs_Iq_Ref = Speed_Loop(We_Ref, Flux_PLL.State.We, Iq_Min, Iq_Max);
    }
}

void Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Theta_e, float *Id_Ref, float *Iq_Ref)
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

        if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();

            Dir = (We_Ref < 0.0f) ? -1 : 1;
            We_IF_Target = IF_Target(We_Ref);
            Theta_Start = -(float)Dir * (0.5f * PI_F);

            IF_Init(&Sensorless_IF, Theta_Start, 0.0f);
            IF_Target_Set(&Sensorless_IF, We_IF_Target);

            Flux_Observer_Reset(&Flux_Obs, Theta_Start, Ialpha, Ibeta);
            PLL_Reset(&Flux_PLL, Theta_Start, 0.0f);
            Flux_Obs_U_Valid = false;

            State = SL_IF;
        }

        return;
    }

    if (Flux_Obs_U_Valid)
    {
        Flux_Observer_Run(&Flux_Obs, Motor_Run.Ualpha, Motor_Run.Ubeta, Ialpha, Ibeta, CUR_TS);
        PLL_Run(&Flux_PLL, Flux_Obs.State.PsiAlpha, Flux_Obs.State.PsiBeta, Flux_Obs.Para.Flux, CUR_TS);
    }

    if (State != SL_OBS)
    {
        IF_Target_Set(&Sensorless_IF, IF_Target(We_Ref));
        IF_Run(&Sensorless_IF, &Theta_IF, &Id_IF, &Iq_IF, CUR_TS);
    }

    Flux_Obs_U_Valid = true;
    Theta_Obs = Flux_PLL.State.Theta;

    if (State == SL_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (Abs_F(Sensorless_IF.State.We) >= IF_WE_TARGET_RAD_S)
        {
            Obs_Wait_Cnt = 0U;
            We_Obs_F = Flux_PLL.State.We;
            To_Obs_State = TO_OBS_WAIT;
            State = SL_IF_TO_OBS;
        }
    }
    else if (State == SL_IF_TO_OBS)
    {
        if (To_Obs_State == TO_OBS_WAIT)
        {
            Theta_Use = Theta_IF;
            *Id_Ref = Id_IF;
            *Iq_Ref = Iq_IF;

            if (Obs_Stable())
            {
                if (Obs_Wait_Cnt < OBS_WAIT_CNT)
                {
                    Obs_Wait_Cnt++;
                }

                if (Obs_Wait_Cnt >= OBS_WAIT_CNT)
                {
                    Handover_Blend_Reset(&Handover);
                    To_Obs_State = TO_OBS_BLEND;
                }
            }
            else
            {
                Obs_Wait_Cnt = 0U;
            }
        }
        else if (To_Obs_State == TO_OBS_BLEND)
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
                Speed_Loop_Track(We_Ref, Flux_PLL.State.We, Obs_Iq_Ref, -I_Max, I_Max);
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
                Initial_IF = false;
                State = SL_OBS;
            }
        }
    }
    else if (State == SL_OBS)
    {
        Theta_Use = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;

        if (Abs_F(Flux_PLL.State.We) <= OBS_TO_IF_WE_RAD_S)
        {
            IF_Init(&Sensorless_IF, Theta_Obs, Flux_PLL.State.We);
            IF_Target_Set(&Sensorless_IF, We_Ref);
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
