/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Start.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Fast_Profile.h"
#include "Flux_Observer.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Sin_LUT.h"
#include "control_params.h"
#include "main.h"

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

#define SPD_DIV 10U
#define IQ_MAX  2.0f

typedef enum
{
    TO_OBS_WAIT = 0,
    TO_OBS_BLEND,
    TO_OBS_I_TRANS,

} To_Obs_State_e;

static Sensorless_State_e Start_State = SL_ALIGN;
static To_Obs_State_e To_Obs_State = TO_OBS_WAIT;
static volatile bool Start_Active = false;

Flux_Observer_T Flux_Obs = { 0 };
PLL_T Flux_PLL = { 0 };

volatile float Sensorless_Theta_IF = 0.0f;
volatile float Sensorless_Theta_Use = 0.0f;
volatile float Sensorless_Id_Ref = 0.0f;
volatile float Sensorless_Iq_Ref = 0.0f;
volatile float Sensorless_Blend = 0.0f;
volatile float Sensorless_We_Obs_F = 0.0f;

static bool Flux_Obs_U_Valid = false;
static bool Profile_Requested = false;
static uint32_t Obs_Wait_Cnt = 0U;
static uint32_t Blend_Cnt = 0U;
static uint32_t Speed_Div = 0U;
static float Obs_Id_Ref = 0.0f;
static float Obs_Iq_Ref = 0.0f;

static float Angle_Diff(float A, float B)
{
    float Diff;

    Diff = A - B;

    while (Diff > PI_F)
    {
        Diff -= TWO_PI_F;
    }

    while (Diff < -PI_F)
    {
        Diff += TWO_PI_F;
    }

    return Diff;
}

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static bool Obs_Stable(void)
{
    float We_Err;
    float Flux2;
    float Flux_Ref2;

    Sensorless_We_Obs_F += OBS_WE_ALPHA * (Flux_PLL.State.We - Sensorless_We_Obs_F);
    We_Err = Sensorless_We_Obs_F - IF_Start_We_Get();
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

static void DQ_Rotate(float Theta_IF, float Theta_Use, float Id_IF, float Iq_IF, float *Id_Ref, float *Iq_Ref)
{
    float Diff;
    float Sin;
    float Cos;

    Diff = Angle_Diff(Theta_IF, Theta_Use);
    SinCos(Angle_Wrap(Diff), &Sin, &Cos);

    *Id_Ref = Id_IF * Cos - Iq_IF * Sin;
    *Iq_Ref = Id_IF * Sin + Iq_IF * Cos;
}

static float Ramp_Zero(float X, float Step)
{
    if (X > Step)
    {
        return X - Step;
    }

    if (X < -Step)
    {
        return X + Step;
    }

    return 0.0f;
}

static void Speed_Track(float We_Ref, float Iq)
{
    float Err;
    float Int;

    Speed_Ctrl.Para.Out_Min = -IQ_MAX;
    Speed_Ctrl.Para.Out_Max = IQ_MAX;
    Speed_Ctrl.Para.Int_Min = -IQ_MAX;
    Speed_Ctrl.Para.Int_Max = IQ_MAX;

    Speed_Ctrl.Sig.Ref = We_Ref;
    Speed_Ctrl.Sig.Fbk = Flux_PLL.State.We;
    Err = Speed_Ctrl.Sig.Ref - Speed_Ctrl.Sig.Fbk;
    Int = Iq - Speed_Ctrl.Para.Kp * Err;
    Limit_Value(&Int, Speed_Ctrl.Para.Int_Min, Speed_Ctrl.Para.Int_Max);

    Speed_Ctrl.Sig.Err = Err;
    Speed_Ctrl.Sig.Out = Iq;
    Speed_Ctrl.State.Int = Int;
    Speed_Ctrl.State.Fbk_Pre = Speed_Ctrl.Sig.Fbk;
    Speed_Div = 0U;
}

static void Speed_Run(float We_Ref)
{
    if (Speed_Div == 0U)
    {
        Obs_Iq_Ref = Speed_Loop(We_Ref, Flux_PLL.State.We, -IQ_MAX, IQ_MAX);
    }

    Speed_Div++;

    if (Speed_Div >= SPD_DIV)
    {
        Speed_Div = 0U;
    }
}

void Sensorless_Start_Begin(void)
{
    Start_State = SL_ALIGN;
    To_Obs_State = TO_OBS_WAIT;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Speed_Div = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;

    Sensorless_Theta_IF = 0.0f;
    Sensorless_Theta_Use = 0.0f;
    Sensorless_Id_Ref = 0.0f;
    Sensorless_Iq_Ref = 0.0f;
    Sensorless_Blend = 0.0f;
    Sensorless_We_Obs_F = 0.0f;

    Align_Reset();
    Current_Loop_State_Reset();
    Start_Active = true;
}

void Sensorless_Start_Stop(void)
{
    Start_Active = false;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Speed_Div = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Sensorless_Blend = 0.0f;
}

bool Sensorless_Start_Active(void)
{
    return Start_Active;
}

bool Sensorless_Start_Ready(void)
{
    return Start_Active && (Start_State == SL_OBS);
}

bool Sensorless_Start_Run(float Ia_A, float Ib_A, float We_Ref, float *Id_Ref, float *Iq_Ref)
{
    float Ialpha;
    float Ibeta;
    float Theta_Start;
    float Theta_IF;
    float Theta_Obs;
    float Theta_Err;
    float Theta_Use;
    float Id_IF;
    float Iq_IF;
    float Blend;
    float We_IF_Target;
    int8_t Dir;
    bool IF_Ready;
    bool Profile_Run;
    uint32_t T0;

    if (!Start_Active)
    {
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return false;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Theta_IF = Sensorless_Theta_IF;
    Id_IF = 0.0f;
    Iq_IF = 0.0f;
    IF_Ready = false;

    if (Start_State == SL_ALIGN)
    {
        Motor_Run.Theta_e = 0.0f;

        if (Align_Current(IF_ALIGN_ID_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();

            Dir = (We_Ref < 0.0f) ? -1 : 1;
            We_IF_Target = Abs_F(We_Ref);
            if (We_IF_Target < IF_WE_TARGET_RAD_S)
            {
                We_IF_Target = IF_WE_TARGET_RAD_S;
            }
            We_IF_Target *= (float)Dir;

            Theta_Start = -(float)Dir * (0.5f * PI_F);
            IF_Start_Reset(Theta_Start, 0.0f);
            IF_Start_Target_Set(We_IF_Target);

            Flux_Obs.Para.Rs = Motor_Para.Rs;
            Flux_Obs.Para.Ls = Motor_Para.Ld;
            Flux_Obs.Para.Flux = Motor_Para.Flux;

            if (Motor_Para.Flux > 0.0f)
            {
                Flux_Obs.Para.Gamma = TWO_PI_F * FLUX_OBS_BW_HZ / (Motor_Para.Flux * Motor_Para.Flux);
            }
            else
            {
                Flux_Obs.Para.Gamma = 0.0f;
            }

            Flux_PLL.Para.Kp = PLL_KP;
            Flux_PLL.Para.Ki = PLL_KI;

            Flux_Observer_Reset(&Flux_Obs, Theta_Start, Ialpha, Ibeta);
            PLL_Reset(&Flux_PLL, Theta_Start, 0.0f);
            Flux_Obs_U_Valid = false;

            Start_State = SL_IF;
        }

        return false;
    }

    Profile_Run = (Fast_Profile.Run != 0U);

    if (Flux_Obs_U_Valid)
    {
        if (Profile_Run)
        {
            T0 = DWT->CYCCNT;
        }

        Flux_Observer_Run(&Flux_Obs, Motor_Run.Ualpha, Motor_Run.Ubeta, Ialpha, Ibeta, CUR_TS);

        if (Profile_Run)
        {
            Fast_Profile_Add(&Fast_Profile.Flux_Observer, DWT->CYCCNT - T0);
            T0 = DWT->CYCCNT;
        }

        PLL_Run(&Flux_PLL, Flux_Obs.State.PsiAlpha, Flux_Obs.State.PsiBeta, Flux_Obs.Para.Flux, CUR_TS);

        if (Profile_Run)
        {
            Fast_Profile_Add(&Fast_Profile.PLL, DWT->CYCCNT - T0);
        }
    }

    if (Start_State != SL_OBS)
    {
        if (Profile_Run)
        {
            T0 = DWT->CYCCNT;
        }

        IF_Ready = IF_Start_Run(&Theta_IF, &Id_IF, &Iq_IF);

        if (Profile_Run)
        {
            Fast_Profile_Add(&Fast_Profile.IF_Start, DWT->CYCCNT - T0);
        }

        Sensorless_Theta_IF = Theta_IF;
    }

    Flux_Obs_U_Valid = true;
    Theta_Obs = Flux_PLL.State.Theta;

    if (IF_Ready && !Profile_Requested)
    {
        Fast_Profile_Request();
        Profile_Requested = true;
    }

    if (Start_State == SL_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (IF_Ready)
        {
            Obs_Wait_Cnt = 0U;
            Sensorless_We_Obs_F = Flux_PLL.State.We;
            To_Obs_State = TO_OBS_WAIT;
            Start_State = SL_IF_TO_OBS;
        }
    }
    else if (Start_State == SL_IF_TO_OBS)
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
                    Blend_Cnt = 0U;
                    Sensorless_Blend = 0.0f;
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

            Theta_Err = Angle_Diff(Theta_Obs, Theta_IF);
            Theta_Use = Angle_Wrap(Theta_IF + Blend * Theta_Err);

            DQ_Rotate(Theta_IF, Theta_Use, Id_IF, Iq_IF, Id_Ref, Iq_Ref);
            Sensorless_Blend = Blend;

            if (Blend_Cnt < BLEND_CNT)
            {
                Blend_Cnt++;
            }

            if (Blend_Cnt >= BLEND_CNT)
            {
                Obs_Id_Ref = *Id_Ref;
                Obs_Iq_Ref = *Iq_Ref;
                Speed_Track(We_Ref, Obs_Iq_Ref);
                Sensorless_Blend = 1.0f;
                To_Obs_State = TO_OBS_I_TRANS;
            }
        }
        else
        {
            Theta_Use = Theta_Obs;
            Speed_Run(We_Ref);
            Obs_Id_Ref = Ramp_Zero(Obs_Id_Ref, ID_RAMP_STEP);
            *Id_Ref = Obs_Id_Ref;
            *Iq_Ref = Obs_Iq_Ref;
            Sensorless_Blend = 1.0f;

            if (Obs_Id_Ref == 0.0f)
            {
                Start_State = SL_OBS;
            }
        }
    }
    else
    {
        Theta_Use = Theta_Obs;
        Speed_Run(We_Ref);
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        Sensorless_Blend = 1.0f;
    }

    Motor_Run.Theta_e = Theta_Use;

    Sensorless_Theta_Use = Theta_Use;
    Sensorless_Id_Ref = *Id_Ref;
    Sensorless_Iq_Ref = *Iq_Ref;

    return Sensorless_Start_Ready();
}

Sensorless_State_e Sensorless_Start_State_Get(void)
{
    return Start_State;
}
