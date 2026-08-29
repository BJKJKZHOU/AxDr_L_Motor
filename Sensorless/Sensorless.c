/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Sensorless.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Fast_Profile.h"
#include "Flux_Observer.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Sin_LUT.h"
#include "control_params.h"
#include "main.h"

#define FLUX_OBS_BW_HZ    200.0f
#define PLL_BW_DEFAULT_HZ 50.0f
#define PLL_BW_MIN_HZ     1.0f
#define PLL_BW_MAX_HZ     200.0f
#define PLL_DAMP          0.707f

#define OBS_WAIT_S   0.20f
#define OBS_WAIT_CNT ((uint32_t)(OBS_WAIT_S / CUR_TS + 0.5f))
#define BLEND_S      0.15f
#define BLEND_CNT    ((uint32_t)(BLEND_S / CUR_TS + 0.5f))
#define ID_RAMP_A_S  5.0f
#define ID_RAMP_STEP (ID_RAMP_A_S * CUR_TS)

#define OBS_WE_TAU_S      0.020f
#define OBS_WE_ALPHA      (CUR_TS / (OBS_WE_TAU_S + CUR_TS))
#define FLUX_WE_TAU_S     0.020f
#define FLUX_WE_ALPHA     (CUR_TS / (FLUX_WE_TAU_S + CUR_TS))
#define WE_ERR_MAX        5.0f
#define PLL_ERR_MAX       0.08f
#define FLUX_MIN_RATIO2   0.64f
#define FLUX_MAX_RATIO2   1.44f

/* Temporary handover boundary. Replace with motor-dependent observer-quality
 * criteria after low-speed hardware characterization. */
#define OBS_TO_IF_WE_RAD_S (0.75f * IF_WE_TARGET_RAD_S)
#define SPD_DIV             10U

typedef enum
{
    TO_OBS_WAIT = 0,
    TO_OBS_BLEND,
    TO_OBS_I_TRANS,

} To_Obs_State_e;

static Sensorless_State_e State = SL_ALIGN;
static To_Obs_State_e To_Obs_State = TO_OBS_WAIT;
static volatile bool Active = false;
static bool Initial_IF = true;
static bool Shadow_Mode = false;
static float PLL_Bw_Hz = PLL_BW_DEFAULT_HZ;

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
static bool Flux_Theta_Valid = false;
static Motor_IF_Para_T IF_Para = { 0 };
static uint32_t Obs_Wait_Cnt = 0U;
static uint32_t Obs_Wait_Max_Cnt = 0U;
static uint32_t Blend_Cnt = 0U;
static uint32_t Speed_Div = 0U;
static float Obs_Id_Ref = 0.0f;
static float Obs_Iq_Ref = 0.0f;
static float Flux_Theta_Pre = 0.0f;
static Sensorless_Diag_T Obs_Diag = { 0 };
static Sensorless_Flux_Diag_T Flux_Diag = { 0 };

static float Angle_Diff(float A, float B)
{
    float Diff = A - B;

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

static void Obs_Diag_Reset(void)
{
    Obs_Diag = (Sensorless_Diag_T){ 0 };
    Flux_Diag = (Sensorless_Flux_Diag_T){ 0 };
    Flux_Theta_Pre = 0.0f;
    Flux_Theta_Valid = false;
    Obs_Wait_Max_Cnt = 0U;
}

static void Flux_Diag_Update(void)
{
    float Theta_Flux;
    float We_Raw;

    Theta_Flux = Angle_Wrap(__builtin_atan2f(Flux_Obs.State.PsiBeta, Flux_Obs.State.PsiAlpha));
    Flux_Diag.Theta_Flux = Theta_Flux;

    if (!Flux_Theta_Valid)
    {
        Flux_Theta_Pre = Theta_Flux;
        Flux_Diag.We_Flux_Raw = 0.0f;
        Flux_Diag.We_Flux_F = 0.0f;
        Flux_Theta_Valid = true;
        return;
    }

    We_Raw = Angle_Diff(Theta_Flux, Flux_Theta_Pre) / CUR_TS;
    Flux_Theta_Pre = Theta_Flux;
    Flux_Diag.We_Flux_Raw = We_Raw;
    Flux_Diag.We_Flux_F += FLUX_WE_ALPHA * (We_Raw - Flux_Diag.We_Flux_F);
}

static float Current_Limit_Get(void)
{
    float I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    return (I_Max > 0.0f) ? I_Max : 0.0f;
}

static bool Obs_Stable(void)
{
    float Flux2;
    float Flux_Ref2;

    Sensorless_We_Obs_F += OBS_WE_ALPHA * (Flux_PLL.State.We - Sensorless_We_Obs_F);

    Obs_Diag.We_IF = IF_Start_We_Get();
    Obs_Diag.We_Obs_F = Sensorless_We_Obs_F;
    Obs_Diag.We_Err = Obs_Diag.We_Obs_F - Obs_Diag.We_IF;
    Obs_Diag.PLL_Err = Flux_PLL.State.Err;
    Obs_Diag.Theta_Err = Angle_Diff(Flux_PLL.State.Theta, Sensorless_Theta_IF);
    Flux_Diag.Theta_Flux_IF_Err = Angle_Diff(Flux_Diag.Theta_Flux, Sensorless_Theta_IF);
    Obs_Diag.Reject = SENSORLESS_REJECT_NONE;

    Flux2 = Flux_Obs.State.PsiAlpha * Flux_Obs.State.PsiAlpha + Flux_Obs.State.PsiBeta * Flux_Obs.State.PsiBeta;
    Flux_Ref2 = Flux_Obs.Para.Flux * Flux_Obs.Para.Flux;

    if (Flux_Ref2 > 0.0f)
    {
        Obs_Diag.Flux_Ratio = __builtin_sqrtf(Flux2 / Flux_Ref2);
    }
    else
    {
        Obs_Diag.Flux_Ratio = 0.0f;
        Obs_Diag.Reject |= SENSORLESS_REJECT_FLUX;
    }

    if (Abs_F(Obs_Diag.We_Err) > WE_ERR_MAX)
    {
        Obs_Diag.Reject |= SENSORLESS_REJECT_WE;
    }
    if (Abs_F(Obs_Diag.PLL_Err) > PLL_ERR_MAX)
    {
        Obs_Diag.Reject |= SENSORLESS_REJECT_PLL;
    }
    if ((Flux_Ref2 > 0.0f) && ((Flux2 < FLUX_MIN_RATIO2 * Flux_Ref2) || (Flux2 > FLUX_MAX_RATIO2 * Flux_Ref2)))
    {
        Obs_Diag.Reject |= SENSORLESS_REJECT_FLUX;
    }

    Obs_Diag.Reject_Seen |= Obs_Diag.Reject;
    return Obs_Diag.Reject == SENSORLESS_REJECT_NONE;
}

static void DQ_Rotate(float Theta_IF, float Theta_Use, float Id_IF, float Iq_IF, float *Id_Ref, float *Iq_Ref)
{
    float Sin;
    float Cos;
    float Diff = Angle_Diff(Theta_IF, Theta_Use);

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
    float I_Max;
    float Err;
    float Int;

    I_Max = Current_Limit_Get();
    Speed_Ctrl.Para.Out_Min = -I_Max;
    Speed_Ctrl.Para.Out_Max = I_Max;
    Speed_Ctrl.Para.Int_Min = -I_Max;
    Speed_Ctrl.Para.Int_Max = I_Max;

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
    float I_Max;

    if (Speed_Div == 0U)
    {
        I_Max = Current_Limit_Get();
        Obs_Iq_Ref = Speed_Loop(We_Ref, Flux_PLL.State.We, -I_Max, I_Max);
    }

    Speed_Div++;
    if (Speed_Div >= SPD_DIV)
    {
        Speed_Div = 0U;
    }
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
    float PLL_Wn;

    State = SL_ALIGN;
    To_Obs_State = TO_OBS_WAIT;
    Active = false;
    Initial_IF = true;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    IF_Para = (Motor_IF_Para_T){ 0 };
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
    Obs_Diag_Reset();

    Align_Reset();
    Current_Loop_State_Reset();

    if (!Motor_IF_Para_Build(ADC.Vbus_V, Current_Limit_Get(), &IF_Para))
    {
        State = SL_FAILED;
        return false;
    }

    PLL_Wn = TWO_PI_F * PLL_Bw_Hz;
    Flux_PLL.Para.Kp = 2.0f * PLL_DAMP * PLL_Wn;
    Flux_PLL.Para.Ki = PLL_Wn * PLL_Wn;

    Active = true;
    return true;
}

void Sensorless_Stop(void)
{
    Active = false;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Speed_Div = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Sensorless_Blend = 0.0f;
}

bool Sensorless_Active(void)
{
    return Active;
}

bool Sensorless_Ready(void)
{
    return Active && !Initial_IF;
}

bool Sensorless_Shadow_Set(bool Enable)
{
    if (Active)
    {
        return false;
    }

    Shadow_Mode = Enable;
    return true;
}

bool Sensorless_Shadow_Get(void)
{
    return Shadow_Mode;
}

bool Sensorless_PLL_BW_Set(float Bw_Hz)
{
    if (Active || !__builtin_isfinite(Bw_Hz) || (Bw_Hz < PLL_BW_MIN_HZ) || (Bw_Hz > PLL_BW_MAX_HZ))
    {
        return false;
    }

    PLL_Bw_Hz = Bw_Hz;
    return true;
}

float Sensorless_PLL_BW_Get(void)
{
    return PLL_Bw_Hz;
}

bool Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Theta_e, float *Id_Ref, float *Iq_Ref)
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

    if (!Active)
    {
        *Theta_e = Sensorless_Theta_Use;
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

    if (State == SL_ALIGN)
    {
        *Theta_e = 0.0f;
        Sensorless_Theta_Use = 0.0f;

        if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            Dir = (We_Ref < 0.0f) ? -1 : 1;
            We_IF_Target = IF_Target(We_Ref);
            Theta_Start = -(float)Dir * (0.5f * PI_F);

            IF_Start_Reset(Theta_Start, 0.0f);
            IF_Start_Para_Set(IF_Para.Iq_Start_A, IF_Para.Iq_Max_A, IF_Para.We_Base, IF_Para.Acc);
            IF_Start_Target_Set(We_IF_Target);

            Flux_Obs.Para.Rs = Motor_Para.Rs;
            Flux_Obs.Para.Ls = Motor_Para.Ld;
            Flux_Obs.Para.Flux = Motor_Para.Flux;
            Flux_Obs.Para.Gamma = (Motor_Para.Flux > 0.0f)
                                      ? TWO_PI_F * FLUX_OBS_BW_HZ / (Motor_Para.Flux * Motor_Para.Flux)
                                      : 0.0f;

            Flux_Observer_Reset(&Flux_Obs, Theta_Start, Ialpha, Ibeta);
            PLL_Reset(&Flux_PLL, Theta_Start, 0.0f);
            Flux_Obs_U_Valid = false;
            Flux_Theta_Valid = false;
            State = SL_IF;
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
        Flux_Diag_Update();
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

    if (State != SL_OBS)
    {
        IF_Start_Target_Set(IF_Target(We_Ref));
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

    if (State == SL_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;
        Sensorless_Blend = 0.0f;

        if (Abs_F(IF_Start_We_Get()) >= IF_WE_TARGET_RAD_S)
        {
            Obs_Wait_Cnt = 0U;
            Sensorless_We_Obs_F = Flux_PLL.State.We;
            Obs_Diag_Reset();
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
                if (Obs_Wait_Cnt > Obs_Wait_Max_Cnt)
                {
                    Obs_Wait_Max_Cnt = Obs_Wait_Cnt;
                }

                Obs_Diag.Stable_s = (float)Obs_Wait_Cnt * CUR_TS;
                Obs_Diag.Stable_Max_s = (float)Obs_Wait_Max_Cnt * CUR_TS;

                if ((Obs_Wait_Cnt >= OBS_WAIT_CNT) && !Shadow_Mode)
                {
                    Blend_Cnt = 0U;
                    Sensorless_Blend = 0.0f;
                    To_Obs_State = TO_OBS_BLEND;
                }
            }
            else
            {
                Obs_Wait_Cnt = 0U;
                Obs_Diag.Stable_s = 0.0f;
                Obs_Diag.Stable_Max_s = (float)Obs_Wait_Max_Cnt * CUR_TS;
            }
        }
        else if (To_Obs_State == TO_OBS_BLEND)
        {
            Blend = (BLEND_CNT > 0U) ? (float)(Blend_Cnt + 1U) / (float)BLEND_CNT : 1.0f;
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
                Initial_IF = false;
                State = SL_OBS;
            }
        }
    }
    else if (State == SL_OBS)
    {
        Theta_Use = Theta_Obs;
        Speed_Run(We_Ref);
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        Sensorless_Blend = 1.0f;

        if (Abs_F(Flux_PLL.State.We) <= OBS_TO_IF_WE_RAD_S)
        {
            IF_Start_Reset(Theta_Obs, Flux_PLL.State.We);
            IF_Start_Para_Set(IF_Para.Iq_Start_A, IF_Para.Iq_Max_A, IF_Para.We_Base, IF_Para.Acc);
            IF_Start_Target_Set(We_Ref);
            Sensorless_Theta_IF = Theta_Obs;
            Blend_Cnt = 0U;
            State = SL_OBS_TO_IF;
        }
    }
    else
    {
        Blend = (BLEND_CNT > 0U) ? (float)(Blend_Cnt + 1U) / (float)BLEND_CNT : 1.0f;
        if (Blend > 1.0f)
        {
            Blend = 1.0f;
        }

        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Obs_Iq_Ref + Blend * (Iq_IF - Obs_Iq_Ref);
        Sensorless_Blend = 1.0f - Blend;

        if (Blend_Cnt < BLEND_CNT)
        {
            Blend_Cnt++;
        }
        if (Blend_Cnt >= BLEND_CNT)
        {
            Sensorless_Blend = 0.0f;
            State = SL_IF;
        }
    }

    *Theta_e = Theta_Use;
    Sensorless_Theta_Use = Theta_Use;
    Sensorless_Id_Ref = *Id_Ref;
    Sensorless_Iq_Ref = *Iq_Ref;
    return Sensorless_Ready();
}

Sensorless_State_e Sensorless_State_Get(void)
{
    return State;
}

const Sensorless_Diag_T *Sensorless_Diag_Get(void)
{
    return &Obs_Diag;
}

const Sensorless_Flux_Diag_T *Sensorless_Flux_Diag_Get(void)
{
    return &Flux_Diag;
}
