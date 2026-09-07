/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

#include <stddef.h>
#include <stdint.h>

#include "Align.h"
#include "Current_Loop.h"
#include "Flux_Estimator.h"
#include "Flux_Observer.h"
#include "IF_Start.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Control.h"
#include "Motion_Loop.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Sensorless.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define FLUX_IF_ACCEL_S          6.0f
#define FLUX_COARSE_I_BW_HZ      200.0f
#define FLUX_COARSE_BW_HZ        20.0f
#define FLUX_COARSE_WE_MIN_RATIO 0.25f
#define FLUX_EST_NUM_MIN_WB      1.0e-12f
#define FLUX_FINE_SETTLE_S       0.30f
#define FLUX_FINE_MEASURE_S      0.20f
#define FLUX_FINE_SETTLE_CNT     ((uint32_t)(FLUX_FINE_SETTLE_S / CUR_TS + 0.5f))
#define FLUX_FINE_MEASURE_CNT    ((uint32_t)(FLUX_FINE_MEASURE_S / CUR_TS + 0.5f))
#define FLUX_FINE_I_BW_HZ        200.0f
#define FLUX_FINE_EST_BW_HZ      5.0f
#define FLUX_FINE_STABLE_RATIO   0.02f
#define FLUX_FINE_STABLE_WINDOWS 2U
#define FLUX_FINE_UPDATE_RATIO   0.25f
#define FLUX_FINISH_IQ_SLEW_A_S  20.0f
#define FLUX_FINISH_S            0.100f
#define FLUX_FINISH_CNT          ((uint32_t)(FLUX_FINISH_S / CUR_TS + 0.5f))

#define FLUX_OBS_EMF_ENTER_RATIO 0.30f
#define FLUX_OBS_EMF_EXIT_RATIO  0.18f
#define FLUX_OBS_PLL_START_RATIO 0.10f
#define FLUX_MOTION_LOST_RATIO   0.05f
#define FLUX_MOTION_LOST_S       0.15f
#define FLUX_MOTION_LOST_CNT     ((uint32_t)(FLUX_MOTION_LOST_S / CUR_TS + 0.5f))
#define FLUX_U_SEARCH_RATIO      0.90f

#define FLUX_OBS_BW_HZ     200.0f
#define FLUX_OBS_GAMMA_MAX 1.0e12f
#define FLUX_PLL_BW_HZ     50.0f
#define FLUX_PLL_DAMP      0.707f
#define FLUX_PLL_WN        (TWO_PI_F * FLUX_PLL_BW_HZ)
#define FLUX_PLL_KP        (2.0f * FLUX_PLL_DAMP * FLUX_PLL_WN)
#define FLUX_PLL_KI        (FLUX_PLL_WN * FLUX_PLL_WN)

#define FLUX_OBS_WAIT_S         0.20f
#define FLUX_OBS_WAIT_CNT       ((uint32_t)(FLUX_OBS_WAIT_S / CUR_TS + 0.5f))
#define FLUX_BLEND_S            0.15f
#define FLUX_BLEND_CNT          ((uint32_t)(FLUX_BLEND_S / CUR_TS + 0.5f))
#define FLUX_ID_RAMP_A_S        5.0f
#define FLUX_ID_RAMP_STEP       (FLUX_ID_RAMP_A_S * CUR_TS)
#define FLUX_OBS_WE_TAU_S       0.020f
#define FLUX_OBS_WE_ALPHA       (CUR_TS / (FLUX_OBS_WE_TAU_S + CUR_TS))
#define FLUX_EMF_TAU_S          0.020f
#define FLUX_EMF_ALPHA          (CUR_TS / (FLUX_EMF_TAU_S + CUR_TS))
#define FLUX_OBS_CMP_TAU_S      0.050f
#define FLUX_OBS_CMP_ALPHA      (CUR_TS / (FLUX_OBS_CMP_TAU_S + CUR_TS))
#define FLUX_OBS_WE_MEAN_RATIO  0.010f
#define FLUX_OBS_WE_RMS_RATIO   0.015f
#define FLUX_OBS_PLL_RMS_MAX    0.08f
#define FLUX_OBS_THETA_RMS_MAX  0.08f
#define FLUX_TARGET_TAU_S       0.10f
#define FLUX_TARGET_ALPHA       (CUR_TS / (FLUX_TARGET_TAU_S + CUR_TS))

typedef enum
{
    FLUX_IDLE = 0,
    FLUX_ALIGN,
    FLUX_IF,
    FLUX_OBS_WAIT,
    FLUX_OBS_BLEND,
    FLUX_OBS_I_TRANS,
    FLUX_FINE_SETTLE,
    FLUX_FINE_MEASURE,
    FLUX_FINISH,
    FLUX_DONE,
    FLUX_FAILED,

} Flux_State_e;

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T IF_Para = { 0 };
static Flux_Estimator_T Flux_Estimator = { 0 };
static volatile float We_Target = 0.0f;
static uint32_t Cnt = 0U;
static float We_Obs_F = 0.0f;
static float Emf_Ratio_F = 0.0f;
static float Obs_We_Err_F = 0.0f;
static float Obs_We_Err2_F = 0.0f;
static float Obs_PLL_Err2_F = 0.0f;
static float Obs_Theta_Err_F = 0.0f;
static float Obs_Theta_Err2_F = 0.0f;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Fine_Num = 0.0f;
static float Fine_Den = 0.0f;
static float Fine_Flux_Pre = 0.0f;
static float Finish_Iq = 0.0f;
static uint32_t Obs_Wait_Cnt = 0U;
static uint32_t Motion_Lost_Cnt = 0U;
static uint32_t Blend_Cnt = 0U;
static uint32_t Fine_Cnt = 0U;
static uint32_t Fine_Stable_Cnt = 0U;
static uint8_t Finish_Init = 0U;
static bool Model_U_Valid = false;
static bool Emf_Valid = false;
static bool Obs_Speed_Cmp_Valid = false;
static bool Obs_Theta_Cmp_Valid = false;
static bool Obs_Active = false;
static bool PLL_Active = false;
static bool Obs_Control = false;
static bool Emf_Target_Reached = false;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

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

static bool Emf_Update(float Yd, float Yq)
{
    const Ident_Envelope_T *Envelope;
    float Ratio;

    Envelope = Identification_Envelope_Get();
    if ((Envelope->U_Available <= 0.0f) || !__builtin_isfinite(Yd) || !__builtin_isfinite(Yq))
    {
        return false;
    }

    Ratio = __builtin_sqrtf(Yd * Yd + Yq * Yq) / Envelope->U_Available;
    if (!__builtin_isfinite(Ratio))
    {
        return false;
    }

    if (!Emf_Valid)
    {
        Emf_Ratio_F = Ratio;
        Emf_Valid = true;
    }
    else
    {
        Emf_Ratio_F += FLUX_EMF_ALPHA * (Ratio - Emf_Ratio_F);
    }
    return true;
}

static void Obs_Para_Update(void)
{
    float Flux;
    float Gamma;

    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= FLUX_EST_NUM_MIN_WB))
    {
        return;
    }

    Flux_Obs.Para.Flux = Flux;
    Gamma = TWO_PI_F * FLUX_OBS_BW_HZ / (Flux * Flux);
    if (!__builtin_isfinite(Gamma) || (Gamma > FLUX_OBS_GAMMA_MAX))
    {
        Gamma = FLUX_OBS_GAMMA_MAX;
    }
    Flux_Obs.Para.Gamma = Gamma;
}

static void Obs_Compare_Reset(void)
{
    Obs_We_Err_F = 0.0f;
    Obs_We_Err2_F = 0.0f;
    Obs_PLL_Err2_F = 0.0f;
    Obs_Theta_Err_F = 0.0f;
    Obs_Theta_Err2_F = 0.0f;
    Obs_Speed_Cmp_Valid = false;
    Obs_Theta_Cmp_Valid = false;
}

static void Qualification_Accumulate(uint32_t *Count, uint32_t Limit, bool Good)
{
    if (Good)
    {
        if (*Count < Limit)
        {
            (*Count)++;
        }
    }
    else if (*Count > 0U)
    {
        (*Count)--;
    }
}

static void Obs_Speed_Compare_Run(float We_Ref)
{
    float We_Err;
    float PLL_Err2;

    We_Err = We_Obs_F - We_Ref;
    PLL_Err2 = Flux_PLL.State.Err * Flux_PLL.State.Err;
    if (!Obs_Speed_Cmp_Valid)
    {
        Obs_We_Err_F = We_Err;
        Obs_We_Err2_F = We_Err * We_Err;
        Obs_PLL_Err2_F = PLL_Err2;
        Obs_Speed_Cmp_Valid = true;
        return;
    }

    Obs_We_Err_F += FLUX_OBS_CMP_ALPHA * (We_Err - Obs_We_Err_F);
    Obs_We_Err2_F += FLUX_OBS_CMP_ALPHA * (We_Err * We_Err - Obs_We_Err2_F);
    Obs_PLL_Err2_F += FLUX_OBS_CMP_ALPHA * (PLL_Err2 - Obs_PLL_Err2_F);
}

static void Obs_IF_Compare_Run(float Theta_IF)
{
    float Theta_Err;
    float Theta_Ripple;

    Obs_Speed_Compare_Run(IF_Start_We_Get());
    Theta_Err = Angle_Diff(Flux_PLL.State.Theta, Theta_IF);
    if (!Obs_Theta_Cmp_Valid)
    {
        Obs_Theta_Err_F = Angle_Wrap(Theta_Err);
        Obs_Theta_Err2_F = 0.0f;
        Obs_Theta_Cmp_Valid = true;
        return;
    }

    Obs_Theta_Err_F = Angle_Wrap(Obs_Theta_Err_F + FLUX_OBS_CMP_ALPHA * Angle_Diff(Theta_Err, Obs_Theta_Err_F));
    Theta_Ripple = Angle_Diff(Theta_Err, Obs_Theta_Err_F);
    Obs_Theta_Err2_F += FLUX_OBS_CMP_ALPHA * (Theta_Ripple * Theta_Ripple - Obs_Theta_Err2_F);
}

static bool Obs_State_Stable(void)
{
    return __builtin_isfinite(We_Obs_F) && __builtin_isfinite(Flux_PLL.State.Err);
}

static bool Obs_Speed_Stable(float We_Ref)
{
    float We_Scale;

    if (!Obs_Speed_Cmp_Valid || !__builtin_isfinite(Obs_We_Err_F) || !__builtin_isfinite(Obs_We_Err2_F) ||
        !__builtin_isfinite(Obs_PLL_Err2_F) || (We_Ref * We_Obs_F <= 0.0f))
    {
        return false;
    }

    We_Scale = Abs_Value(We_Ref);
    if (We_Scale < IF_Para.We_Base)
    {
        We_Scale = IF_Para.We_Base;
    }

    return (Abs_Value(Obs_We_Err_F) <= FLUX_OBS_WE_MEAN_RATIO * We_Scale) &&
           (Obs_We_Err2_F <= FLUX_OBS_WE_RMS_RATIO * FLUX_OBS_WE_RMS_RATIO * We_Scale * We_Scale) &&
           (Obs_PLL_Err2_F <= FLUX_OBS_PLL_RMS_MAX * FLUX_OBS_PLL_RMS_MAX);
}

static bool Obs_IF_Stable(void)
{
    return Obs_State_Stable() && Obs_Speed_Stable(IF_Start_We_Get()) && Obs_Theta_Cmp_Valid &&
           __builtin_isfinite(Obs_Theta_Err2_F) &&
           (Obs_Theta_Err2_F <= FLUX_OBS_THETA_RMS_MAX * FLUX_OBS_THETA_RMS_MAX);
}

static bool Obs_Control_Stable(void)
{
    return Obs_State_Stable() && Obs_Speed_Stable(We_Target);
}

static bool Obs_Run_Valid(void)
{
    return Emf_Valid && __builtin_isfinite(Emf_Ratio_F) && (Emf_Ratio_F >= FLUX_MOTION_LOST_RATIO) &&
           Obs_State_Stable() && (We_Target * We_Obs_F > 0.0f);
}

static bool Motion_Lost_Run(bool Motion_Valid)
{
    if (!PLL_Active || Motion_Valid)
    {
        Motion_Lost_Cnt = 0U;
        return false;
    }
    if (Motion_Lost_Cnt < FLUX_MOTION_LOST_CNT)
    {
        Motion_Lost_Cnt++;
    }
    return Motion_Lost_Cnt >= FLUX_MOTION_LOST_CNT;
}

static float Ramp_Zero(float Value, float Step)
{
    if (Value > Step)
    {
        return Value - Step;
    }
    if (Value < -Step)
    {
        return Value + Step;
    }
    return 0.0f;
}

static bool Coarse_Run(void)
{
    float We;

    We = IF_Start_We_Get();
    if (!Flux_Estimator_Model_Run(&Flux_Estimator,
                                  Motor_Run.Ud,
                                  Motor_Run.Uq,
                                  Motor_Run.Id,
                                  Motor_Run.Iq,
                                  We,
                                  CUR_TS))
    {
        return false;
    }

    if (!Emf_Update(Flux_Estimator.State.Yd, Flux_Estimator.State.Yq))
    {
        return false;
    }

    return Flux_Estimator_Vector_Update(&Flux_Estimator, We, CUR_TS);
}

static void IF_Target_Update(int8_t Dir)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Abs;
    float U_Search_Max;
    float U_Mag;
    float We_U_Max;
    float We_Req;
    float Flux;

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if ((Envelope->U_Available <= 0.0f) || (Envelope->U_Max <= 0.0f) || !__builtin_isfinite(We_Max) || (We_Max <= 0.0f))
    {
        return;
    }

    We_Abs = Abs_Value(IF_Start_We_Get());
    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= FLUX_EST_NUM_MIN_WB))
    {
        We_Target = (float)Dir * We_Abs;
        IF_Start_Target_Set(We_Target);
        return;
    }

    We_Req = FLUX_OBS_EMF_ENTER_RATIO * Envelope->U_Available / Flux;
    U_Mag = __builtin_sqrtf(Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq);
    U_Search_Max = FLUX_U_SEARCH_RATIO * Envelope->U_Max;
    if ((We_Abs > 0.0f) && (U_Mag > 0.0f))
    {
        We_U_Max = We_Abs * U_Search_Max / U_Mag;
        if (We_Req > We_U_Max)
        {
            We_Req = We_U_Max;
        }
    }
    if (We_Req < IF_Para.We_Base)
    {
        We_Req = IF_Para.We_Base;
    }
    if (We_Req > We_Max)
    {
        We_Req = We_Max;
    }

    We_Abs = Abs_Value(We_Target);
    We_Abs += FLUX_TARGET_ALPHA * (We_Req - We_Abs);
    IF_Para.Acc = We_Abs / FLUX_IF_ACCEL_S;
    We_Target = (float)Dir * We_Abs;
    IF_Start_Para_Set(IF_Para.Iq_Start_A, IF_Para.Iq_Max_A, IF_Para.We_Base, IF_Para.Acc);
    IF_Start_Target_Set(We_Target);
}

static void Fine_Begin(void)
{
    Flux_Estimator.Para.I_BW_Hz = FLUX_FINE_I_BW_HZ;
    Flux_Estimator.Para.Est_BW_Hz = FLUX_FINE_EST_BW_HZ;
    Flux_Estimator_Current_Reset(&Flux_Estimator);
    Fine_Num = 0.0f;
    Fine_Den = 0.0f;
    Fine_Cnt = 0U;
}

static void Fine_Window_Reset(void)
{
    Fine_Num = 0.0f;
    Fine_Den = 0.0f;
    Fine_Cnt = 0U;
}

static bool Fine_Run(bool Adapt, bool Measure)
{
    float We;

    We = We_Obs_F;
    if (!Flux_Estimator_Model_Run(&Flux_Estimator,
                                  Motor_Run.Ud,
                                  Motor_Run.Uq,
                                  Motor_Run.Id,
                                  Motor_Run.Iq,
                                  We,
                                  CUR_TS))
    {
        return false;
    }

    if (!Emf_Update(Flux_Estimator.State.Yd, Flux_Estimator.State.Yq))
    {
        return false;
    }

    if (!Adapt)
    {
        return true;
    }

    if (!Flux_Estimator_Scalar_Update(&Flux_Estimator, We, CUR_TS))
    {
        return false;
    }
    Obs_Para_Update();

    if (Measure)
    {
        Fine_Num += We * Flux_Estimator.State.Yd;
        Fine_Den += We * We;
        Fine_Cnt++;
    }
    return true;
}

static bool Fine_Calc(float *Flux_Out)
{
    float Flux;

    if ((Flux_Out == NULL) || (Fine_Cnt == 0U) || (Fine_Den <= 0.0f))
    {
        return false;
    }

    Flux = Fine_Num / Fine_Den;
    if (!__builtin_isfinite(Flux) || (Flux <= 0.0f))
    {
        return false;
    }
    *Flux_Out = Flux;
    return true;
}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;
    float We_Max;

    Result = (Flux_Result_T){ 0 };
    IF_Para = (Motor_IF_Para_T){ 0 };
    We_Target = 0.0f;
    We_Obs_F = 0.0f;
    Emf_Ratio_F = 0.0f;
    Obs_We_Err_F = 0.0f;
    Obs_We_Err2_F = 0.0f;
    Obs_PLL_Err2_F = 0.0f;
    Obs_Theta_Err_F = 0.0f;
    Obs_Theta_Err2_F = 0.0f;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Fine_Num = 0.0f;
    Fine_Den = 0.0f;
    Fine_Flux_Pre = 0.0f;
    Finish_Iq = 0.0f;
    Obs_Wait_Cnt = 0U;
    Motion_Lost_Cnt = 0U;
    Blend_Cnt = 0U;
    Fine_Cnt = 0U;
    Fine_Stable_Cnt = 0U;
    Finish_Init = 0U;
    Model_U_Valid = false;
    Emf_Valid = false;
    Obs_Speed_Cmp_Valid = false;
    Obs_Theta_Cmp_Valid = false;
    Obs_Active = false;
    PLL_Active = false;
    Obs_Control = false;
    Emf_Target_Reached = false;
    Flux_Estimator_Reset(&Flux_Estimator);

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if ((Envelope->I_Max <= 0.0f) || !__builtin_isfinite(We_Max) || (We_Max <= 0.0f) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &IF_Para))
    {
        State = FLUX_FAILED;
        return false;
    }

    Flux_Estimator.Para.Rs = Motor_Para.Rs;
    Flux_Estimator.Para.Ld = Motor_Para.Ld;
    Flux_Estimator.Para.Lq = Motor_Para.Lq;
    Flux_Estimator.Para.I_BW_Hz = FLUX_COARSE_I_BW_HZ;
    Flux_Estimator.Para.Est_BW_Hz = FLUX_COARSE_BW_HZ;
    Flux_Estimator.Para.We_Min = FLUX_COARSE_WE_MIN_RATIO * IF_Para.We_Base;

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * ((IF_Para.We_Base < We_Max) ? IF_Para.We_Base : We_Max);
    IF_Para.Acc = Abs_Value(We_Target) / FLUX_IF_ACCEL_S;
    Align_Reset();
    Current_Loop_State_Reset();
    State = FLUX_ALIGN;
    return true;
}

bool Flux_Active(void)
{
    return (State != FLUX_IDLE) && (State != FLUX_DONE) && (State != FLUX_FAILED);
}

void Flux_Control(void)
{
    if ((State != FLUX_OBS_I_TRANS) && (State != FLUX_FINE_SETTLE) && (State != FLUX_FINE_MEASURE))
    {
        return;
    }
    if (IF_Para.Iq_Max_A <= 0.0f)
    {
        return;
    }
    Obs_Iq_Ref = Speed_Loop(We_Target, Flux_PLL.State.We, -IF_Para.Iq_Max_A, IF_Para.Iq_Max_A);
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A, float Ib_A, float Ic_A, float *Theta_e, float *Id_Ref, float *Iq_Ref)
{
    const Ident_Envelope_T *Envelope;
    float Ialpha;
    float Ibeta;
    float Theta_IF;
    float Theta_Obs;
    float Theta_Rough;
    float Theta_Err;
    float Id_IF;
    float Iq_IF;
    float Iq_Step;
    float Flux_Fine;
    float Emf_Ratio;
    float Blend;
    float I_Max;
    float U_Mag2;
    bool Adapt_Valid;
    bool Fine_Valid;
    bool Fine_Sample_Valid;
    bool Flux_Ready;
    bool Motion_Valid;
    int8_t Dir;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    Envelope = Identification_Envelope_Get();
    Dir = (We_Target < 0.0f) ? -1 : 1;

    if (State == FLUX_ALIGN)
    {
        if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Start_Reset(-0.5f * PI_F * (float)Dir, 0.0f);
            IF_Start_Para_Set(IF_Para.Iq_Start_A, IF_Para.Iq_Max_A, IF_Para.We_Base, IF_Para.Acc);
            IF_Start_Target_Set(We_Target);
            State = FLUX_IF;
        }
        return FAST_CURRENT;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    if (Obs_Active)
    {
        Flux_Observer_Run(&Flux_Obs, Motor_Run.Ualpha, Motor_Run.Ubeta, Ialpha, Ibeta, CUR_TS);
        if (PLL_Active)
        {
            PLL_Run(&Flux_PLL, Flux_Obs.State.PsiAlpha, Flux_Obs.State.PsiBeta, Flux_Obs.Para.Flux, CUR_TS);
            We_Obs_F += FLUX_OBS_WE_ALPHA * (Flux_PLL.State.We - We_Obs_F);
        }
    }
    Theta_Obs = Flux_PLL.State.Theta;

    if (State == FLUX_FINISH)
    {
        if (Obs_Control)
        {
            *Theta_e = Theta_Obs;
            *Id_Ref = 0.0f;
            *Iq_Ref = Obs_Iq_Ref;
        }
        else
        {
            IF_Start_Run(&Theta_IF, Id_Ref, Iq_Ref);
            *Theta_e = Theta_IF;
        }

        if (Finish_Init == 0U)
        {
            Finish_Iq = *Iq_Ref;
            Finish_Init = 1U;
        }
        Iq_Step = FLUX_FINISH_IQ_SLEW_A_S * CUR_TS;
        if (Finish_Iq > Iq_Step)
        {
            Finish_Iq -= Iq_Step;
        }
        else if (Finish_Iq < -Iq_Step)
        {
            Finish_Iq += Iq_Step;
        }
        else
        {
            Finish_Iq = 0.0f;
        }
        *Id_Ref = 0.0f;
        *Iq_Ref = Finish_Iq;
        if (Finish_Iq == 0.0f)
        {
            if (++Cnt >= FLUX_FINISH_CNT)
            {
                State = Result.Valid ? FLUX_DONE : FLUX_FAILED;
                return FAST_OFF;
            }
        }
        else
        {
            Cnt = 0U;
        }
        return FAST_CURRENT;
    }

    if ((Envelope->I_Max <= 0.0f) || (Envelope->U_Max <= 0.0f) || (Envelope->U_Available <= 0.0f))
    {
        State = FLUX_FAILED;
        Result.Valid = false;
        return FAST_OFF;
    }

    Theta_IF = Motor_Run.Theta_e;
    Id_IF = 0.0f;
    Iq_IF = 0.0f;
    Flux_Ready = false;

    if (Model_U_Valid && ((State == FLUX_IF) || (State == FLUX_OBS_WAIT) || (State == FLUX_OBS_BLEND)))
    {
        Flux_Ready = Coarse_Run();
        if (!__builtin_isfinite(Flux_Estimator.State.Psi_d) || !__builtin_isfinite(Flux_Estimator.State.Psi_q))
        {
            State = FLUX_FAILED;
            Result.Valid = false;
            return FAST_OFF;
        }

        if (Flux_Ready)
        {
            if ((State == FLUX_OBS_WAIT) && !Emf_Target_Reached)
            {
                if (PLL_Active && Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
                    (Emf_Ratio_F >= FLUX_OBS_EMF_ENTER_RATIO))
                {
                    We_Target = IF_Start_We_Get();
                    IF_Start_Target_Set(We_Target);
                    Emf_Target_Reached = true;
                }
                else
                {
                    IF_Target_Update(Dir);
                }
            }

            if (!Obs_Active)
            {
                Theta_Rough = Angle_Wrap(Theta_IF + __builtin_atan2f(Flux_Estimator.State.Psi_q,
                                                                     Flux_Estimator.State.Psi_d));
                Flux_Obs.Para.Rs = Motor_Para.Rs;
                Flux_Obs.Para.Ls = Motor_Para.Ld;
                Obs_Para_Update();
                Flux_PLL.Para.Kp = FLUX_PLL_KP;
                Flux_PLL.Para.Ki = FLUX_PLL_KI;
                Flux_Observer_Reset(&Flux_Obs, Theta_Rough, Ialpha, Ibeta);
                PLL_Reset(&Flux_PLL, Theta_Rough, IF_Start_We_Get());
                We_Obs_F = IF_Start_We_Get();
                Obs_Wait_Cnt = 0U;
                Obs_Compare_Reset();
                Obs_Active = true;
                Theta_Obs = Theta_Rough;
            }
            else
            {
                Obs_Para_Update();
            }
        }
    }

    if ((State == FLUX_IF) || (State == FLUX_OBS_WAIT) || (State == FLUX_OBS_BLEND))
    {
        IF_Start_Run(&Theta_IF, &Id_IF, &Iq_IF);
        *Theta_e = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;
        Model_U_Valid = true;
        if ((State == FLUX_IF) && (IF_Start_State_Get() == IF_HOLD))
        {
            State = FLUX_OBS_WAIT;
        }

        if (Obs_Active && !PLL_Active && Emf_Valid && (Emf_Ratio_F >= FLUX_OBS_PLL_START_RATIO))
        {
            Theta_Rough = Angle_Wrap(__builtin_atan2f(Flux_Obs.State.PsiBeta, Flux_Obs.State.PsiAlpha));
            PLL_Reset(&Flux_PLL, Theta_Rough, IF_Start_We_Get());
            We_Obs_F = IF_Start_We_Get();
            Motion_Lost_Cnt = 0U;
            Obs_Compare_Reset();
            PLL_Active = true;
        }
        if (PLL_Active)
        {
            Obs_IF_Compare_Run(Theta_IF);
        }

        Motion_Valid = Emf_Valid && __builtin_isfinite(Emf_Ratio_F) && (Emf_Ratio_F >= FLUX_MOTION_LOST_RATIO);
        if (Motion_Lost_Run(Motion_Valid))
        {
            Result.Valid = false;
            Cnt = 0U;
            Finish_Init = 0U;
            State = FLUX_FINISH;
            return FAST_CURRENT;
        }
    }

    if (State == FLUX_OBS_WAIT)
    {
        Emf_Ratio = Emf_Ratio_F;
        Qualification_Accumulate(&Obs_Wait_Cnt,
                                 FLUX_OBS_WAIT_CNT,
                                 Emf_Target_Reached && PLL_Active &&
                                     (Emf_Ratio >= FLUX_OBS_EMF_EXIT_RATIO) && Obs_IF_Stable());
        if (Obs_Wait_Cnt >= FLUX_OBS_WAIT_CNT)
        {
            Blend_Cnt = 0U;
            State = FLUX_OBS_BLEND;
        }
    }
    else if (State == FLUX_OBS_BLEND)
    {
        Blend = (FLUX_BLEND_CNT > 0U) ? (float)(Blend_Cnt + 1U) / (float)FLUX_BLEND_CNT : 1.0f;
        if (Blend > 1.0f)
        {
            Blend = 1.0f;
        }
        Theta_Err = Angle_Diff(Theta_Obs, Theta_IF);
        *Theta_e = Angle_Wrap(Theta_IF + Blend * Theta_Err);
        DQ_Rotate(Theta_IF, *Theta_e, Id_IF, Iq_IF, Id_Ref, Iq_Ref);
        if (Blend_Cnt < FLUX_BLEND_CNT)
        {
            Blend_Cnt++;
        }
        if (Blend_Cnt >= FLUX_BLEND_CNT)
        {
            Obs_Id_Ref = *Id_Ref;
            Obs_Iq_Ref = *Iq_Ref;
            I_Max = IF_Para.Iq_Max_A;
            Speed_Loop_Track(We_Target, Flux_PLL.State.We, Obs_Iq_Ref, -I_Max, I_Max);
            Fine_Begin();
            Obs_Control = true;
            State = FLUX_OBS_I_TRANS;
        }
    }
    else if (State == FLUX_OBS_I_TRANS)
    {
        *Theta_e = Theta_Obs;
        Obs_Id_Ref = Ramp_Zero(Obs_Id_Ref, FLUX_ID_RAMP_STEP);
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;
        Obs_Speed_Compare_Run(We_Target);
        Adapt_Valid = Obs_Run_Valid();
        Fine_Valid = Fine_Run(Adapt_Valid, false);
        Motion_Valid = Obs_Run_Valid();
        if (Motion_Lost_Run(Motion_Valid))
        {
            Result.Valid = false;
            Cnt = 0U;
            Finish_Init = 0U;
            State = FLUX_FINISH;
            return FAST_CURRENT;
        }
        if (Obs_Id_Ref == 0.0f)
        {
            Cnt = 0U;
            Fine_Flux_Pre = 0.0f;
            Fine_Stable_Cnt = 0U;
            State = FLUX_FINE_SETTLE;
        }
    }
    else if (State == FLUX_FINE_SETTLE)
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        Obs_Speed_Compare_Run(We_Target);
        Adapt_Valid = Obs_Run_Valid();
        Fine_Valid = Fine_Run(Adapt_Valid, false);
        Motion_Valid = Obs_Run_Valid();
        if (Motion_Lost_Run(Motion_Valid))
        {
            Result.Valid = false;
            Cnt = 0U;
            Finish_Init = 0U;
            State = FLUX_FINISH;
            return FAST_CURRENT;
        }
        Emf_Ratio = Emf_Ratio_F;
        Qualification_Accumulate(&Cnt,
                                 FLUX_FINE_SETTLE_CNT,
                                 Fine_Valid && (Emf_Ratio >= FLUX_OBS_EMF_EXIT_RATIO) && Obs_Control_Stable());
        if (Cnt >= FLUX_FINE_SETTLE_CNT)
        {
            Fine_Window_Reset();
            State = FLUX_FINE_MEASURE;
        }
    }
    else if (State == FLUX_FINE_MEASURE)
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        Obs_Speed_Compare_Run(We_Target);
        Adapt_Valid = Obs_Run_Valid();
        Emf_Ratio = Emf_Ratio_F;
        U_Mag2 = Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq;
        Fine_Sample_Valid = Adapt_Valid && (Emf_Ratio >= FLUX_OBS_EMF_EXIT_RATIO) &&
                            Obs_Control_Stable() && (U_Mag2 <= Envelope->U_Max * Envelope->U_Max);
        Fine_Valid = Fine_Run(Adapt_Valid, Fine_Sample_Valid);
        Motion_Valid = Obs_Run_Valid();
        if (Motion_Lost_Run(Motion_Valid))
        {
            Result.Valid = false;
            Cnt = 0U;
            Finish_Init = 0U;
            State = FLUX_FINISH;
            return FAST_CURRENT;
        }

        if (Fine_Valid && Fine_Cnt >= FLUX_FINE_MEASURE_CNT)
        {
            if (Fine_Calc(&Flux_Fine))
            {
                if ((Fine_Flux_Pre > 0.0f) &&
                    (Abs_Value(Flux_Fine - Fine_Flux_Pre) <= FLUX_FINE_STABLE_RATIO * Fine_Flux_Pre))
                {
                    if (Fine_Stable_Cnt < FLUX_FINE_STABLE_WINDOWS)
                    {
                        Fine_Stable_Cnt++;
                    }
                }
                else if (Fine_Stable_Cnt > 0U)
                {
                    Fine_Stable_Cnt--;
                }

                Flux_Estimator.State.Flux += FLUX_FINE_UPDATE_RATIO * (Flux_Fine - Flux_Estimator.State.Flux);
                Obs_Para_Update();
                if (Fine_Stable_Cnt >= FLUX_FINE_STABLE_WINDOWS)
                {
                    Result.Flux_Wb = 0.5f * (Fine_Flux_Pre + Flux_Fine);
                    Result.Valid = true;
                    Cnt = 0U;
                    Finish_Init = 0U;
                    State = FLUX_FINISH;
                }
                Fine_Flux_Pre = Flux_Fine;
            }
            Fine_Window_Reset();
        }
    }
    else if (State != FLUX_IF)
    {
        State = FLUX_FAILED;
        Result.Valid = false;
        return FAST_OFF;
    }

    return FAST_CURRENT;
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
