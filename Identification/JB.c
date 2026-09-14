/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "JB.h"

#include <stdint.h>

#include "Align.h"
#include "Current_Loop.h"
#include "Flux_Observer.h"
#include "Handover.h"
#include "IF_Start.h"
#include "Identification.h"
#include "Math.h"
#include "Motion_Loop.h"
#include "Motor_ADC.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define JB_OPEN_ACCEL_S        6.0f
#define JB_OBSERVER_BW_HZ      200.0f
#define JB_PLL_BW_HZ           50.0f
#define JB_PLL_DAMP            0.707f
#define JB_PLL_WN              (TWO_PI_F * JB_PLL_BW_HZ)
#define JB_PLL_KP              (2.0f * JB_PLL_DAMP * JB_PLL_WN)
#define JB_PLL_KI              (JB_PLL_WN * JB_PLL_WN)

#define JB_HANDOVER_READY_CNT  ((uint32_t)(0.20f / SPD_TS + 0.5f))
#define JB_HANDOVER_BLEND_CNT  ((uint32_t)(0.15f / CUR_TS + 0.5f))
#define JB_HANDOVER_ID_STEP    (5.0f * CUR_TS)
#define JB_HANDOVER_ALPHA      (SPD_TS / (0.050f + SPD_TS))
#define JB_HANDOVER_WE_MEAN    0.010f
#define JB_HANDOVER_WE_RMS     0.025f
#define JB_HANDOVER_PLL_RMS    0.08f
#define JB_HANDOVER_THETA_RMS  0.20f

#define JB_SETTLE_CNT          ((uint32_t)(0.50f / SPD_TS + 0.5f))
#define JB_EXCITE_HZ           5.0f
#define JB_EXCITE_CYCLES       3U
#define JB_MEASURE_CYCLES      10U
#define JB_IQ_EXCITE_RATIO     0.10f
#define JB_IQ_CONTROL_RATIO    0.35f
#define JB_FINISH_IQ_SLEW_A_S  20.0f
#define JB_FINISH_CNT          ((uint32_t)(0.10f / CUR_TS + 0.5f))

typedef enum
{
    JB_ALIGN = 0,
    JB_STARTUP,
    JB_SETTLE,
    JB_EXCITE,
    JB_MEASURE,
    JB_FINISH,

} JB_Phase_e;

typedef enum
{
    JB_START_IF = 0,
    JB_START_WAIT,
    JB_START_BLEND,
    JB_START_CURRENT,

} JB_Startup_Phase_e;

typedef struct
{
    float Iq_Cos;
    float Iq_Sin;
    float Wm_Cos;
    float Wm_Sin;
    uint32_t Samples;

} JB_DFT_T;

static volatile JB_Phase_e Phase = JB_ALIGN;
static volatile JB_Startup_Phase_e Startup_Phase = JB_START_IF;
static volatile bool Active = false;
static JB_Result_T Result = { 0 };
static Motor_IF_Para_T IF_Para = { 0 };
static IF_T JB_IF = { 0 };
static Handover_T Handover = { 0 };
static JB_DFT_T DFT = { 0 };

static float We_Target = 0.0f;
static float We_Startup = 0.0f;
static float We_Obs_F = 0.0f;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Iq_Bias = 0.0f;
static float Iq_Excite = 0.0f;
static float Excite_Phase = 0.0f;
static float Excite_Phase_Step = 0.0f;
static uint32_t Samples_Per_Cycle = 0U;
static uint32_t Phase_Samples = 0U;
static uint32_t Handover_Ready_Cnt = 0U;
static uint32_t Settle_Cnt = 0U;
static uint32_t Finish_Cnt = 0U;
static float Finish_Iq = 0.0f;
static bool Finish_Init = false;
static bool Obs_U_Valid = false;
static bool Obs_Control = false;

static volatile float Startup_Theta_Open = 0.0f;
static volatile float Startup_We_Open = 0.0f;
static volatile float Startup_Theta_Obs = 0.0f;
static volatile float Startup_We_Obs = 0.0f;
static volatile float Startup_PLL_Err = 0.0f;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Clamp(float Value, float Min, float Max)
{
    if (Value < Min)
    {
        return Min;
    }
    if (Value > Max)
    {
        return Max;
    }
    return Value;
}

static void DFT_Reset(void)
{
    DFT = (JB_DFT_T){ 0 };
}

static void Result_Calculate(void)
{
    float Kt;
    float Tr;
    float Ti;
    float Wr;
    float Wi;
    float Den;
    float Omega;
    float J;
    float B;

    Result = (JB_Result_T){ 0 };
    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
    Omega = TWO_PI_F * JB_EXCITE_HZ;

    Tr = Kt * DFT.Iq_Cos;
    Ti = -Kt * DFT.Iq_Sin;
    Wr = DFT.Wm_Cos;
    Wi = -DFT.Wm_Sin;
    Den = Wr * Wr + Wi * Wi;

    if (!__builtin_isfinite(Kt) || (Kt <= 0.0f) ||
        !__builtin_isfinite(Den) || (Den <= 1.0e-12f))
    {
        return;
    }

    B = (Tr * Wr + Ti * Wi) / Den;
    J = (Ti * Wr - Tr * Wi) / (Den * Omega);
    if (!__builtin_isfinite(J) || !__builtin_isfinite(B) ||
        (J <= 0.0f) || (B < 0.0f))
    {
        return;
    }

    Result.J_Kgm2 = J;
    Result.B_Nms = B;
    Result.Valid = true;
}

static void Excitation_Run(bool Measure)
{
    float Sin;
    float Cos;
    float I_Max;
    float Wm;
    uint32_t Limit;

    SinCos(Excite_Phase, &Sin, &Cos);
    I_Max = JB_IQ_CONTROL_RATIO * IF_Para.Iq_Max_A;
    Obs_Iq_Ref = Clamp(Iq_Bias + Iq_Excite * Sin, -I_Max, I_Max);

    if (Measure)
    {
        Wm = Startup_We_Obs / (float)Motor_Para.Pp;
        DFT.Iq_Cos += Motor_Run.Iq * Cos;
        DFT.Iq_Sin += Motor_Run.Iq * Sin;
        DFT.Wm_Cos += Wm * Cos;
        DFT.Wm_Sin += Wm * Sin;
        DFT.Samples++;
    }

    Excite_Phase += Excite_Phase_Step;
    if (Excite_Phase >= TWO_PI_F)
    {
        Excite_Phase -= TWO_PI_F;
    }

    Phase_Samples++;
    Limit = Samples_Per_Cycle * (Measure ? JB_MEASURE_CYCLES : JB_EXCITE_CYCLES);
    if (Phase_Samples < Limit)
    {
        return;
    }

    Phase_Samples = 0U;
    if (!Measure)
    {
        DFT_Reset();
        Phase = JB_MEASURE;
    }
    else
    {
        Result_Calculate();
        Phase = JB_FINISH;
        Finish_Init = false;
        Finish_Cnt = 0U;
    }
}

static void Startup_Supervision(void)
{
    bool Stable;

    if (Startup_Phase != JB_START_WAIT)
    {
        return;
    }

    Handover_Source_Compare(&Handover,
                            Startup_Theta_Open,
                            Startup_We_Open,
                            Startup_Theta_Obs,
                            Startup_We_Obs,
                            Startup_PLL_Err,
                            JB_HANDOVER_ALPHA);

    Stable = (Startup_We_Open * Startup_We_Obs > 0.0f) &&
             Handover_Source_Stable(&Handover,
                                    Startup_We_Open,
                                    IF_Para.We_Base,
                                    JB_HANDOVER_WE_MEAN,
                                    JB_HANDOVER_WE_RMS,
                                    JB_HANDOVER_PLL_RMS,
                                    JB_HANDOVER_THETA_RMS);

    Handover_Qualification_Accumulate(&Handover_Ready_Cnt,
                                      JB_HANDOVER_READY_CNT,
                                      Stable);
    if (Handover_Ready_Cnt >= JB_HANDOVER_READY_CNT)
    {
        Handover_Blend_Reset(&Handover);
        Startup_Phase = JB_START_BLEND;
    }
}

bool JB_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Abs;
    float Sign;

    Result = (JB_Result_T){ 0 };
    IF_Para = (Motor_IF_Para_T){ 0 };
    JB_IF = (IF_T){ 0 };
    Handover_Reset(&Handover);
    DFT_Reset();

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    We_Target = (float)Motor_Para.Pp * Wm_Target;
    We_Abs = Abs_Value(We_Target);

    if ((Motor_Para.Pp == 0U) ||
        !__builtin_isfinite(Motor_Para.Rs) || (Motor_Para.Rs <= 0.0f) ||
        !__builtin_isfinite(Motor_Para.Ld) || (Motor_Para.Ld <= 0.0f) ||
        !__builtin_isfinite(Motor_Para.Flux) || (Motor_Para.Flux <= 0.0f) ||
        !__builtin_isfinite(We_Target) || (We_Abs <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Abs > We_Max) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &IF_Para) ||
        (We_Abs < IF_Para.We_Base))
    {
        return false;
    }

    Sign = (We_Target < 0.0f) ? -1.0f : 1.0f;
    We_Startup = Sign * IF_Para.We_Base;

    JB_IF.Para.Iq_Min_A = IF_Para.Iq_Start_A;
    JB_IF.Para.Iq_Max_A = IF_Para.Iq_Max_A;
    JB_IF.Para.We_Base = IF_Para.We_Base;
    JB_IF.Para.Acc = IF_Para.We_Base / JB_OPEN_ACCEL_S;
    JB_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;
    JB_IF.Para.Rs_Ohm = Motor_Para.Rs;
    JB_IF.Para.Ld_H = Motor_Para.Ld;
    JB_IF.Para.Lq_H = Motor_Para.Lq;

    Ident_Observer.Para.Rs = Motor_Para.Rs;
    Ident_Observer.Para.Ls = Motor_Para.Ld;
    Ident_Observer.Para.Flux = Motor_Para.Flux;
    Ident_Observer.Para.BW_Hz = JB_OBSERVER_BW_HZ;
    Ident_PLL.Para.Kp = JB_PLL_KP;
    Ident_PLL.Para.Ki = JB_PLL_KI;

    Samples_Per_Cycle = (uint32_t)(1.0f / (JB_EXCITE_HZ * SPD_TS) + 0.5f);
    if (Samples_Per_Cycle == 0U)
    {
        return false;
    }
    Excite_Phase_Step = TWO_PI_F / (float)Samples_Per_Cycle;
    Iq_Excite = JB_IQ_EXCITE_RATIO * IF_Para.Iq_Max_A;

    Phase = JB_ALIGN;
    Startup_Phase = JB_START_IF;
    Handover_Ready_Cnt = 0U;
    Settle_Cnt = 0U;
    Finish_Cnt = 0U;
    Finish_Init = false;
    Obs_U_Valid = false;
    Obs_Control = false;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Iq_Bias = 0.0f;
    Excite_Phase = 0.0f;
    Phase_Samples = 0U;
    We_Obs_F = 0.0f;
    Startup_Theta_Open = 0.0f;
    Startup_We_Open = 0.0f;
    Startup_Theta_Obs = 0.0f;
    Startup_We_Obs = 0.0f;
    Startup_PLL_Err = 0.0f;

    Align_Reset();
    Current_Loop_State_Reset();
    Active = true;
    return true;
}

void JB_Abort(void)
{
    Active = false;
    Result.Valid = false;
}

bool JB_Active(void)
{
    return Active;
}

void JB_Control(void)
{
    float I_Max;

    if (!Active)
    {
        return;
    }

    if (Phase == JB_STARTUP)
    {
        Startup_Supervision();
        return;
    }

    if (Phase == JB_SETTLE)
    {
        I_Max = JB_IQ_CONTROL_RATIO * IF_Para.Iq_Max_A;
        Obs_Iq_Ref = Speed_Loop(We_Target, Startup_We_Obs, -I_Max, I_Max);
        if (++Settle_Cnt >= JB_SETTLE_CNT)
        {
            Iq_Bias = Obs_Iq_Ref;
            Excite_Phase = 0.0f;
            Phase_Samples = 0U;
            Phase = JB_EXCITE;
        }
        return;
    }

    if (Phase == JB_EXCITE)
    {
        Excitation_Run(false);
        return;
    }

    if (Phase == JB_MEASURE)
    {
        Excitation_Run(true);
    }
}

Motor_Fast_Mode_e JB_Fast_Run(float Ia_A,
                              float Ib_A,
                              float Ic_A,
                              float *Theta_e,
                              float *Id_Ref,
                              float *Iq_Ref,
                              float *Ualpha_V,
                              float *Ubeta_V)
{
    float Ialpha;
    float Ibeta;
    float Theta_Open;
    float Theta_Obs;
    float Id_Open;
    float Iq_Open;
    float Sin;
    float Cos;
    float Ud_Open;
    float Uq_Open;
    float I_Max;
    int8_t Dir;

    (void)Ic_A;
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (!Active)
    {
        return FAST_OFF;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Dir = (We_Target < 0.0f) ? -1 : 1;

    if (Phase == JB_ALIGN)
    {
        if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Init(&JB_IF, -0.5f * PI_F * (float)Dir, 0.0f);
            JB_IF.State.Iq = (float)Dir * IF_Para.Iq_Start_A;
            IF_Target_Set(&JB_IF, We_Startup);
            Flux_Observer_Reset(&Ident_Observer, JB_IF.State.Theta_e, Ialpha, Ibeta);
            PLL_Reset(&Ident_PLL, JB_IF.State.Theta_e, 0.0f);
            We_Obs_F = 0.0f;
            Obs_U_Valid = false;
            Phase = JB_STARTUP;
        }
        return FAST_CURRENT;
    }

    Theta_Obs = Ident_PLL.State.Theta;
    if (Obs_U_Valid && Flux_Observer_Run(&Ident_Observer,
                                         Motor_Run.Ualpha,
                                         Motor_Run.Ubeta,
                                         Ialpha,
                                         Ibeta,
                                         CUR_TS))
    {
        if (PLL_Run(&Ident_PLL,
                    Ident_Observer.State.PsiAlpha,
                    Ident_Observer.State.PsiBeta,
                    CUR_TS))
        {
            We_Obs_F += (CUR_TS / (0.020f + CUR_TS)) * (Ident_PLL.State.We - We_Obs_F);
            Theta_Obs = Ident_PLL.State.Theta;
        }
    }

    Startup_Theta_Obs = Theta_Obs;
    Startup_We_Obs = We_Obs_F;
    Startup_PLL_Err = Ident_PLL.State.Err;

    if (Phase == JB_STARTUP)
    {
        Theta_Open = JB_IF.State.Theta_e;
        SinCos(Theta_Open, &Sin, &Cos);
        Id_Open = Ialpha * Cos + Ibeta * Sin;
        Iq_Open = -Ialpha * Sin + Ibeta * Cos;
        Ud_Open = Motor_Run.Ualpha * Cos + Motor_Run.Ubeta * Sin;
        Uq_Open = -Motor_Run.Ualpha * Sin + Motor_Run.Ubeta * Cos;

        if (Startup_Phase == JB_START_IF)
        {
            JB_IF.Para.Acc = Abs_Value(We_Startup) / JB_OPEN_ACCEL_S *
                             ((Abs_Value(JB_IF.State.We) < 0.50f * IF_Para.We_Base) ? 3.0f : 0.60f);
        }

        IF_Run(&JB_IF,
               Id_Open,
               Iq_Open,
               Ud_Open,
               Uq_Open,
               &Theta_Open,
               &Id_Open,
               &Iq_Open,
               CUR_TS);
        if (JB_IF.State.Mode == IF_FAILED)
        {
            JB_Abort();
            return FAST_OFF;
        }

        Obs_U_Valid = true;
        Startup_Theta_Open = Theta_Open;
        Startup_We_Open = JB_IF.State.We;

        if (Startup_Phase == JB_START_IF)
        {
            *Theta_e = Theta_Open;
            *Id_Ref = Id_Open;
            *Iq_Ref = Iq_Open;
            if (JB_IF.State.Mode == IF_HOLD)
            {
                Handover_Ready_Cnt = 0U;
                Handover_Compare_Reset(&Handover);
                Startup_Phase = JB_START_WAIT;
            }
            return FAST_CURRENT;
        }

        if (Startup_Phase == JB_START_WAIT)
        {
            *Theta_e = Theta_Open;
            *Id_Ref = Id_Open;
            *Iq_Ref = Iq_Open;
            return FAST_CURRENT;
        }

        if (Startup_Phase == JB_START_BLEND)
        {
            if (Handover_Blend_Run(&Handover,
                                   JB_HANDOVER_BLEND_CNT,
                                   Theta_Open,
                                   Theta_Obs,
                                   Id_Open,
                                   Iq_Open,
                                   Theta_e,
                                   Id_Ref,
                                   Iq_Ref))
            {
                Obs_Id_Ref = *Id_Ref;
                Obs_Iq_Ref = *Iq_Ref;
                I_Max = JB_IQ_CONTROL_RATIO * IF_Para.Iq_Max_A;
                Speed_Loop_Track(We_Target, We_Obs_F, Obs_Iq_Ref, -I_Max, I_Max);
                Obs_Control = true;
                Startup_Phase = JB_START_CURRENT;
            }
            return FAST_CURRENT;
        }

        *Theta_e = Theta_Obs;
        Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, JB_HANDOVER_ID_STEP);
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;
        if (Obs_Id_Ref == 0.0f)
        {
            Settle_Cnt = 0U;
            Phase = JB_SETTLE;
        }
        return FAST_CURRENT;
    }

    if (Phase == JB_FINISH)
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        if (!Finish_Init)
        {
            Finish_Iq = Obs_Iq_Ref;
            Finish_Init = true;
        }

        if (Finish_Iq > JB_FINISH_IQ_SLEW_A_S * CUR_TS)
        {
            Finish_Iq -= JB_FINISH_IQ_SLEW_A_S * CUR_TS;
        }
        else if (Finish_Iq < -JB_FINISH_IQ_SLEW_A_S * CUR_TS)
        {
            Finish_Iq += JB_FINISH_IQ_SLEW_A_S * CUR_TS;
        }
        else
        {
            Finish_Iq = 0.0f;
        }

        *Iq_Ref = Finish_Iq;
        if (Finish_Iq == 0.0f)
        {
            if (++Finish_Cnt >= JB_FINISH_CNT)
            {
                Active = false;
                return FAST_OFF;
            }
        }
        else
        {
            Finish_Cnt = 0U;
        }
        return FAST_CURRENT;
    }

    if (Obs_Control)
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        return FAST_CURRENT;
    }

    JB_Abort();
    return FAST_OFF;
}

const JB_Result_T *JB_Result_Get(void)
{
    return &Result;
}
