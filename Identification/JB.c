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
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Ramp.h"
#include "Sin_LUT.h"
#include "control_params.h"
#include "main.h"

#define JB_OPEN_ACCEL_S        6.0f

#define JB_HANDOVER_BLEND_CNT  ((uint32_t)(0.15f / CUR_TS + 0.5f))
#define JB_HANDOVER_ID_STEP    (5.0f * CUR_TS)

#define JB_WORK_RATIO          0.30f
#define JB_SETTLE_CNT          ((uint32_t)(0.50f / SPD_TS + 0.5f))
#define JB_EXCITE_HZ           (Ident_JB_Excite_Hz)
#define JB_EXCITE_CYCLES       3U
#define JB_MEASURE_CYCLES      10U
#define JB_WM_EXCITE_RATIO     (Ident_JB_Excite_Ratio)
#define JB_FINISH_IQ_SLEW_A_S  20.0f

typedef enum
{
    JB_IDLE = 0,
    JB_ALIGN,
    JB_IF_RUN,
    JB_HANDOVER_BLEND,
    JB_HANDOVER_CURRENT,
    JB_EXCITE,
    JB_MEASURE,
    JB_FINISH,
    JB_DONE,
    JB_FAILED,

} JB_State_e;

typedef struct
{
    float Iq_Cos;
    float Iq_Sin;
    float Wm_Cos;
    float Wm_Sin;

} JB_DFT_T;

static volatile JB_State_e State = JB_IDLE;
JB_Result_T JB_Result = { 0 };
static Motor_IF_Para_T Start_Para = { 0 };
static IF_T JB_IF = { 0 };
static Handover_T Handover = { 0 };
static Motion_Ref_T JB_Speed_Ref = { 0 };
static JB_DFT_T DFT = { 0 };

static float We_Startup = 0.0f;
static float Wm_Bias = 0.0f;
static float Wm_Amp = 0.0f;
static volatile float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Excite_Phase = 0.0f;
static float Excite_Phase_Step = 0.0f;
static uint32_t Samples_Per_Cycle = 0U;
static uint32_t Phase_Samples = 0U;
static uint32_t Settle_Cnt = 0U;
static volatile bool Model_U_Valid = false;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void JB_Fail(void)
{
    JB_Result.Valid = false;
    State = JB_FAILED;
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

    JB_Result = (JB_Result_T){ 0 };
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

    JB_Result.J_Kgm2 = J;
    JB_Result.B_Nms = B;
    JB_Result.Valid = true;
}

static void Speed_Control_Run(float We_Ref)
{
    float I_Max;

    I_Max = Identification_Envelope_Get()->I_Max;
    if (I_Max <= 0.0f)
    {
        return;
    }

    Obs_Iq_Ref = Speed_Loop(We_Ref,
                            Ident_PLL.State.We,
                            -I_Max,
                            I_Max);
}

static void Excitation_Run(bool Measure)
{
    float Sin;
    float Cos;
    float Wm_Ref;
    float Wm;
    uint32_t Limit;

    SinCos(Excite_Phase, &Sin, &Cos);
    Wm_Ref = Wm_Bias + Wm_Amp * Sin;
    Speed_Control_Run((float)Motor_Para.Pp * Wm_Ref);

    if (Measure)
    {
        /* PLL_Run advances Theta with We + Kp * Err. Use that phase-advance
         * speed for the DFT; State.We alone is the integral state and its
         * phase lag biases B. This is not an empirical delay compensation. */
        Wm = (Ident_PLL.State.We +
              Ident_PLL.Para.Kp * Ident_PLL.State.Err) /
             (float)Motor_Para.Pp;
        DFT.Iq_Cos += Motor_Run.Iq * Cos;
        DFT.Iq_Sin += Motor_Run.Iq * Sin;
        DFT.Wm_Cos += Wm * Cos;
        DFT.Wm_Sin += Wm * Sin;
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
        State = JB_MEASURE;
    }
    else
    {
        Result_Calculate();
        State = JB_FINISH;
    }
}

bool JB_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Work_Max;
    float Pll_Wn;
    float We_Target;

    (void)Wm_Target;

    JB_Result = (JB_Result_T){ 0 };
    Start_Para = (Motor_IF_Para_T){ 0 };
    JB_IF = (IF_T){ 0 };

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();

    if ((Motor_Para.Pp == 0U) ||
        !__builtin_isfinite(Motor_Para.Rs) || (Motor_Para.Rs <= 0.0f) ||
        !__builtin_isfinite(Motor_Para.Ld) || (Motor_Para.Ld <= 0.0f) ||
        !__builtin_isfinite(Motor_Para.Flux) || (Motor_Para.Flux <= 0.0f) ||
        !__builtin_isfinite(Ident_JB_Excite_Ratio) || (Ident_JB_Excite_Ratio <= 0.0f) ||
        !__builtin_isfinite(Ident_JB_Excite_Hz) || (Ident_JB_Excite_Hz <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Max <= 0.0f) ||
        !__builtin_isfinite(Envelope->U_Available) || (Envelope->U_Available <= 0.0f) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Motor_Config.IF_Current_A, &Start_Para) ||
        !__builtin_isfinite(Motor_Config.Align_Current_A) ||
        (Motor_Config.Align_Current_A <= 0.0f) ||
        (Motor_Config.Align_Current_A > Envelope->I_Max) ||
        !__builtin_isfinite(Motor_Config.IF_Current_A) ||
        (Motor_Config.IF_Current_A <= 0.0f) ||
        (Motor_Config.IF_Current_A > Envelope->I_Max))
    {
        State = JB_FAILED;
        return false;
    }

    /* Reuse the validated Flux observer work point. J/B does not take a Host
     * speed target: the bias speed is derived from the known flux and current
     * available phase voltage. Leave headroom for the sinusoidal speed swing. */
    We_Target = JB_WORK_RATIO * Envelope->U_Available / Motor_Para.Flux;
    We_Work_Max = We_Max / (1.0f + JB_WM_EXCITE_RATIO);
    if (We_Target < Start_Para.We_Base)
    {
        We_Target = Start_Para.We_Base;
    }
    if (We_Target > We_Work_Max)
    {
        We_Target = We_Work_Max;
    }
    if (!__builtin_isfinite(We_Target) ||
        (We_Target < Start_Para.We_Base) ||
        (We_Target <= 0.0f))
    {
        State = JB_FAILED;
        return false;
    }

    We_Startup = Start_Para.We_Base;
    Wm_Bias = We_Target / (float)Motor_Para.Pp;
    Wm_Amp = JB_WM_EXCITE_RATIO * Wm_Bias;

    JB_IF.Para.Iq_Min_A = Motor_Config.IF_Current_A;
    JB_IF.Para.Iq_Max_A = Start_Para.Iq_Max_A;
    JB_IF.Para.We_Base = Start_Para.We_Base;
    JB_IF.Para.Acc = Abs_Value(We_Startup) / JB_OPEN_ACCEL_S;
    JB_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

    Ident_Observer.Para.Rs = Motor_Para.Rs;
    Ident_Observer.Para.Ls = Motor_Para.Ld;
    Ident_Observer.Para.Flux = Motor_Para.Flux;
    Ident_Observer.Para.BW_Hz = Control_Flux_Obs_Bw_Hz;
    Pll_Wn = TWO_PI_F * Control_PLL_Bw_Hz;
    Ident_PLL.Para.Kp = 2.0f * Control_PLL_Damp * Pll_Wn;
    Ident_PLL.Para.Ki = Pll_Wn * Pll_Wn;

    Samples_Per_Cycle = (uint32_t)(1.0f / (JB_EXCITE_HZ * SPD_TS) + 0.5f);
    if (Samples_Per_Cycle == 0U)
    {
        State = JB_FAILED;
        return false;
    }

    Excite_Phase_Step = TWO_PI_F / (float)Samples_Per_Cycle;

    State = JB_ALIGN;

    Align_Reset();
    Current_Loop_State_Reset();
    return true;
}

void JB_Abort(void)
{
    JB_Result.Valid = false;
    State = JB_IDLE;
}

bool JB_Active(void)
{
    return (State != JB_IDLE) && (State != JB_DONE) && (State != JB_FAILED);
}

void JB_Control(void)
{
    if (!JB_Active())
    {
        return;
    }

    if (State == JB_HANDOVER_CURRENT)
    {
        if (Obs_Id_Ref == 0.0f)
        {
            Ramp_Run(&JB_Speed_Ref, Wm_Bias,
                     Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
        }
        Speed_Control_Run((float)Motor_Para.Pp * JB_Speed_Ref.Wm);
        if ((Obs_Id_Ref == 0.0f) && (JB_Speed_Ref.Wm == Wm_Bias) &&
            (++Settle_Cnt >= JB_SETTLE_CNT))
        {
            Excite_Phase = 0.0f;
            Phase_Samples = 0U;
            State = JB_EXCITE;
        }
        return;
    }

    if (State == JB_EXCITE)
    {
        Excitation_Run(false);
        return;
    }

    if (State == JB_MEASURE)
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
    float I_Max;
    bool Obs_Valid;

    (void)Ic_A;
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (!JB_Active())
    {
        return FAST_OFF;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;

    if (State == JB_ALIGN)
    {
        if (Align_Current(Motor_Config.Align_Current_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Init(&JB_IF, -0.5f * PI_F, 0.0f);
            JB_IF.State.Iq = Motor_Config.IF_Current_A;
            IF_Target_Set(&JB_IF, We_Startup);
            if (JB_IF.State.Mode == IF_FAILED)
            {
                JB_Fail();
                return FAST_OFF;
            }

            Flux_Observer_Reset(&Ident_Observer, JB_IF.State.Theta_e, Ialpha, Ibeta);
            PLL_Reset(&Ident_PLL, JB_IF.State.Theta_e, 0.0f);
            Model_U_Valid = false;
            State = JB_IF_RUN;
        }
        return FAST_CURRENT;
    }

    Obs_Valid = Model_U_Valid &&
                Flux_Observer_Run(&Ident_Observer,
                                  Motor_Run.Ualpha, Motor_Run.Ubeta,
                                  Ialpha, Ibeta, CUR_TS) &&
                PLL_Run(&Ident_PLL,
                        Ident_Observer.State.PsiAlpha,
                        Ident_Observer.State.PsiBeta,
                        CUR_TS);
    Theta_Obs = Ident_PLL.State.Theta;

    if ((State == JB_IF_RUN) || (State == JB_HANDOVER_BLEND))
    {
        if (State == JB_IF_RUN)
        {
            JB_IF.Para.Acc = Abs_Value(We_Startup) / JB_OPEN_ACCEL_S *
                             ((Abs_Value(JB_IF.State.We) < 0.50f * Start_Para.We_Base) ?
                                  3.0f : 0.60f);
        }

        IF_Run(&JB_IF,
               &Theta_Open,
               &Id_Open,
               &Iq_Open,
               CUR_TS);
        if (JB_IF.State.Mode == IF_FAILED)
        {
            JB_Fail();
            return FAST_OFF;
        }

        Model_U_Valid = true;
        if (State == JB_IF_RUN)
        {
            if ((JB_IF.State.Mode == IF_HOLD) && Obs_Valid)
            {
                Handover_Blend_Reset(&Handover);
                State = JB_HANDOVER_BLEND;
            }

            *Theta_e = Theta_Open;
            *Id_Ref = Id_Open;
            *Iq_Ref = Iq_Open;
            return FAST_CURRENT;
        }

        if (State == JB_HANDOVER_BLEND)
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
                I_Max = Identification_Envelope_Get()->I_Max;
                Speed_Loop_Track(Ident_PLL.State.We,
                                 Ident_PLL.State.We,
                                 Obs_Iq_Ref,
                                 -I_Max,
                                 I_Max);
                /* Start the work-point ramp at the observer speed, not at
                 * the final excitation bias, while Id transfers to Iq. */
                Ramp_Reset(&JB_Speed_Ref, Ident_PLL.State.We / (float)Motor_Para.Pp);
                Settle_Cnt = 0U;
                State = JB_HANDOVER_CURRENT;
            }
            return FAST_CURRENT;
        }
    }

    if (State == JB_HANDOVER_CURRENT)
    {
        *Theta_e = Theta_Obs;
        Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, JB_HANDOVER_ID_STEP);
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;
        return FAST_CURRENT;
    }

    if (State == JB_FINISH)
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        Obs_Iq_Ref = Handover_Ramp_Zero(Obs_Iq_Ref, JB_FINISH_IQ_SLEW_A_S * CUR_TS);
        *Iq_Ref = Obs_Iq_Ref;
        if (Obs_Iq_Ref == 0.0f)
        {
            State = JB_Result.Valid ? JB_DONE : JB_FAILED;
            return FAST_OFF;
        }
        return FAST_CURRENT;
    }

    if ((State == JB_EXCITE) || (State == JB_MEASURE))
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        return FAST_CURRENT;
    }

    JB_Fail();
    return FAST_OFF;
}

