/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "JB.h"

#include <stdint.h>

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
#include "Motion.h"
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
static IF_T JB_IF = { 0 };
static uint32_t Align_Cnt = 0U;
static Handover_T Handover = { 0 };
static Motion_Ref_T JB_Speed_Ref = { 0 };
static JB_DFT_T DFT = { 0 };

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

    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
    Omega = TWO_PI_F * JB_EXCITE_HZ;

    Tr = Kt * DFT.Iq_Cos;
    Ti = -Kt * DFT.Iq_Sin;
    Wr = DFT.Wm_Cos;
    Wi = -DFT.Wm_Sin;
    Den = Wr * Wr + Wi * Wi;

    if (!__builtin_isfinite(Den) || (Den <= 1.0e-12f))
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

static void Excitation_Run(bool Measure)
{
    float Sin;
    float Cos;
    float Wm_Ref;
    float Wm;
    uint32_t Limit;

    SinCos(Excite_Phase, &Sin, &Cos);
    Wm_Ref = Wm_Bias + Wm_Amp * Sin;
    Obs_Iq_Ref = Speed_Loop((float)Motor_Para.Pp * Wm_Ref, Ident_PLL.State.We,
                           -Ident_Envelope.I_Max, Ident_Envelope.I_Max);

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
        DFT = (JB_DFT_T){ 0 };
        State = JB_MEASURE;
    }
    else
    {
        Result_Calculate();
        State = JB_FINISH;
    }
}

bool JB_Start(void)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Work_Max;
    float Pll_Wn;
    float We_Target;

    JB_Result = (JB_Result_T){ 0 };

    Envelope = &Ident_Envelope;
    We_Max = (float)Motor_Para.Pp * User_Lim.Wm_Max;

    if ((Motor_Config.Align_Current_A > Envelope->I_Max) ||
        (Motor_Config.IF_Current_A > Envelope->I_Max))
    {
        return false;
    }

    JB_IF.Para.We_Base = Motor_IF_We_Base_Calc();

    /* 按反电动势占可用电压的 30% 选取工作点，并为正弦速度激励预留转速余量。 */
    We_Target = JB_WORK_RATIO * Envelope->U_Available / Motor_Para.Flux;
    We_Work_Max = We_Max / (1.0f + JB_WM_EXCITE_RATIO);
    if (We_Target < JB_IF.Para.We_Base)
    {
        We_Target = JB_IF.Para.We_Base;
    }
    if (We_Target > We_Work_Max)
    {
        We_Target = We_Work_Max;
    }

    Wm_Bias = We_Target / (float)Motor_Para.Pp;
    Wm_Amp = JB_WM_EXCITE_RATIO * Wm_Bias;

    JB_IF.Para.Iq_Min_A = Motor_Config.IF_Current_A;
    JB_IF.Para.Acc = JB_IF.Para.We_Base / JB_OPEN_ACCEL_S;
    JB_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

    Ident_Observer.Para.Rs = Motor_Para.Rs;
    Ident_Observer.Para.Ld = Motor_Para.Ld;
    Ident_Observer.Para.Lq = Motor_Para.Lq;
    Ident_Observer.Para.Flux = Motor_Para.Flux;
    Ident_Observer.Para.BW_Hz = Control_Flux_Obs_Bw_Hz;
    Pll_Wn = TWO_PI_F * Control_PLL_Bw_Hz;
    Ident_PLL.Para.Kp = 2.0f * Control_PLL_Damp * Pll_Wn;
    Ident_PLL.Para.Ki = Pll_Wn * Pll_Wn;

    Samples_Per_Cycle = (uint32_t)(1.0f / (JB_EXCITE_HZ * SPD_TS) + 0.5f);
    if (Samples_Per_Cycle == 0U)
    {
        return false;
    }

    Excite_Phase_Step = TWO_PI_F / (float)Samples_Per_Cycle;

    State = JB_ALIGN;

    Align_Cnt = 0U;
    Current_Loop_State_Reset();
    return true;
}

void JB_Control(void)
{
    if (State == JB_HANDOVER_CURRENT)
    {
        if (Obs_Id_Ref == 0.0f)
        {
            Ramp_Run(&JB_Speed_Ref, Wm_Bias,
                     Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
        }
        Obs_Iq_Ref = Speed_Loop((float)Motor_Para.Pp * JB_Speed_Ref.Wm, Ident_PLL.State.We,
                               -Ident_Envelope.I_Max, Ident_Envelope.I_Max);
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

Motor_Fast_Mode_e JB_Fast_Run(float Ialpha,
                              float Ibeta,
                              float *Theta_e,
                              float *Id_Ref,
                              float *Iq_Ref)
{
    float Theta_Open;
    float Theta_Obs;
    float Id_Open;
    float Iq_Open;
    float I_Max;
    bool Obs_Valid;

    if (State == JB_ALIGN)
    {
        *Id_Ref = Motor_Config.Align_Current_A;

        if (++Align_Cnt >= IF_ALIGN_CNT)
        {
            float Ud_Int = Id_Ctrl.State.Int;

            IF_Init(&JB_IF, -0.5f * PI_F, 0.0f);
            JB_IF.State.Iq = Motor_Config.Align_Current_A;
            IF_Target_Set(&JB_IF, JB_IF.Para.We_Base);
            if (JB_IF.State.Failed)
            {
                Identification_Report(false);
                return FAST_OFF;
            }

            /* 参考和积分同拍换轴，继承预定位电流及其积分电压。 */
            *Theta_e = JB_IF.State.Theta_e;
            *Id_Ref = 0.0f;
            *Iq_Ref = JB_IF.State.Iq;
            Id_Ctrl.State.Int = -Iq_Ctrl.State.Int;
            Iq_Ctrl.State.Int = Ud_Int;

            Flux_Observer_Reset(&Ident_Observer, JB_IF.State.Theta_e, Ialpha, Ibeta);
            Ident_PLL.State = (PLL_State_T){ .Theta = Angle_Wrap(JB_IF.State.Theta_e), .We = 0.0f };
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
            JB_IF.Para.Acc = JB_IF.Para.We_Base / JB_OPEN_ACCEL_S *
                             ((__builtin_fabsf(JB_IF.State.We) < 0.50f * JB_IF.Para.We_Base) ?
                                  3.0f : 0.60f);
        }

        IF_Run(&JB_IF,
               &Theta_Open,
               &Id_Open,
               &Iq_Open,
               CUR_TS);
        if (JB_IF.State.Failed)
        {
            Identification_Report(false);
            return FAST_OFF;
        }

        Model_U_Valid = true;
        if (State == JB_IF_RUN)
        {
            if ((JB_IF.State.We == JB_IF.State.We_Target) && Obs_Valid)
            {
                Handover.Blend_Cnt = 0U;
                State = JB_HANDOVER_BLEND;
            }

            *Theta_e = Theta_Open;
            *Id_Ref = Id_Open;
            *Iq_Ref = Iq_Open;
            return FAST_CURRENT;
        }

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
            I_Max = Ident_Envelope.I_Max;
            Speed_Loop_Track(Ident_PLL.State.We,
                             Ident_PLL.State.We,
                             Obs_Iq_Ref,
                             -I_Max,
                             I_Max);
            /* Start the work-point ramp at the observer speed, not at
             * the final excitation bias, while Id transfers to Iq. */
            JB_Speed_Ref.Wm = Ident_PLL.State.We / (float)Motor_Para.Pp;
            JB_Speed_Ref.Am = 0.0f;
            Settle_Cnt = 0U;
            State = JB_HANDOVER_CURRENT;
        }
        return FAST_CURRENT;
    }

    *Theta_e = Theta_Obs;
    if (State == JB_HANDOVER_CURRENT)
    {
        Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, JB_HANDOVER_ID_STEP);
        *Id_Ref = Obs_Id_Ref;
    }
    else if (State == JB_FINISH)
    {
        Obs_Iq_Ref = Handover_Ramp_Zero(Obs_Iq_Ref, JB_FINISH_IQ_SLEW_A_S * CUR_TS);
        if (Obs_Iq_Ref == 0.0f)
        {
            Identification_Report(JB_Result.Valid != 0U);
            return FAST_OFF;
        }
    }

    *Iq_Ref = Obs_Iq_Ref;
    return FAST_CURRENT;
}

