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
#include "Handover.h"
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
#include "VF.h"
#include "control_params.h"

typedef struct
{
    struct
    {
        float Open_Accel_S;
        float Est_Num_Min_Wb;
        float U_Search_Ratio;
        float Target_Alpha;
        float Finish_Iq_Slew_A_S;
        uint32_t Finish_Cnt;
        float Breakaway_We_Ratio;
        float Breakaway_Acc_Ratio;
        float Current_Soft_Ratio;
    } Workflow;

    struct
    {
        float I_BW_Hz;
        float Est_BW_Hz;
        float We_Min_Ratio;
    } Coarse;

    struct
    {
        uint32_t Settle_Cnt;
        uint32_t Measure_Cnt;
        float I_BW_Hz;
        float Est_BW_Hz;
        float Stable_Ratio;
        uint32_t Stable_Windows;
        float Update_Ratio;
    } Fine;

    struct
    {
        float Work_Ratio;
        float Motion_Lost_Ratio;
        uint32_t Motion_Lost_Cnt;
        float BW_Hz;
        float Pll_Kp;
        float Pll_Ki;
        float We_Alpha;
        float Emf_Alpha;
    } Observer;

    struct
    {
        uint32_t Ready_Cnt;
        uint32_t Blend_Cnt;
        float Id_Ramp_Step;
        float Compare_Alpha;
        float We_Mean_Ratio;
        float We_Rms_Ratio;
        float Pll_Rms_Max;
        float Theta_Rms_Max;
    } Handover;

} Flux_Config_T;

static const Flux_Config_T Flux_Config = {
    .Workflow = {
        .Open_Accel_S = 6.0f,
        .Est_Num_Min_Wb = 1.0e-12f,
        .U_Search_Ratio = 0.90f,
        .Target_Alpha = CUR_TS / (0.10f + CUR_TS),
        .Finish_Iq_Slew_A_S = 20.0f,
        .Finish_Cnt = (uint32_t)(0.100f / CUR_TS + 0.5f),
        .Breakaway_We_Ratio = 0.50f,
        .Breakaway_Acc_Ratio = 3.0f,
        .Current_Soft_Ratio = 0.80f,
    },
    .Coarse = {
        .I_BW_Hz = 200.0f,
        .Est_BW_Hz = 20.0f,
        .We_Min_Ratio = 0.25f,
    },
    .Fine = {
        .Settle_Cnt = (uint32_t)(0.30f / CUR_TS + 0.5f),
        .Measure_Cnt = (uint32_t)(0.20f / CUR_TS + 0.5f),
        .I_BW_Hz = 200.0f,
        .Est_BW_Hz = 5.0f,
        .Stable_Ratio = 0.02f,
        .Stable_Windows = 2U,
        .Update_Ratio = 0.25f,
    },
    .Observer = {
        .Work_Ratio = 0.30f,
        .Motion_Lost_Ratio = 0.05f,
        .Motion_Lost_Cnt = (uint32_t)(0.15f / CUR_TS + 0.5f),
        .BW_Hz = 200.0f,
        .Pll_Kp = 2.0f * 0.707f * (TWO_PI_F * 50.0f),
        .Pll_Ki = (TWO_PI_F * 50.0f) * (TWO_PI_F * 50.0f),
        .We_Alpha = CUR_TS / (0.020f + CUR_TS),
        .Emf_Alpha = CUR_TS / (0.020f + CUR_TS),
    },
    .Handover = {
        .Ready_Cnt = (uint32_t)(0.20f / CUR_TS + 0.5f),
        .Blend_Cnt = (uint32_t)(0.15f / CUR_TS + 0.5f),
        .Id_Ramp_Step = 5.0f * CUR_TS,
        .Compare_Alpha = CUR_TS / (0.050f + CUR_TS),
        .We_Mean_Ratio = 0.010f,
        .We_Rms_Ratio = 0.015f,
        .Pll_Rms_Max = 0.08f,
        .Theta_Rms_Max = 0.08f,
    },
};

typedef enum
{
    FLUX_IDLE = 0,
    FLUX_ALIGN,
    FLUX_INITIAL_VF,
    FLUX_SEARCH,
    FLUX_HANDOVER_BLEND,
    FLUX_HANDOVER_CURRENT,
    FLUX_OBS_ACCEL,
    FLUX_REFINE_SETTLE,
    FLUX_REFINE_MEASURE,
    FLUX_FINISH,
    FLUX_DONE,
    FLUX_FAILED,

} Flux_State_e;

typedef enum
{
    FLUX_STEP_CONTINUE = 0,
    FLUX_STEP_FINISH,
    FLUX_STEP_OFF,

} Flux_Step_e;

typedef struct
{
    const Ident_Envelope_T *Envelope;
    float Ialpha;
    float Ibeta;
    float Theta_Open;
    float Theta_Obs;
    float Id_Open;
    float Iq_Open;
    float Ud_Open;
    float Uq_Open;
    int8_t Dir;

} Flux_Fast_Context_T;

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T Start_Para = { 0 };
static VF_T Flux_VF = { 0 };
static Flux_Estimator_T Flux_Estimator = { 0 };
static Handover_T Handover = { 0 };
static volatile float We_Target = 0.0f;
static uint32_t Cnt = 0U;
static float We_Obs_F = 0.0f;
static float Emf_Ratio_F = 0.0f;
static float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Fine_Flux_Pre = 0.0f;
static float Finish_Iq = 0.0f;
static uint32_t Handover_Ready_Cnt = 0U;
static uint32_t Motion_Lost_Cnt = 0U;
static uint32_t Fine_Stable_Cnt = 0U;
static uint8_t Finish_Init = 0U;
static bool Model_U_Valid = false;
static bool Emf_Valid = false;
static bool Obs_Active = false;
static bool PLL_Active = false;
static bool Obs_Control = false;
static bool Current_Control_Active = false;
static bool Work_Point_Reached = false;
static bool Motion_Lost_Armed = false;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Flux_Finish_Start(void)
{
    Cnt = 0U;
    Finish_Init = 0U;
    State = FLUX_FINISH;
}

static Motor_Fast_Mode_e Flux_Fail_Off(void)
{
    State = FLUX_FAILED;
    Result.Valid = false;
    return FAST_OFF;
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
        Emf_Ratio_F += Flux_Config.Observer.Emf_Alpha * (Ratio - Emf_Ratio_F);
    }
    return true;
}

static void Obs_Para_Update(void)
{
    float Flux;

    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= Flux_Config.Workflow.Est_Num_Min_Wb))
    {
        return;
    }

    Flux_Obs.Para.Flux = Flux;
    Flux_Obs.Para.BW_Hz = Flux_Config.Observer.BW_Hz;
}

static bool Obs_State_Stable(void)
{
    return __builtin_isfinite(We_Obs_F) && __builtin_isfinite(Flux_PLL.State.Err);
}

static bool Obs_Speed_Stable(float We_Ref)
{
    return Obs_State_Stable() && (We_Ref * We_Obs_F > 0.0f) &&
           Handover_Speed_Stable(&Handover,
                                 We_Ref,
                                 Start_Para.We_Base,
                                 Flux_Config.Handover.We_Mean_Ratio,
                                 Flux_Config.Handover.We_Rms_Ratio,
                                 Flux_Config.Handover.Pll_Rms_Max);
}

static bool Obs_Open_Stable(void)
{
    return Obs_State_Stable() && (Flux_VF.State.We * We_Obs_F > 0.0f) &&
           Handover_IF_Stable(&Handover,
                              Flux_VF.State.We,
                              Start_Para.We_Base,
                              Flux_Config.Handover.We_Mean_Ratio,
                              Flux_Config.Handover.We_Rms_Ratio,
                              Flux_Config.Handover.Pll_Rms_Max,
                              Flux_Config.Handover.Theta_Rms_Max);
}

static bool Obs_Control_Stable(void)
{
    return Obs_Speed_Stable(We_Target);
}

static bool Obs_Run_Valid(void)
{
    return Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
           (Emf_Ratio_F >= Flux_Config.Observer.Motion_Lost_Ratio) &&
           Obs_State_Stable() && (We_Target * We_Obs_F > 0.0f);
}

static bool Motion_Lost_Run(bool Motion_Valid)
{
    if (!Motion_Lost_Armed)
    {
        if (Motion_Valid)
        {
            Motion_Lost_Armed = true;
        }
        Motion_Lost_Cnt = 0U;
        return false;
    }

    if (Motion_Valid)
    {
        Motion_Lost_Cnt = 0U;
        return false;
    }
    if (Motion_Lost_Cnt < Flux_Config.Observer.Motion_Lost_Cnt)
    {
        Motion_Lost_Cnt++;
    }
    return Motion_Lost_Cnt >= Flux_Config.Observer.Motion_Lost_Cnt;
}

static bool Coarse_Run(float Ud, float Uq, float Id, float Iq, float We)
{
    if (!Flux_Estimator_Run(&Flux_Estimator,
                            FLUX_EST_VECTOR,
                            FLUX_EST_UPDATE,
                            false,
                            Ud,
                            Uq,
                            Id,
                            Iq,
                            We,
                            CUR_TS))
    {
        return false;
    }

    if (!Emf_Update(Flux_Estimator.State.Yd, Flux_Estimator.State.Yq))
    {
        return false;
    }

    return Flux_Estimator.State.Estimate_Valid;
}

static bool Flux_Target_Update(int8_t Dir, float We_Actual)
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
    if ((Envelope->U_Available <= 0.0f) || (Envelope->U_Max <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Max <= 0.0f))
    {
        return false;
    }

    We_Abs = Abs_Value(We_Actual);
    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= Flux_Config.Workflow.Est_Num_Min_Wb))
    {
        return false;
    }

    We_Req = Flux_Config.Observer.Work_Ratio * Envelope->U_Available / Flux;
    U_Mag = __builtin_sqrtf(Motor_Run.Ualpha * Motor_Run.Ualpha +
                            Motor_Run.Ubeta * Motor_Run.Ubeta);
    U_Search_Max = Flux_Config.Workflow.U_Search_Ratio * Envelope->U_Max;
    if ((We_Abs > 0.0f) && (U_Mag > 0.0f))
    {
        We_U_Max = We_Abs * U_Search_Max / U_Mag;
        if (We_Req > We_U_Max)
        {
            We_Req = We_U_Max;
        }
    }
    if (We_Req < Start_Para.We_Base)
    {
        We_Req = Start_Para.We_Base;
    }
    if (We_Req > We_Max)
    {
        We_Req = We_Max;
    }

    We_Abs = Abs_Value(We_Target);
    We_Abs += Flux_Config.Workflow.Target_Alpha * (We_Req - We_Abs);
    We_Target = (float)Dir * We_Abs;
    return true;
}

static void VF_Search_Target_Update(int8_t Dir)
{
    if (!Flux_Target_Update(Dir, Flux_VF.State.We))
    {
        return;
    }

    Flux_VF.Para.We_Acc = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;
}

static void Fine_Begin(void)
{
    float Flux_Init;

    Flux_Init = Flux_Estimator.State.Flux;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };
    Flux_Estimator.State.Flux = Flux_Init;
    Flux_Estimator.Para.I_BW_Hz = Flux_Config.Fine.I_BW_Hz;
    Flux_Estimator.Para.Est_BW_Hz = Flux_Config.Fine.Est_BW_Hz;
    Flux_Estimator.Para.Window_Update_Ratio = Flux_Config.Fine.Update_Ratio;
    Flux_Estimator.Para.Window_Samples = Flux_Config.Fine.Measure_Cnt;
}

static bool Fine_Run(bool Adapt, bool Measure)
{
    Flux_Estimator_Action_e Action;
    float We;

    We = We_Obs_F;
    Action = Adapt ? FLUX_EST_UPDATE : FLUX_EST_HOLD;
    if (!Flux_Estimator_Run(&Flux_Estimator,
                            FLUX_EST_SCALAR,
                            Action,
                            Measure,
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

    if (!Flux_Estimator.State.Estimate_Valid)
    {
        return false;
    }

    Obs_Para_Update();
    return true;
}

static void Flux_Context_Init(Flux_Fast_Context_T *Context, float Ia_A, float Ib_A)
{
    float Sin;
    float Cos;

    Context->Envelope = Identification_Envelope_Get();
    Context->Ialpha = Ia_A;
    Context->Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Context->Theta_Open = Flux_VF.State.Theta_e;
    Context->Theta_Obs = Flux_PLL.State.Theta;

    SinCos(Context->Theta_Open, &Sin, &Cos);
    Context->Id_Open = Context->Ialpha * Cos + Context->Ibeta * Sin;
    Context->Iq_Open = -Context->Ialpha * Sin + Context->Ibeta * Cos;
    Context->Ud_Open = Motor_Run.Ualpha * Cos + Motor_Run.Ubeta * Sin;
    Context->Uq_Open = -Motor_Run.Ualpha * Sin + Motor_Run.Ubeta * Cos;
    Context->Dir = (We_Target < 0.0f) ? -1 : 1;
}

static void Flux_Observer_Runtime_Run(Flux_Fast_Context_T *Context)
{
    if (!Obs_Active)
    {
        return;
    }

    if (!Flux_Observer_Run(&Flux_Obs,
                           Motor_Run.Ualpha,
                           Motor_Run.Ubeta,
                           Context->Ialpha,
                           Context->Ibeta,
                           CUR_TS))
    {
        if (!Obs_Control)
        {
            Obs_Active = false;
            PLL_Active = false;
            Handover_Ready_Cnt = 0U;
            Motion_Lost_Cnt = 0U;
            Handover_Compare_Reset(&Handover);
        }
        Context->Theta_Obs = Flux_PLL.State.Theta;
        return;
    }

    if (PLL_Active)
    {
        if (!PLL_Run(&Flux_PLL,
                     Flux_Obs.State.PsiAlpha,
                     Flux_Obs.State.PsiBeta,
                     CUR_TS))
        {
            if (!Obs_Control)
            {
                PLL_Active = false;
                Handover_Ready_Cnt = 0U;
                Motion_Lost_Cnt = 0U;
                Handover_Compare_Reset(&Handover);
            }
            Context->Theta_Obs = Flux_PLL.State.Theta;
            return;
        }
        We_Obs_F += Flux_Config.Observer.We_Alpha * (Flux_PLL.State.We - We_Obs_F);
    }

    Context->Theta_Obs = Flux_PLL.State.Theta;
}

static Motor_Fast_Mode_e Flux_Align_Run(int8_t Dir, float *Id_Ref, float *Iq_Ref)
{
    if (Align_Current(Start_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
    {
        Current_Loop_State_Reset();
        Flux_VF.State = (VF_State_T){ 0 };
        Flux_VF.State.Theta_e = -0.5f * PI_F * (float)Dir;
        Flux_VF.State.U = Motor_Para.Rs * Start_Para.Iq_Start_A;
        if (Flux_VF.State.U > Flux_VF.Para.U_Max_V)
        {
            Flux_VF.State.U = Flux_VF.Para.U_Max_V;
        }
        State = FLUX_INITIAL_VF;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Finish_Run(const Flux_Fast_Context_T *Context,
                                         float *Theta_e,
                                         float *Id_Ref,
                                         float *Iq_Ref,
                                         float *Ualpha_V,
                                         float *Ubeta_V)
{
    float Iq_Step;

    if (!Current_Control_Active)
    {
        VF_Run(&Flux_VF, 0.0f, 0.0f, Ualpha_V, Ubeta_V, CUR_TS);
        if (Flux_VF.State.U == 0.0f)
        {
            if (++Cnt >= Flux_Config.Workflow.Finish_Cnt)
            {
                State = Result.Valid ? FLUX_DONE : FLUX_FAILED;
                return FAST_OFF;
            }
        }
        else
        {
            Cnt = 0U;
        }
        return FAST_VOLTAGE;
    }

    if (Obs_Control)
    {
        *Theta_e = Context->Theta_Obs;
    }
    else
    {
        *Theta_e = Motor_Run.Theta_e;
    }
    *Id_Ref = 0.0f;

    if (Finish_Init == 0U)
    {
        Finish_Iq = Obs_Control ? Obs_Iq_Ref : Motor_Run.Iq;
        Finish_Init = 1U;
    }

    Iq_Step = Flux_Config.Workflow.Finish_Iq_Slew_A_S * CUR_TS;
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

    *Iq_Ref = Finish_Iq;
    if (Finish_Iq == 0.0f)
    {
        if (++Cnt >= Flux_Config.Workflow.Finish_Cnt)
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

static Flux_Step_e Flux_Open_Loop_Run(Flux_Fast_Context_T *Context,
                                      float *Theta_e,
                                      float *Id_Ref,
                                      float *Iq_Ref,
                                      float *Ualpha_V,
                                      float *Ubeta_V)
{
    float Theta_Rough;
    float I2;
    float I_Mag;
    float I_Soft_Max;
    float U_R;
    float U_L;
    float U_Base;
    float U_Target;
    float We_Command;
    bool Flux_Ready;
    bool Motion_Valid;

    Flux_Ready = false;
    if (Model_U_Valid)
    {
        Flux_Ready = Coarse_Run(Context->Ud_Open,
                                Context->Uq_Open,
                                Context->Id_Open,
                                Context->Iq_Open,
                                Flux_VF.State.We);
        if (!__builtin_isfinite(Flux_Estimator.State.Psi_d) ||
            !__builtin_isfinite(Flux_Estimator.State.Psi_q))
        {
            Flux_Fail_Off();
            return FLUX_STEP_OFF;
        }

        if ((State == FLUX_INITIAL_VF) && Flux_Ready && Emf_Valid &&
            __builtin_isfinite(Emf_Ratio_F) &&
            (Emf_Ratio_F >= Flux_Config.Observer.Motion_Lost_Ratio))
        {
            Motion_Lost_Armed = false;
            Motion_Lost_Cnt = 0U;
            State = FLUX_SEARCH;
        }

        if ((State != FLUX_INITIAL_VF) && Flux_Ready)
        {
            if ((State == FLUX_SEARCH) && !Work_Point_Reached)
            {
                if (Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
                    (Emf_Ratio_F >= Flux_Config.Observer.Work_Ratio))
                {
                    We_Target = Flux_VF.State.We;
                    Work_Point_Reached = true;
                }
                else
                {
                    VF_Search_Target_Update(Context->Dir);
                }
            }

            if (!Obs_Active)
            {
                Theta_Rough = Angle_Wrap(Context->Theta_Open +
                                         __builtin_atan2f(Flux_Estimator.State.Psi_q,
                                                          Flux_Estimator.State.Psi_d));
                Flux_Obs.Para.Rs = Motor_Para.Rs;
                Flux_Obs.Para.Ls = Motor_Para.Ld;
                Obs_Para_Update();
                Flux_PLL.Para.Kp = Flux_Config.Observer.Pll_Kp;
                Flux_PLL.Para.Ki = Flux_Config.Observer.Pll_Ki;
                Flux_Observer_Reset(&Flux_Obs, Theta_Rough, Context->Ialpha, Context->Ibeta);
                PLL_Reset(&Flux_PLL, Theta_Rough, Flux_VF.State.We);
                We_Obs_F = Flux_VF.State.We;
                Handover_Ready_Cnt = 0U;
                Handover_Compare_Reset(&Handover);
                Obs_Active = true;
                PLL_Active = true;
                Context->Theta_Obs = Theta_Rough;
            }
            else
            {
                Obs_Para_Update();
            }
        }
    }

    if (Obs_Active && !PLL_Active)
    {
        Theta_Rough = Angle_Wrap(__builtin_atan2f(Flux_Obs.State.PsiBeta, Flux_Obs.State.PsiAlpha));
        PLL_Reset(&Flux_PLL, Theta_Rough, Flux_VF.State.We);
        We_Obs_F = Flux_VF.State.We;
        Motion_Lost_Cnt = 0U;
        Handover_Ready_Cnt = 0U;
        Handover_Compare_Reset(&Handover);
        PLL_Active = true;
    }

    Context->Theta_Obs = Flux_PLL.State.Theta;
    if (PLL_Active)
    {
        Handover_IF_Compare(&Handover,
                            Context->Theta_Open,
                            Flux_VF.State.We,
                            Context->Theta_Obs,
                            We_Obs_F,
                            Flux_PLL.State.Err,
                            Flux_Config.Handover.Compare_Alpha);
    }

    if (State != FLUX_INITIAL_VF)
    {
        Motion_Valid = Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
                       (Emf_Ratio_F >= Flux_Config.Observer.Motion_Lost_Ratio);
        if (Motion_Lost_Run(Motion_Valid))
        {
            Result.Valid = false;
            Flux_Finish_Start();
            return FLUX_STEP_FINISH;
        }
    }

    I2 = Context->Ialpha * Context->Ialpha + Context->Ibeta * Context->Ibeta;
    I_Mag = (__builtin_isfinite(I2) && (I2 >= 0.0f)) ? __builtin_sqrtf(I2) : 0.0f;
    I_Soft_Max = Flux_Config.Workflow.Current_Soft_Ratio * Context->Envelope->I_Max;

    U_R = Motor_Para.Rs * Start_Para.Iq_Start_A;
    U_L = Abs_Value(Flux_VF.State.We) * Motor_Para.Lq * Start_Para.Iq_Start_A;
    U_Base = __builtin_sqrtf(U_R * U_R + U_L * U_L);
    U_Target = (Flux_VF.State.U > U_Base) ? Flux_VF.State.U : U_Base;
    We_Command = We_Target;

    if ((I_Soft_Max > 0.0f) && (I_Mag >= I_Soft_Max))
    {
        U_Target = Flux_VF.State.U;
        We_Command = Flux_VF.State.We;
    }
    else if (State == FLUX_INITIAL_VF)
    {
        U_Target += Flux_Config.Workflow.Target_Alpha * Motor_Para.Rs *
                    Start_Para.Iq_Start_A;
    }

    if (U_Target > Context->Envelope->U_Max)
    {
        U_Target = Context->Envelope->U_Max;
    }

    VF_Run(&Flux_VF,
           We_Command,
           U_Target,
           Ualpha_V,
           Ubeta_V,
           CUR_TS);

    *Theta_e = Flux_VF.State.Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    Motor_Run.Theta_e = Context->Theta_Open;
    Motor_Run.Id = Context->Id_Open;
    Motor_Run.Iq = Context->Iq_Open;
    Motor_Run.Ud = Context->Ud_Open;
    Motor_Run.Uq = Context->Uq_Open;
    Model_U_Valid = true;

    return FLUX_STEP_CONTINUE;
}

static Motor_Fast_Mode_e Flux_Search_Run(void)
{
    Handover_Qualification_Accumulate(&Handover_Ready_Cnt,
                                      Flux_Config.Handover.Ready_Cnt,
                                      PLL_Active && Obs_Open_Stable());
    if (Handover_Ready_Cnt >= Flux_Config.Handover.Ready_Cnt)
    {
        Handover_Blend_Reset(&Handover);
        State = FLUX_HANDOVER_BLEND;
    }
    return FAST_VOLTAGE;
}

static Motor_Fast_Mode_e Flux_Handover_Blend_Run(Flux_Fast_Context_T *Context,
                                                  float *Theta_e,
                                                  float *Id_Ref,
                                                  float *Iq_Ref)
{
    float I_Max;
    bool Blend_Start;

    Blend_Start = Handover.Blend_Cnt == 0U;
    if (Handover_Blend_Run(&Handover,
                           Flux_Config.Handover.Blend_Cnt,
                           Context->Theta_Open,
                           Context->Theta_Obs,
                           Context->Id_Open,
                           Context->Iq_Open,
                           Theta_e,
                           Id_Ref,
                           Iq_Ref))
    {
        Obs_Id_Ref = *Id_Ref;
        Obs_Iq_Ref = *Iq_Ref;
        I_Max = Start_Para.Iq_Max_A;
        Speed_Loop_Track(We_Target, Flux_PLL.State.We, Obs_Iq_Ref, -I_Max, I_Max);
        Fine_Begin();
        Obs_Control = true;
        State = FLUX_HANDOVER_CURRENT;
    }

    if (Blend_Start)
    {
        Current_Loop_Track(*Theta_e,
                           *Id_Ref,
                           *Iq_Ref,
                           Motor_Run.Ualpha,
                           Motor_Run.Ubeta);
        Current_Control_Active = true;
    }
    return FAST_CURRENT;
}

static bool Flux_Observer_Motion_Check(void)
{
    if (!Motion_Lost_Run(Obs_Run_Valid()))
    {
        return true;
    }

    Result.Valid = false;
    Flux_Finish_Start();
    return false;
}

static Motor_Fast_Mode_e Flux_Handover_Current_Run(const Flux_Fast_Context_T *Context,
                                                    float *Theta_e,
                                                    float *Id_Ref,
                                                    float *Iq_Ref)
{
    bool Adapt_Valid;

    *Theta_e = Context->Theta_Obs;
    Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, Flux_Config.Handover.Id_Ramp_Step);
    *Id_Ref = Obs_Id_Ref;
    *Iq_Ref = Obs_Iq_Ref;
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Flux_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    (void)Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return FAST_CURRENT;
    }

    if (Obs_Id_Ref == 0.0f)
    {
        Cnt = 0U;
        Fine_Flux_Pre = 0.0f;
        Fine_Stable_Cnt = 0U;
        State = Work_Point_Reached ? FLUX_REFINE_SETTLE : FLUX_OBS_ACCEL;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Obs_Accel_Run(const Flux_Fast_Context_T *Context,
                                             float *Theta_e,
                                             float *Id_Ref,
                                             float *Iq_Ref)
{
    bool Adapt_Valid;
    bool Fine_Valid;

    *Theta_e = Context->Theta_Obs;
    *Id_Ref = 0.0f;
    *Iq_Ref = Obs_Iq_Ref;
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Flux_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    Fine_Valid = Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return FAST_CURRENT;
    }

    if (Fine_Valid && Flux_Estimator.State.Estimate_Valid)
    {
        (void)Flux_Target_Update(Context->Dir, We_Obs_F);
    }

    if (Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
        (Emf_Ratio_F >= Flux_Config.Observer.Work_Ratio))
    {
        We_Target = We_Obs_F;
        Work_Point_Reached = true;
        Cnt = 0U;
        Fine_Flux_Pre = 0.0f;
        Fine_Stable_Cnt = 0U;
        State = FLUX_REFINE_SETTLE;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Refine_Settle_Run(const Flux_Fast_Context_T *Context,
                                                 float *Theta_e,
                                                 float *Id_Ref,
                                                 float *Iq_Ref)
{
    bool Adapt_Valid;
    bool Fine_Valid;

    *Theta_e = Context->Theta_Obs;
    *Id_Ref = 0.0f;
    *Iq_Ref = Obs_Iq_Ref;
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Flux_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    Fine_Valid = Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return FAST_CURRENT;
    }

    Handover_Qualification_Accumulate(&Cnt,
                                      Flux_Config.Fine.Settle_Cnt,
                                      Fine_Valid && Obs_Control_Stable());
    if (Cnt >= Flux_Config.Fine.Settle_Cnt)
    {
        State = FLUX_REFINE_MEASURE;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Refine_Measure_Run(const Flux_Fast_Context_T *Context,
                                                  float *Theta_e,
                                                  float *Id_Ref,
                                                  float *Iq_Ref)
{
    float Flux_Fine;
    float U_Mag2;
    bool Adapt_Valid;
    bool Fine_Valid;
    bool Fine_Sample_Valid;

    *Theta_e = Context->Theta_Obs;
    *Id_Ref = 0.0f;
    *Iq_Ref = Obs_Iq_Ref;
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Flux_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    U_Mag2 = Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq;
    Fine_Sample_Valid = Adapt_Valid &&
                        Obs_Control_Stable() &&
                        (U_Mag2 <= Context->Envelope->U_Max * Context->Envelope->U_Max);
    Fine_Valid = Fine_Run(Adapt_Valid, Fine_Sample_Valid);
    if (!Flux_Observer_Motion_Check())
    {
        return FAST_CURRENT;
    }

    if (Fine_Valid && Flux_Estimator.State.Window_Ready)
    {
        Flux_Fine = Flux_Estimator.State.Window_Flux;
        if ((Fine_Flux_Pre > 0.0f) &&
            (Abs_Value(Flux_Fine - Fine_Flux_Pre) <=
             Flux_Config.Fine.Stable_Ratio * Fine_Flux_Pre))
        {
            if (Fine_Stable_Cnt < Flux_Config.Fine.Stable_Windows)
            {
                Fine_Stable_Cnt++;
            }
        }
        else if (Fine_Stable_Cnt > 0U)
        {
            Fine_Stable_Cnt--;
        }

        Obs_Para_Update();
        if (Fine_Stable_Cnt >= Flux_Config.Fine.Stable_Windows)
        {
            Result.Flux_Wb = 0.5f * (Fine_Flux_Pre + Flux_Fine);
            Result.Valid = true;
            Flux_Finish_Start();
        }
        Fine_Flux_Pre = Flux_Fine;
    }
    return FAST_CURRENT;
}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;
    float We_Max;

    Result = (Flux_Result_T){ 0 };
    Start_Para = (Motor_IF_Para_T){ 0 };
    Flux_VF = (VF_T){ 0 };
    We_Target = 0.0f;
    We_Obs_F = 0.0f;
    Emf_Ratio_F = 0.0f;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Fine_Flux_Pre = 0.0f;
    Finish_Iq = 0.0f;
    Handover_Ready_Cnt = 0U;
    Motion_Lost_Cnt = 0U;
    Fine_Stable_Cnt = 0U;
    Finish_Init = 0U;
    Model_U_Valid = false;
    Emf_Valid = false;
    Obs_Active = false;
    PLL_Active = false;
    Obs_Control = false;
    Current_Control_Active = false;
    Work_Point_Reached = false;
    Motion_Lost_Armed = false;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };
    Handover_Reset(&Handover);

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if ((Envelope->I_Max <= 0.0f) || !__builtin_isfinite(We_Max) || (We_Max <= 0.0f) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &Start_Para))
    {
        State = FLUX_FAILED;
        return false;
    }

    Flux_Estimator.Para.Rs = Motor_Para.Rs;
    Flux_Estimator.Para.Ld = Motor_Para.Ld;
    Flux_Estimator.Para.Lq = Motor_Para.Lq;
    Flux_Estimator.Para.I_BW_Hz = Flux_Config.Coarse.I_BW_Hz;
    Flux_Estimator.Para.Est_BW_Hz = Flux_Config.Coarse.Est_BW_Hz;
    Flux_Estimator.Para.We_Min = Flux_Config.Coarse.We_Min_Ratio * Start_Para.We_Base;
    Flux_Estimator.Para.Window_Update_Ratio = 0.0f;
    Flux_Estimator.Para.Window_Samples = 0U;

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * ((Start_Para.We_Base < We_Max) ? Start_Para.We_Base : We_Max);

    Flux_VF.Para.U_Max_V = Envelope->U_Max;
    Flux_VF.Para.U_Slew_V_S = Envelope->U_Max / Flux_Config.Workflow.Open_Accel_S;
    Flux_VF.Para.We_Acc = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;

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
    if ((State != FLUX_HANDOVER_CURRENT) &&
        (State != FLUX_OBS_ACCEL) &&
        (State != FLUX_REFINE_SETTLE) &&
        (State != FLUX_REFINE_MEASURE))
    {
        return;
    }
    if (Start_Para.Iq_Max_A <= 0.0f)
    {
        return;
    }
    Obs_Iq_Ref = Speed_Loop(We_Target,
                            Flux_PLL.State.We,
                            -Start_Para.Iq_Max_A,
                            Start_Para.Iq_Max_A);
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float Ic_A,
                                float *Theta_e,
                                float *Id_Ref,
                                float *Iq_Ref,
                                float *Ualpha_V,
                                float *Ubeta_V)
{
    Flux_Fast_Context_T Context;
    Flux_Step_e Step;

    (void)Ic_A;
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;
    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    Flux_Context_Init(&Context, Ia_A, Ib_A);

    if (State == FLUX_ALIGN)
    {
        return Flux_Align_Run(Context.Dir, Id_Ref, Iq_Ref);
    }

    Flux_Observer_Runtime_Run(&Context);

    if (State == FLUX_FINISH)
    {
        return Flux_Finish_Run(&Context,
                               Theta_e,
                               Id_Ref,
                               Iq_Ref,
                               Ualpha_V,
                               Ubeta_V);
    }

    if ((Context.Envelope->I_Max <= 0.0f) ||
        (Context.Envelope->U_Max <= 0.0f) ||
        (Context.Envelope->U_Available <= 0.0f))
    {
        return Flux_Fail_Off();
    }

    if ((State == FLUX_INITIAL_VF) ||
        (State == FLUX_SEARCH) ||
        (State == FLUX_HANDOVER_BLEND))
    {
        Step = Flux_Open_Loop_Run(&Context,
                                  Theta_e,
                                  Id_Ref,
                                  Iq_Ref,
                                  Ualpha_V,
                                  Ubeta_V);
        if (Step == FLUX_STEP_OFF)
        {
            return FAST_OFF;
        }
        if (Step == FLUX_STEP_FINISH)
        {
            return Flux_Finish_Run(&Context,
                                   Theta_e,
                                   Id_Ref,
                                   Iq_Ref,
                                   Ualpha_V,
                                   Ubeta_V);
        }
    }

    switch (State)
    {
        case FLUX_INITIAL_VF:
            return FAST_VOLTAGE;

        case FLUX_SEARCH:
            return Flux_Search_Run();

        case FLUX_HANDOVER_BLEND:
            return Flux_Handover_Blend_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_HANDOVER_CURRENT:
            return Flux_Handover_Current_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_OBS_ACCEL:
            return Flux_Obs_Accel_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_REFINE_SETTLE:
            return Flux_Refine_Settle_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_REFINE_MEASURE:
            return Flux_Refine_Measure_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        default:
            return Flux_Fail_Off();
    }
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
