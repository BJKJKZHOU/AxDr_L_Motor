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
#include "control_params.h"

typedef struct
{
    struct
    {
        float If_Accel_S;
        float Est_Num_Min_Wb;
        float U_Search_Ratio;
        float Target_Alpha;
        float Finish_Iq_Slew_A_S;
        uint32_t Finish_Cnt;
        float Breakaway_We_Ratio;
        float Breakaway_Acc_Ratio;
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
        float Emf_Enter_Ratio;
        float Emf_Exit_Ratio;
        float Pll_Start_Ratio;
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
        .If_Accel_S = 6.0f,
        .Est_Num_Min_Wb = 1.0e-12f,
        .U_Search_Ratio = 0.90f,
        .Target_Alpha = CUR_TS / (0.10f + CUR_TS),
        .Finish_Iq_Slew_A_S = 20.0f,
        .Finish_Cnt = (uint32_t)(0.100f / CUR_TS + 0.5f),
        .Breakaway_We_Ratio = 0.50f,
        .Breakaway_Acc_Ratio = 3.0f,
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
        .Emf_Enter_Ratio = 0.30f,
        .Emf_Exit_Ratio = 0.18f,
        .Pll_Start_Ratio = 0.10f,
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
    FLUX_INITIAL_IF,
    FLUX_SEARCH,
    FLUX_HANDOVER_BLEND,
    FLUX_HANDOVER_CURRENT,
    FLUX_REFINE_SETTLE,
    FLUX_REFINE_MEASURE,
    FLUX_FINISH,
    FLUX_DONE,
    FLUX_FAILED,

} Flux_State_e;

typedef enum
{
    FLUX_STEP_CONTINUE = 0,
    FLUX_STEP_CURRENT,
    FLUX_STEP_OFF,

} Flux_Step_e;

typedef struct
{
    const Ident_Envelope_T *Envelope;
    float Ialpha;
    float Ibeta;
    float Theta_IF;
    float Theta_Obs;
    float Id_IF;
    float Iq_IF;
    int8_t Dir;

} Flux_Fast_Context_T;

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T IF_Para = { 0 };
static IF_T Flux_IF = { 0 };
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
static bool Emf_Target_Reached = false;

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
                                 IF_Para.We_Base,
                                 Flux_Config.Handover.We_Mean_Ratio,
                                 Flux_Config.Handover.We_Rms_Ratio,
                                 Flux_Config.Handover.Pll_Rms_Max);
}

static bool Obs_IF_Stable(void)
{
    return Obs_State_Stable() && (Flux_IF.State.We * We_Obs_F > 0.0f) &&
           Handover_IF_Stable(&Handover,
                              Flux_IF.State.We,
                              IF_Para.We_Base,
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
    if (!PLL_Active || Motion_Valid)
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

static bool Coarse_Run(void)
{
    float We;

    We = Flux_IF.State.We;
    if (!Flux_Estimator_Run(&Flux_Estimator,
                            FLUX_EST_VECTOR,
                            FLUX_EST_UPDATE,
                            false,
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

    return Flux_Estimator.State.Estimate_Valid;
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
    if ((Envelope->U_Available <= 0.0f) || (Envelope->U_Max <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Max <= 0.0f))
    {
        return;
    }

    We_Abs = Abs_Value(Flux_IF.State.We);
    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= Flux_Config.Workflow.Est_Num_Min_Wb))
    {
        We_Target = (float)Dir * We_Abs;
        IF_Target_Set(&Flux_IF, We_Target);
        return;
    }

    We_Req = Flux_Config.Observer.Emf_Enter_Ratio * Envelope->U_Available / Flux;
    U_Mag = __builtin_sqrtf(Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq);
    U_Search_Max = Flux_Config.Workflow.U_Search_Ratio * Envelope->U_Max;
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
    We_Abs += Flux_Config.Workflow.Target_Alpha * (We_Req - We_Abs);
    IF_Para.Acc = We_Abs / Flux_Config.Workflow.If_Accel_S;
    Flux_IF.Para.Acc = IF_Para.Acc;
    We_Target = (float)Dir * We_Abs;
    IF_Target_Set(&Flux_IF, We_Target);
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
    Context->Envelope = Identification_Envelope_Get();
    Context->Ialpha = Ia_A;
    Context->Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Context->Theta_IF = Motor_Run.Theta_e;
    Context->Theta_Obs = Flux_PLL.State.Theta;
    Context->Id_IF = 0.0f;
    Context->Iq_IF = 0.0f;
    Context->Dir = (We_Target < 0.0f) ? -1 : 1;
}

static void Flux_Observer_Runtime_Run(Flux_Fast_Context_T *Context)
{
    if (Obs_Active)
    {
        Flux_Observer_Run(&Flux_Obs,
                          Motor_Run.Ualpha,
                          Motor_Run.Ubeta,
                          Context->Ialpha,
                          Context->Ibeta,
                          CUR_TS);
        if (PLL_Active)
        {
            PLL_Run(&Flux_PLL,
                    Flux_Obs.State.PsiAlpha,
                    Flux_Obs.State.PsiBeta,
                    Flux_Obs.Para.Flux,
                    CUR_TS);
            We_Obs_F += Flux_Config.Observer.We_Alpha * (Flux_PLL.State.We - We_Obs_F);
        }
    }
    Context->Theta_Obs = Flux_PLL.State.Theta;
}

static Motor_Fast_Mode_e Flux_Align_Run(int8_t Dir, float *Id_Ref, float *Iq_Ref)
{
    if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
    {
        Current_Loop_State_Reset();
        IF_Init(&Flux_IF, -0.5f * PI_F * (float)Dir, 0.0f);
        IF_Target_Set(&Flux_IF, We_Target);
        State = FLUX_INITIAL_IF;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Finish_Run(const Flux_Fast_Context_T *Context,
                                         float *Theta_e,
                                         float *Id_Ref,
                                         float *Iq_Ref)
{
    float Iq_Step;

    if (Obs_Control)
    {
        *Theta_e = Context->Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
    }
    else
    {
        IF_Run(&Flux_IF, Theta_e, Id_Ref, Iq_Ref, CUR_TS);
    }

    if (Finish_Init == 0U)
    {
        Finish_Iq = *Iq_Ref;
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

    *Id_Ref = 0.0f;
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
                                      float *Iq_Ref)
{
    float Theta_Rough;
    bool Flux_Ready;
    bool Motion_Valid;

    Flux_Ready = false;
    if (Model_U_Valid)
    {
        Flux_Ready = Coarse_Run();
        if (!__builtin_isfinite(Flux_Estimator.State.Psi_d) ||
            !__builtin_isfinite(Flux_Estimator.State.Psi_q))
        {
            Flux_Fail_Off();
            return FLUX_STEP_OFF;
        }

        if (Flux_Ready)
        {
            if ((State == FLUX_SEARCH) && !Emf_Target_Reached)
            {
                if (PLL_Active && Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
                    (Emf_Ratio_F >= Flux_Config.Observer.Emf_Enter_Ratio))
                {
                    We_Target = Flux_IF.State.We;
                    IF_Target_Set(&Flux_IF, We_Target);
                    Emf_Target_Reached = true;
                }
                else
                {
                    IF_Target_Update(Context->Dir);
                }
            }

            if (!Obs_Active)
            {
                Theta_Rough = Angle_Wrap(Context->Theta_IF +
                                         __builtin_atan2f(Flux_Estimator.State.Psi_q,
                                                          Flux_Estimator.State.Psi_d));
                Flux_Obs.Para.Rs = Motor_Para.Rs;
                Flux_Obs.Para.Ls = Motor_Para.Ld;
                Obs_Para_Update();
                Flux_PLL.Para.Kp = Flux_Config.Observer.Pll_Kp;
                Flux_PLL.Para.Ki = Flux_Config.Observer.Pll_Ki;
                Flux_Observer_Reset(&Flux_Obs, Theta_Rough, Context->Ialpha, Context->Ibeta);
                PLL_Reset(&Flux_PLL, Theta_Rough, Flux_IF.State.We);
                We_Obs_F = Flux_IF.State.We;
                Handover_Ready_Cnt = 0U;
                Handover_Compare_Reset(&Handover);
                Obs_Active = true;
                Context->Theta_Obs = Theta_Rough;
            }
            else
            {
                Obs_Para_Update();
            }
        }
    }

    IF_Run(&Flux_IF, &Context->Theta_IF, &Context->Id_IF, &Context->Iq_IF, CUR_TS);
    *Theta_e = Context->Theta_IF;
    *Id_Ref = Context->Id_IF;
    *Iq_Ref = Context->Iq_IF;
    Model_U_Valid = true;

    if ((State == FLUX_INITIAL_IF) && (Flux_IF.State.Mode == IF_HOLD))
    {
        State = FLUX_SEARCH;
    }

    if (Obs_Active && !PLL_Active && Emf_Valid &&
        (Emf_Ratio_F >= Flux_Config.Observer.Pll_Start_Ratio))
    {
        Theta_Rough = Angle_Wrap(__builtin_atan2f(Flux_Obs.State.PsiBeta, Flux_Obs.State.PsiAlpha));
        PLL_Reset(&Flux_PLL, Theta_Rough, Flux_IF.State.We);
        We_Obs_F = Flux_IF.State.We;
        Motion_Lost_Cnt = 0U;
        Handover_Compare_Reset(&Handover);
        PLL_Active = true;
    }

    Context->Theta_Obs = Flux_PLL.State.Theta;
    if (PLL_Active)
    {
        Handover_IF_Compare(&Handover,
                            Context->Theta_IF,
                            Flux_IF.State.We,
                            Context->Theta_Obs,
                            We_Obs_F,
                            Flux_PLL.State.Err,
                            Flux_Config.Handover.Compare_Alpha);
    }

    Motion_Valid = Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
                   (Emf_Ratio_F >= Flux_Config.Observer.Motion_Lost_Ratio);
    if (Motion_Lost_Run(Motion_Valid))
    {
        Result.Valid = false;
        Flux_Finish_Start();
        return FLUX_STEP_CURRENT;
    }

    return FLUX_STEP_CONTINUE;
}

static Motor_Fast_Mode_e Flux_Search_Run(void)
{
    Handover_Qualification_Accumulate(&Handover_Ready_Cnt,
                                      Flux_Config.Handover.Ready_Cnt,
                                      Emf_Target_Reached && PLL_Active &&
                                          (Emf_Ratio_F >= Flux_Config.Observer.Emf_Exit_Ratio) &&
                                          Obs_IF_Stable());
    if (Handover_Ready_Cnt >= Flux_Config.Handover.Ready_Cnt)
    {
        Handover_Blend_Reset(&Handover);
        State = FLUX_HANDOVER_BLEND;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Handover_Blend_Run(Flux_Fast_Context_T *Context,
                                                  float *Theta_e,
                                                  float *Id_Ref,
                                                  float *Iq_Ref)
{
    float I_Max;

    if (Handover_Blend_Run(&Handover,
                           Flux_Config.Handover.Blend_Cnt,
                           Context->Theta_IF,
                           Context->Theta_Obs,
                           Context->Id_IF,
                           Context->Iq_IF,
                           Theta_e,
                           Id_Ref,
                           Iq_Ref))
    {
        Obs_Id_Ref = *Id_Ref;
        Obs_Iq_Ref = *Iq_Ref;
        I_Max = IF_Para.Iq_Max_A;
        Speed_Loop_Track(We_Target, Flux_PLL.State.We, Obs_Iq_Ref, -I_Max, I_Max);
        Fine_Begin();
        Obs_Control = true;
        State = FLUX_HANDOVER_CURRENT;
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
                                      Fine_Valid &&
                                          (Emf_Ratio_F >= Flux_Config.Observer.Emf_Exit_Ratio) &&
                                          Obs_Control_Stable());
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
                        (Emf_Ratio_F >= Flux_Config.Observer.Emf_Exit_Ratio) &&
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
    IF_Para = (Motor_IF_Para_T){ 0 };
    Flux_IF = (IF_T){ 0 };
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
    Emf_Target_Reached = false;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };
    Handover_Reset(&Handover);

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
    Flux_Estimator.Para.I_BW_Hz = Flux_Config.Coarse.I_BW_Hz;
    Flux_Estimator.Para.Est_BW_Hz = Flux_Config.Coarse.Est_BW_Hz;
    Flux_Estimator.Para.We_Min = Flux_Config.Coarse.We_Min_Ratio * IF_Para.We_Base;
    Flux_Estimator.Para.Window_Update_Ratio = 0.0f;
    Flux_Estimator.Para.Window_Samples = 0U;

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * ((IF_Para.We_Base < We_Max) ? IF_Para.We_Base : We_Max);
    IF_Para.Acc = Abs_Value(We_Target) / Flux_Config.Workflow.If_Accel_S;

    Flux_IF.Para.Iq_Run_A = IF_Para.Iq_Max_A;
    Flux_IF.Para.We_Base = IF_Para.We_Base;
    Flux_IF.Para.Acc = IF_Para.Acc;
    Flux_IF.Para.Breakaway_We_Ratio = Flux_Config.Workflow.Breakaway_We_Ratio;
    Flux_IF.Para.Breakaway_Acc_Ratio = Flux_Config.Workflow.Breakaway_Acc_Ratio;
    Flux_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

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
        (State != FLUX_REFINE_SETTLE) &&
        (State != FLUX_REFINE_MEASURE))
    {
        return;
    }
    if (IF_Para.Iq_Max_A <= 0.0f)
    {
        return;
    }
    Obs_Iq_Ref = Speed_Loop(We_Target,
                            Flux_PLL.State.We,
                            -IF_Para.Iq_Max_A,
                            IF_Para.Iq_Max_A);
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float Ic_A,
                                float *Theta_e,
                                float *Id_Ref,
                                float *Iq_Ref)
{
    Flux_Fast_Context_T Context;
    Flux_Step_e Step;

    (void)Ic_A;
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
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
        return Flux_Finish_Run(&Context, Theta_e, Id_Ref, Iq_Ref);
    }

    if ((Context.Envelope->I_Max <= 0.0f) ||
        (Context.Envelope->U_Max <= 0.0f) ||
        (Context.Envelope->U_Available <= 0.0f))
    {
        return Flux_Fail_Off();
    }

    if ((State == FLUX_INITIAL_IF) ||
        (State == FLUX_SEARCH) ||
        (State == FLUX_HANDOVER_BLEND))
    {
        Step = Flux_Open_Loop_Run(&Context, Theta_e, Id_Ref, Iq_Ref);
        if (Step == FLUX_STEP_OFF)
        {
            return FAST_OFF;
        }
        if (Step == FLUX_STEP_CURRENT)
        {
            return FAST_CURRENT;
        }
    }

    switch (State)
    {
        case FLUX_INITIAL_IF:
            return FAST_CURRENT;

        case FLUX_SEARCH:
            return Flux_Search_Run();

        case FLUX_HANDOVER_BLEND:
            return Flux_Handover_Blend_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_HANDOVER_CURRENT:
            return Flux_Handover_Current_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

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
