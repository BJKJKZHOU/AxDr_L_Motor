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
#include "Sin_LUT.h"
#include "control_params.h"
#include "main.h"

typedef struct
{
    struct
    {
        float Open_Accel_S;
        float Est_Num_Min_Wb;
        float Finish_Iq_Slew_A_S;
        uint32_t Finish_Cnt;
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
        float Work_U_Search_Ratio;
        float Target_Alpha;
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
        .Finish_Iq_Slew_A_S = 20.0f,
        .Finish_Cnt = (uint32_t)(0.100f / CUR_TS + 0.5f),
    },
    .Coarse = {
        .I_BW_Hz = 200.0f,
        .Est_BW_Hz = 20.0f,
        .We_Min_Ratio = 0.25f,
    },
    .Fine = {
        .Settle_Cnt = (uint32_t)(0.30f / SPD_TS + 0.5f),
        .Measure_Cnt = (uint32_t)(0.20f / SPD_TS + 0.5f),
        .I_BW_Hz = 200.0f,
        .Est_BW_Hz = 5.0f,
        .Stable_Ratio = 0.02f,
        .Stable_Windows = 2U,
        .Update_Ratio = 0.25f,
    },
    .Observer = {
        .Work_Ratio = 0.30f,
        .Motion_Lost_Ratio = 0.05f,
        .Work_U_Search_Ratio = 0.90f,
        .Target_Alpha = SPD_TS / (0.10f + SPD_TS),
        .Motion_Lost_Cnt = (uint32_t)(0.15f / SPD_TS + 0.5f),
        .BW_Hz = 200.0f,
        .Pll_Kp = 2.0f * 0.707f * (TWO_PI_F * 50.0f),
        .Pll_Ki = (TWO_PI_F * 50.0f) * (TWO_PI_F * 50.0f),
        .We_Alpha = CUR_TS / (0.020f + CUR_TS),
        .Emf_Alpha = SPD_TS / (0.020f + SPD_TS),
    },
    .Handover = {
        .Ready_Cnt = (uint32_t)(0.20f / SPD_TS + 0.5f),
        .Blend_Cnt = (uint32_t)(0.15f / CUR_TS + 0.5f),
        .Id_Ramp_Step = 5.0f * CUR_TS,
        .Compare_Alpha = SPD_TS / (0.050f + SPD_TS),
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
    FLUX_IF,
    FLUX_OBS_WAIT,
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
    float Id_Ref_Open;
    float Iq_Ref_Open;
    float Ud_Open;
    float Uq_Open;
    int8_t Dir;

} Flux_Fast_Context_T;

typedef struct
{
    float Ialpha;
    float Ibeta;
    float Theta_Open;
    float We_Open;
    float Theta_Obs;
    float We_Obs;
    float PLL_Err;
    float Id_Open;
    float Iq_Open;
    float Ud_Open;
    float Uq_Open;
    bool PLL_Active;

} Flux_Slow_Snapshot_T;

volatile Flux_Time_T Flux_Time = { 0 };

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T Start_Para = { 0 };
static IF_T Flux_IF = { 0 };
static Flux_Estimator_T Flux_Estimator = { 0 };
static Handover_T Handover = { 0 };
static volatile float We_Target = 0.0f;
static uint32_t Cnt = 0U;
static volatile float We_Obs_F = 0.0f;
static float Emf_Ratio_F = 0.0f;
static volatile float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Fine_Flux_Pre = 0.0f;
static float Finish_Iq = 0.0f;
static uint32_t Handover_Ready_Cnt = 0U;
static uint32_t Motion_Lost_Cnt = 0U;
static uint32_t Fine_Stable_Cnt = 0U;
static uint8_t Finish_Init = 0U;
static volatile bool Model_U_Valid = false;
static bool Emf_Valid = false;
static volatile bool Obs_Active = false;
static volatile bool PLL_Active = false;
static volatile bool Obs_Control = false;
static bool Current_Control_Active = false;
static bool Work_Point_Reached = false;
static bool Motion_Lost_Armed = false;

/* Fast ISR publishes open-loop source and observer data for the 2 kHz
 * supervisory path. Sequence is odd while the ISR writes and even while the
 * complete source/observer sample is stable. */
static volatile uint32_t Slow_Snapshot_Seq = 0U;
static Flux_Slow_Snapshot_T Slow_Snapshot = { 0 };

/* Slow supervision calculates the next I/F target. The fast path applies it so
 * IF_Run and IF_Target_Set never mutate Flux_IF.State from different contexts. */
static volatile float IF_Target_Command = 0.0f;
static volatile float IF_Acc_Command = 0.0f;
static volatile bool IF_Target_Pending = false;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Flux_Time_Set(uint32_t Cyc, volatile uint32_t *Current, volatile uint32_t *Max)
{
    *Current = Cyc;
    if (Cyc > *Max)
    {
        *Max = Cyc;
    }
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

    /* Flux is a naturally aligned 32-bit value on Cortex-M4. The fast observer
     * may consume either the old or new complete value, never a partial float. */
    Ident_Observer.Para.Flux = Flux;
    Ident_Observer.Para.BW_Hz = Flux_Config.Observer.BW_Hz;
}

static bool Obs_State_Stable(void)
{
    return __builtin_isfinite(We_Obs_F) && __builtin_isfinite(Ident_PLL.State.Err);
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

static bool Obs_Open_Stable(const Flux_Slow_Snapshot_T *Snapshot)
{
    if ((Snapshot == NULL) || !Snapshot->PLL_Active ||
        !__builtin_isfinite(Snapshot->We_Open) ||
        !__builtin_isfinite(Snapshot->We_Obs) ||
        !__builtin_isfinite(Snapshot->PLL_Err))
    {
        return false;
    }

    return (Snapshot->We_Open * Snapshot->We_Obs > 0.0f) &&
           Handover_Source_Stable(&Handover,
                                  Snapshot->We_Open,
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

static bool Coarse_Run(void)
{
    if (!Flux_Estimator_Run(&Flux_Estimator,
                            FLUX_EST_VECTOR,
                            FLUX_EST_UPDATE,
                            false,
                            Motor_Run.Ud,
                            Motor_Run.Uq,
                            Motor_Run.Id,
                            Motor_Run.Iq,
                            Flux_IF.State.We,
                            SPD_TS))
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
    U_Search_Max = Flux_Config.Observer.Work_U_Search_Ratio * Envelope->U_Max;
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
    We_Abs += Flux_Config.Observer.Target_Alpha * (We_Req - We_Abs);
    We_Target = (float)Dir * We_Abs;
    return true;
}

static bool Flux_IF_Target_Request(int8_t Dir, float We_Actual)
{
    if (!Flux_Target_Update(Dir, We_Actual))
    {
        return false;
    }

    IF_Acc_Command = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;
    IF_Target_Command = We_Target;
    __DMB();
    IF_Target_Pending = true;
    return true;
}

static void Flux_IF_Target_Apply(void)
{
    float Acc;
    float Target;

    if (!IF_Target_Pending)
    {
        return;
    }

    Acc = IF_Acc_Command;
    Target = IF_Target_Command;
    IF_Target_Pending = false;

    if (!__builtin_isfinite(Acc) || (Acc <= 0.0f) || !__builtin_isfinite(Target))
    {
        return;
    }

    Flux_IF.Para.Acc = Acc;
    IF_Target_Set(&Flux_IF, Target);
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
                            SPD_TS))
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

static void Slow_Snapshot_Publish(const Flux_Fast_Context_T *Context)
{
    Slow_Snapshot_Seq++;
    __DMB();
    Slow_Snapshot.Ialpha = Context->Ialpha;
    Slow_Snapshot.Ibeta = Context->Ibeta;
    Slow_Snapshot.Theta_Open = Context->Theta_Open;
    Slow_Snapshot.We_Open = Flux_IF.State.We;
    Slow_Snapshot.Theta_Obs = Context->Theta_Obs;
    Slow_Snapshot.We_Obs = We_Obs_F;
    Slow_Snapshot.PLL_Err = Ident_PLL.State.Err;
    Slow_Snapshot.Id_Open = Context->Id_Open;
    Slow_Snapshot.Iq_Open = Context->Iq_Open;
    Slow_Snapshot.Ud_Open = Context->Ud_Open;
    Slow_Snapshot.Uq_Open = Context->Uq_Open;
    Slow_Snapshot.PLL_Active = PLL_Active;
    __DMB();
    Slow_Snapshot_Seq++;
}

static bool Slow_Snapshot_Read(Flux_Slow_Snapshot_T *Snapshot)
{
    uint32_t Seq_Begin;
    uint32_t Seq_End;

    if (Snapshot == NULL)
    {
        return false;
    }

    do
    {
        Seq_Begin = Slow_Snapshot_Seq;
        if ((Seq_Begin & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        *Snapshot = Slow_Snapshot;
        __DMB();
        Seq_End = Slow_Snapshot_Seq;
    } while ((Seq_Begin != Seq_End) || ((Seq_End & 1U) != 0U));

    return Seq_End != 0U;
}

static void Flux_Context_Base_Init(Flux_Fast_Context_T *Context, float Ia_A, float Ib_A)
{
    Context->Envelope = Identification_Envelope_Get();
    Context->Ialpha = Ia_A;
    Context->Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Context->Theta_Obs = Ident_PLL.State.Theta;
    Context->Dir = (We_Target < 0.0f) ? -1 : 1;
}

static void Flux_Context_Open_Init(Flux_Fast_Context_T *Context)
{
    float Sin;
    float Cos;

    Context->Theta_Open = Flux_IF.State.Theta_e;
    SinCos(Context->Theta_Open, &Sin, &Cos);
    Context->Id_Open = Context->Ialpha * Cos + Context->Ibeta * Sin;
    Context->Iq_Open = -Context->Ialpha * Sin + Context->Ibeta * Cos;
    Context->Id_Ref_Open = 0.0f;
    Context->Iq_Ref_Open = 0.0f;
    Context->Ud_Open = Motor_Run.Ualpha * Cos + Motor_Run.Ubeta * Sin;
    Context->Uq_Open = -Motor_Run.Ualpha * Sin + Motor_Run.Ubeta * Cos;
}

static void Flux_Observer_Runtime_Run(Flux_Fast_Context_T *Context)
{
    if (!Obs_Active)
    {
        return;
    }

    if (!Flux_Observer_Run(&Ident_Observer,
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
        Context->Theta_Obs = Ident_PLL.State.Theta;
        return;
    }

    if (PLL_Active)
    {
        if (!PLL_Run(&Ident_PLL,
                     Ident_Observer.State.PsiAlpha,
                     Ident_Observer.State.PsiBeta,
                     CUR_TS))
        {
            if (!Obs_Control)
            {
                PLL_Active = false;
                Handover_Ready_Cnt = 0U;
                Motion_Lost_Cnt = 0U;
                Handover_Compare_Reset(&Handover);
            }
            Context->Theta_Obs = Ident_PLL.State.Theta;
            return;
        }
        We_Obs_F += Flux_Config.Observer.We_Alpha * (Ident_PLL.State.We - We_Obs_F);
    }

    Context->Theta_Obs = Ident_PLL.State.Theta;
}

static Motor_Fast_Mode_e Flux_Align_Run(int8_t Dir, float *Id_Ref, float *Iq_Ref)
{
    if (Align_Current(Start_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
    {
        Current_Loop_State_Reset();
        IF_Init(&Flux_IF, -0.5f * PI_F * (float)Dir, 0.0f);
        Flux_IF.State.Iq = (float)Dir * Start_Para.Iq_Start_A;
        IF_Target_Set(&Flux_IF, We_Target);
        if (Flux_IF.State.Mode == IF_FAILED)
        {
            return Flux_Fail_Off();
        }
        Current_Control_Active = true;
        State = FLUX_IF;
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
        IF_Run(&Flux_IF,
               Context->Id_Open,
               Context->Iq_Open,
               Context->Ud_Open,
               Context->Uq_Open,
               Theta_e,
               Id_Ref,
               Iq_Ref,
               CUR_TS);
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
    uint32_t T0;
    uint32_t Cyc;
    float Theta_Rough;
    bool Profile;

    Profile = (State == FLUX_OBS_WAIT);

    Flux_IF_Target_Apply();

    T0 = DWT->CYCCNT;
    IF_Run(&Flux_IF,
           Context->Id_Open,
           Context->Iq_Open,
           Context->Ud_Open,
           Context->Uq_Open,
           Theta_e,
           Id_Ref,
           Iq_Ref,
           CUR_TS);
    if (Profile)
    {
        Cyc = DWT->CYCCNT - T0;
        Flux_Time_Set(Cyc, &Flux_Time.IF_Cyc, &Flux_Time.IF_Max);
    }
    if (Flux_IF.State.Mode == IF_FAILED)
    {
        Flux_Fail_Off();
        return FLUX_STEP_OFF;
    }

    Context->Theta_Open = *Theta_e;
    Context->Id_Ref_Open = *Id_Ref;
    Context->Iq_Ref_Open = *Iq_Ref;

    if ((State == FLUX_IF) && (Flux_IF.State.Mode == IF_HOLD))
    {
        Motion_Lost_Armed = false;
        Motion_Lost_Cnt = 0U;
        State = FLUX_OBS_WAIT;
    }

    /* PLL recovery is exceptional and remains in the fast path so the open-loop
     * source keeps continuous control while the observer is rebuilt. */
    if (Obs_Active && !PLL_Active)
    {
        Theta_Rough = Angle_Wrap(__builtin_atan2f(Ident_Observer.State.PsiBeta,
                                                  Ident_Observer.State.PsiAlpha));
        PLL_Reset(&Ident_PLL, Theta_Rough, Flux_IF.State.We);
        We_Obs_F = Flux_IF.State.We;
        Motion_Lost_Cnt = 0U;
        Handover_Ready_Cnt = 0U;
        Handover_Compare_Reset(&Handover);
        PLL_Active = true;
    }

    Context->Theta_Obs = Ident_PLL.State.Theta;

    Motor_Run.Theta_e = Context->Theta_Open;
    Motor_Run.Id = Context->Id_Open;
    Motor_Run.Iq = Context->Iq_Open;
    Motor_Run.Ud = Context->Ud_Open;
    Motor_Run.Uq = Context->Uq_Open;
    Model_U_Valid = true;
    Slow_Snapshot_Publish(Context);

    return FLUX_STEP_CONTINUE;
}

static Motor_Fast_Mode_e Flux_Handover_Blend_Run(Flux_Fast_Context_T *Context,
                                                  float *Theta_e,
                                                  float *Id_Ref,
                                                  float *Iq_Ref)
{
    float I_Max;

    if (Handover_Blend_Run(&Handover,
                           Flux_Config.Handover.Blend_Cnt,
                           Context->Theta_Open,
                           Context->Theta_Obs,
                           Context->Id_Ref_Open,
                           Context->Iq_Ref_Open,
                           Theta_e,
                           Id_Ref,
                           Iq_Ref))
    {
        Obs_Id_Ref = *Id_Ref;
        Obs_Iq_Ref = *Iq_Ref;
        I_Max = 0.35f * Start_Para.Iq_Max_A;
        Speed_Loop_Track(We_Target, Ident_PLL.State.We, Obs_Iq_Ref, -I_Max, I_Max);
        Fine_Begin();
        Obs_Control = true;
        State = FLUX_HANDOVER_CURRENT;
    }

    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Handover_Current_Run(const Flux_Fast_Context_T *Context,
                                                    float *Theta_e,
                                                    float *Id_Ref,
                                                    float *Iq_Ref)
{
    *Theta_e = Context->Theta_Obs;
    Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, Flux_Config.Handover.Id_Ramp_Step);
    *Id_Ref = Obs_Id_Ref;
    *Iq_Ref = Obs_Iq_Ref;
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Observer_Control_Run(const Flux_Fast_Context_T *Context,
                                                   float *Theta_e,
                                                   float *Id_Ref,
                                                   float *Iq_Ref)
{
    *Theta_e = Context->Theta_Obs;
    *Id_Ref = 0.0f;
    *Iq_Ref = Obs_Iq_Ref;
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

static void Flux_Speed_Control_Run(void)
{
    if (Start_Para.Iq_Max_A <= 0.0f)
    {
        return;
    }

    Obs_Iq_Ref = Speed_Loop(We_Target,
                            Ident_PLL.State.We,
                            -0.35f * Start_Para.Iq_Max_A,
                            0.35f * Start_Para.Iq_Max_A);
}

static bool Flux_Coarse_Control_Run(const Flux_Slow_Snapshot_T *Snapshot, bool Search)
{
    float Theta_Rough;
    bool Coarse_Valid;
    int8_t Dir;

    if (!Model_U_Valid)
    {
        return false;
    }

    Coarse_Valid = Coarse_Run();
    if (!__builtin_isfinite(Flux_Estimator.State.Psi_d) ||
        !__builtin_isfinite(Flux_Estimator.State.Psi_q))
    {
        Result.Valid = false;
        State = FLUX_FAILED;
        return false;
    }

    if (!Coarse_Valid)
    {
        return false;
    }

    Dir = (We_Target < 0.0f) ? -1 : 1;
    if (Search && !Work_Point_Reached)
    {
        if (Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
            (Emf_Ratio_F >= Flux_Config.Observer.Work_Ratio))
        {
            We_Target = (Snapshot != NULL) ? Snapshot->We_Open : Flux_IF.State.We;
            IF_Acc_Command = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;
            IF_Target_Command = We_Target;
            __DMB();
            IF_Target_Pending = true;
            Work_Point_Reached = true;
        }
        else
        {
            (void)Flux_IF_Target_Request(Dir,
                                         (Snapshot != NULL) ? Snapshot->We_Open : Flux_IF.State.We);
        }
    }

    if (!Obs_Active)
    {
        if (Snapshot == NULL)
        {
            return true;
        }

        Theta_Rough = Angle_Wrap(Snapshot->Theta_Open +
                                 __builtin_atan2f(Flux_Estimator.State.Psi_q,
                                                  Flux_Estimator.State.Psi_d));
        Ident_Observer.Para.Rs = Motor_Para.Rs;
        Ident_Observer.Para.Ls = Motor_Para.Ld;
        Obs_Para_Update();
        Ident_PLL.Para.Kp = Flux_Config.Observer.Pll_Kp;
        Ident_PLL.Para.Ki = Flux_Config.Observer.Pll_Ki;
        Flux_Observer_Reset(&Ident_Observer, Theta_Rough, Snapshot->Ialpha, Snapshot->Ibeta);
        PLL_Reset(&Ident_PLL, Theta_Rough, Snapshot->We_Open);
        We_Obs_F = Snapshot->We_Open;
        Handover_Ready_Cnt = 0U;
        Handover_Compare_Reset(&Handover);
        __DMB();
        PLL_Active = true;
        Obs_Active = true;
    }
    else
    {
        Obs_Para_Update();
    }

    return true;
}

static void Flux_Open_Supervision_Run(Flux_State_e State_Local,
                                      const Flux_Slow_Snapshot_T *Snapshot)
{
    (void)Flux_Coarse_Control_Run(Snapshot, State_Local == FLUX_OBS_WAIT);
    if ((State != State_Local) || (State_Local == FLUX_IF))
    {
        return;
    }

    if ((Snapshot != NULL) && Snapshot->PLL_Active)
    {
        Handover_Source_Compare(&Handover,
                                Snapshot->Theta_Open,
                                Snapshot->We_Open,
                                Snapshot->Theta_Obs,
                                Snapshot->We_Obs,
                                Snapshot->PLL_Err,
                                Flux_Config.Handover.Compare_Alpha);
    }

    Handover_Qualification_Accumulate(&Handover_Ready_Cnt,
                                      Flux_Config.Handover.Ready_Cnt,
                                      Obs_Open_Stable(Snapshot));
    if ((State == State_Local) &&
        (Handover_Ready_Cnt >= Flux_Config.Handover.Ready_Cnt))
    {
        Handover_Blend_Reset(&Handover);
        State = FLUX_HANDOVER_BLEND;
    }
}

static void Flux_Handover_Current_Control_Run(Flux_State_e State_Local)
{
    bool Adapt_Valid;

    Flux_Speed_Control_Run();
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Ident_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    (void)Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return;
    }

    if ((State == State_Local) && (Obs_Id_Ref == 0.0f))
    {
        Cnt = 0U;
        Fine_Flux_Pre = 0.0f;
        Fine_Stable_Cnt = 0U;
        State = Work_Point_Reached ? FLUX_REFINE_SETTLE : FLUX_OBS_ACCEL;
    }
}

static void Flux_Obs_Accel_Control_Run(Flux_State_e State_Local)
{
    bool Adapt_Valid;
    bool Fine_Valid;
    int8_t Dir;

    Flux_Speed_Control_Run();
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Ident_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    Fine_Valid = Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return;
    }

    if (Fine_Valid && Flux_Estimator.State.Estimate_Valid)
    {
        Dir = (We_Target < 0.0f) ? -1 : 1;
        (void)Flux_Target_Update(Dir, We_Obs_F);
    }

    if ((State == State_Local) && Emf_Valid && __builtin_isfinite(Emf_Ratio_F) &&
        (Emf_Ratio_F >= Flux_Config.Observer.Work_Ratio))
    {
        We_Target = We_Obs_F;
        Work_Point_Reached = true;
        Cnt = 0U;
        Fine_Flux_Pre = 0.0f;
        Fine_Stable_Cnt = 0U;
        State = FLUX_REFINE_SETTLE;
    }
}

static void Flux_Refine_Settle_Control_Run(Flux_State_e State_Local)
{
    bool Adapt_Valid;
    bool Fine_Valid;

    Flux_Speed_Control_Run();
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Ident_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    Fine_Valid = Fine_Run(Adapt_Valid, false);
    if (!Flux_Observer_Motion_Check())
    {
        return;
    }

    Handover_Qualification_Accumulate(&Cnt,
                                      Flux_Config.Fine.Settle_Cnt,
                                      Fine_Valid && Obs_Control_Stable());
    if ((State == State_Local) && (Cnt >= Flux_Config.Fine.Settle_Cnt))
    {
        State = FLUX_REFINE_MEASURE;
    }
}

static void Flux_Refine_Measure_Control_Run(Flux_State_e State_Local)
{
    const Ident_Envelope_T *Envelope;
    float Flux_Fine;
    float U_Mag2;
    bool Adapt_Valid;
    bool Fine_Valid;
    bool Fine_Sample_Valid;

    Envelope = Identification_Envelope_Get();
    Flux_Speed_Control_Run();
    Handover_Speed_Compare(&Handover,
                           We_Target,
                           We_Obs_F,
                           Ident_PLL.State.Err,
                           Flux_Config.Handover.Compare_Alpha);

    Adapt_Valid = Obs_Run_Valid();
    U_Mag2 = Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq;
    Fine_Sample_Valid = Adapt_Valid &&
                        Obs_Control_Stable() &&
                        (U_Mag2 <= Envelope->U_Max * Envelope->U_Max);
    Fine_Valid = Fine_Run(Adapt_Valid, Fine_Sample_Valid);
    if (!Flux_Observer_Motion_Check())
    {
        return;
    }

    if (!Fine_Valid || !Flux_Estimator.State.Window_Ready)
    {
        return;
    }

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
    if ((State == State_Local) &&
        (Fine_Stable_Cnt >= Flux_Config.Fine.Stable_Windows))
    {
        Result.Flux_Wb = 0.5f * (Fine_Flux_Pre + Flux_Fine);
        Result.Valid = true;
        Flux_Finish_Start();
    }
    Fine_Flux_Pre = Flux_Fine;
}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;
    float We_Max;

    Result = (Flux_Result_T){ 0 };
    Start_Para = (Motor_IF_Para_T){ 0 };
    Flux_IF = (IF_T){ 0 };
    Flux_Time = (Flux_Time_T){ 0 };
    We_Target = 0.0f;
    Cnt = 0U;
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
    Slow_Snapshot_Seq = 0U;
    Slow_Snapshot = (Flux_Slow_Snapshot_T){ 0 };
    IF_Target_Command = 0.0f;
    IF_Acc_Command = 0.0f;
    IF_Target_Pending = false;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };
    Handover_Reset(&Handover);

    Envelope = Identification_Envelope_Get();
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if ((Envelope->I_Max <= 0.0f) || (Envelope->U_Max <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Max <= 0.0f) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &Start_Para) ||
        !__builtin_isfinite(Ident_IF_Current_A) ||
        (Ident_IF_Current_A <= 0.0f) ||
        (Ident_IF_Current_A > Envelope->I_Max))
    {
        State = FLUX_FAILED;
        return false;
    }

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * ((Start_Para.We_Base < We_Max) ? Start_Para.We_Base : We_Max);

    Flux_IF.Para.Iq_Min_A = Ident_IF_Current_A;
    Flux_IF.Para.Iq_Max_A = Start_Para.Iq_Max_A;
    Flux_IF.Para.We_Base = Start_Para.We_Base;
    Flux_IF.Para.Acc = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;
    Flux_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;
    Flux_IF.Para.Rs_Ohm = Motor_Para.Rs;
    Flux_IF.Para.Ld_H = Motor_Para.Ld;
    Flux_IF.Para.Lq_H = Motor_Para.Lq;

    Flux_Estimator.Para.Rs = Motor_Para.Rs;
    Flux_Estimator.Para.Ld = Motor_Para.Ld;
    Flux_Estimator.Para.Lq = Motor_Para.Lq;
    Flux_Estimator.Para.I_BW_Hz = Flux_Config.Coarse.I_BW_Hz;
    Flux_Estimator.Para.Est_BW_Hz = Flux_Config.Coarse.Est_BW_Hz;
    Flux_Estimator.Para.We_Min = Flux_Config.Coarse.We_Min_Ratio * Start_Para.We_Base;
    Flux_Estimator.Para.Window_Update_Ratio = 0.0f;
    Flux_Estimator.Para.Window_Samples = 0U;

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
    Flux_State_e State_Local;
    Flux_Slow_Snapshot_T Snapshot;
    Flux_Slow_Snapshot_T *Snapshot_Ptr;

    State_Local = State;
    if ((State_Local == FLUX_IDLE) || (State_Local == FLUX_ALIGN) ||
        (State_Local == FLUX_HANDOVER_BLEND) || (State_Local == FLUX_FINISH) ||
        (State_Local == FLUX_DONE) || (State_Local == FLUX_FAILED))
    {
        return;
    }

    Snapshot_Ptr = Slow_Snapshot_Read(&Snapshot) ? &Snapshot : NULL;

    switch (State_Local)
    {
        case FLUX_IF:
        case FLUX_OBS_WAIT:
            Flux_Open_Supervision_Run(State_Local, Snapshot_Ptr);
            break;

        case FLUX_HANDOVER_CURRENT:
            Flux_Handover_Current_Control_Run(State_Local);
            break;

        case FLUX_OBS_ACCEL:
            Flux_Obs_Accel_Control_Run(State_Local);
            break;

        case FLUX_REFINE_SETTLE:
            Flux_Refine_Settle_Control_Run(State_Local);
            break;

        case FLUX_REFINE_MEASURE:
            Flux_Refine_Measure_Control_Run(State_Local);
            break;

        default:
            break;
    }
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
    uint32_t T0;
    uint32_t Cyc;
    bool Need_Open_Context;
    bool Profile;

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

    Flux_Time.State = (uint32_t)State;
    Profile = (State == FLUX_OBS_WAIT);

    Flux_Context_Base_Init(&Context, Ia_A, Ib_A);

    Need_Open_Context = (State == FLUX_IF) ||
                        (State == FLUX_OBS_WAIT) ||
                        (State == FLUX_HANDOVER_BLEND) ||
                        ((State == FLUX_FINISH) && !Obs_Control);
    if (Need_Open_Context)
    {
        T0 = DWT->CYCCNT;
        Flux_Context_Open_Init(&Context);
        if (Profile)
        {
            Cyc = DWT->CYCCNT - T0;
            Flux_Time_Set(Cyc,
                          &Flux_Time.Context_Open_Cyc,
                          &Flux_Time.Context_Open_Max);
        }
    }

    if (State == FLUX_ALIGN)
    {
        return Flux_Align_Run(Context.Dir, Id_Ref, Iq_Ref);
    }

    T0 = DWT->CYCCNT;
    Flux_Observer_Runtime_Run(&Context);
    if (Profile)
    {
        Cyc = DWT->CYCCNT - T0;
        Flux_Time_Set(Cyc, &Flux_Time.Observer_Cyc, &Flux_Time.Observer_Max);
    }

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

    if ((State == FLUX_IF) ||
        (State == FLUX_OBS_WAIT) ||
        (State == FLUX_HANDOVER_BLEND))
    {
        Step = Flux_Open_Loop_Run(&Context, Theta_e, Id_Ref, Iq_Ref);
        if (Step == FLUX_STEP_OFF)
        {
            return FAST_OFF;
        }
        if (State == FLUX_FINISH)
        {
            return Flux_Finish_Run(&Context, Theta_e, Id_Ref, Iq_Ref);
        }
    }

    switch (State)
    {
        case FLUX_IF:
        case FLUX_OBS_WAIT:
            return FAST_CURRENT;

        case FLUX_HANDOVER_BLEND:
            return Flux_Handover_Blend_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_HANDOVER_CURRENT:
            return Flux_Handover_Current_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_OBS_ACCEL:
        case FLUX_REFINE_SETTLE:
        case FLUX_REFINE_MEASURE:
            return Flux_Observer_Control_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        default:
            return Flux_Fail_Off();
    }
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
