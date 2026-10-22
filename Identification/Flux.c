/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

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
#include "Motor_Config.h"
#include "Motion_Loop.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Ramp.h"
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
        float Est_BW_Hz;
        float Stable_Ratio;
        float Update_Ratio;
    } Fine;

    struct
    {
        float Work_Ratio;
        float Work_U_Search_Ratio;
        float We_Alpha;
        float Emf_Alpha;
    } Observer;

    struct
    {
        uint32_t Blend_Cnt;
        float Id_Ramp_Step;
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
        .Est_BW_Hz = 5.0f,
        .Stable_Ratio = 0.02f,
        .Update_Ratio = 0.25f,
    },
    .Observer = {
        .Work_Ratio = 0.30f,
        .Work_U_Search_Ratio = 0.90f,
        .We_Alpha = CUR_TS / (0.020f + CUR_TS),
        .Emf_Alpha = SPD_TS / (0.020f + SPD_TS),
    },
    .Handover = {
        .Blend_Cnt = (uint32_t)(0.15f / CUR_TS + 0.5f),
        .Id_Ramp_Step = 5.0f * CUR_TS,
    },
};

typedef enum
{
    FLUX_IDLE = 0,
    FLUX_ALIGN,
    FLUX_IF,
    FLUX_HANDOVER_BLEND,
    FLUX_HANDOVER_CURRENT,
    FLUX_OBS_ACCEL,
    FLUX_REFINE_SETTLE,
    FLUX_REFINE_MEASURE,
    FLUX_FINISH,

} Flux_State_e;

typedef struct
{
    float Ialpha;
    float Ibeta;
    float Theta_Open;
    float Theta_Obs;
    float Id_Ref_Open;
    float Iq_Ref_Open;

} Flux_Fast_Context_T;

typedef struct
{
    float Ialpha;
    float Ibeta;
    float Theta_Open;
    float We_Open;
    float We_Obs;
    float Id;
    float Iq;
    float Ud;
    float Uq;

} Flux_Slow_Snapshot_T;

static volatile Flux_State_e State = FLUX_IDLE;
Flux_Result_T Flux_Result = { 0 };
static Motor_IF_Para_T Start_Para = { 0 };
static IF_T Flux_IF = { 0 };
static Flux_Estimator_T Flux_Estimator = { 0 };
static Handover_T Handover = { 0 };
static Motion_Ref_T Flux_Speed_Ref = { 0 };
static volatile float We_Target = 0.0f;
static uint32_t Cnt = 0U;
static volatile float We_Obs_F = 0.0f;
static float Emf_Ratio_F = 0.0f;
static volatile float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Fine_Flux_Pre = 0.0f;
static float Finish_Iq = 0.0f;
static volatile bool Model_U_Valid = false;
static bool Emf_Valid = false;
static volatile bool Obs_Active = false;
static volatile bool PLL_Active = false;

/* Only open-loop source signals need a cross-context snapshot.
 * Observer and motor feedback are read in the slow task's IRQ lock. */
static struct
{
    float Ialpha;
    float Ibeta;
    float Theta_Open;
} Slow_Snapshot = { 0 };

/* Initial two-stage I/F acceleration; 30% work point is found after takeover. */
static float IF_Acc_Command = 0.0f;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static Motor_Fast_Mode_e Flux_Fail_Off(void)
{
    Flux_Result.Valid = false;
    Identification_Report(false);
    return FAST_OFF;
}

static bool Emf_Update(float Yd, float Yq)
{
    const Ident_Envelope_T *Envelope;
    float Ratio;

    Envelope = (&Ident_Envelope);
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
    Flux_Observer_Para_T Para;
    Flux_Observer_Calc_T Calc;
    Flux_State_e State_Local = State;
    uint32_t Primask;
    float Flux;

    /* Coarse fitting still uses the open-loop speed during angle blending. */
    if (State_Local == FLUX_HANDOVER_BLEND)
    {
        return;
    }

    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= Flux_Config.Workflow.Est_Num_Min_Wb))
    {
        return;
    }

    Para = Ident_Observer.Para;
    Calc = Ident_Observer.Calc;
    Para.Flux = Flux;
    Para.BW_Hz = Control_Flux_Obs_Bw_Hz;
    (void)Flux_Observer_Calc_Update(&Para, &Calc);

    /* Publish matching flux and gain only if the sampling stage is unchanged. */
    Primask = __get_PRIMASK();
    __disable_irq();
    if (State == State_Local)
    {
        Ident_Observer.Para = Para;
        Ident_Observer.Calc = Calc;
    }
    __DMB();
    __set_PRIMASK(Primask);
}

static void Flux_Target_Update(void)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Abs;
    float U_Search_Max;
    float U_Mag;
    float We_U_Max;
    float We_Req;
    float Flux;

    Envelope = (&Ident_Envelope);
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if ((Envelope->U_Available <= 0.0f) || (Envelope->U_Max <= 0.0f) ||
        !__builtin_isfinite(We_Max) || (We_Max <= 0.0f))
    {
        return;
    }

    We_Abs = Abs_Value(We_Obs_F);
    Flux = Flux_Estimator.State.Flux;
    if (!__builtin_isfinite(Flux) || (Flux <= Flux_Config.Workflow.Est_Num_Min_Wb))
    {
        return;
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

    Ramp_Run(&Flux_Speed_Ref,
             ((We_Target < 0.0f) ? -We_Req : We_Req) / (float)Motor_Para.Pp,
             Motion_Config.Wm_Acc,
             Motion_Config.Wm_Dec,
             SPD_TS);
    We_Target = (float)Motor_Para.Pp * Flux_Speed_Ref.Wm;
}

static void Fine_Begin(void)
{
    float Flux_Init;

    Flux_Init = Flux_Estimator.State.Flux;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };
    Flux_Estimator.State.Flux = Flux_Init;
    Flux_Estimator.Para.Est_BW_Hz = Flux_Config.Fine.Est_BW_Hz;
    Flux_Estimator.Para.Window_Update_Ratio = Flux_Config.Fine.Update_Ratio;
    Flux_Estimator.Para.Window_Samples = Flux_Config.Fine.Measure_Cnt;
}

static bool Fine_Run(const Flux_Slow_Snapshot_T *Snapshot, bool Measure)
{
    float We;

    We = Snapshot->We_Obs;
    if (!Flux_Estimator_Run(&Flux_Estimator,
                            FLUX_EST_SCALAR,
                            Measure,
                            Snapshot->Ud,
                            Snapshot->Uq,
                            Snapshot->Id,
                            Snapshot->Iq,
                            We,
                            SPD_TS))
    {
        return false;
    }

    if (!Emf_Update(Flux_Estimator.State.Yd, Flux_Estimator.State.Yq))
    {
        return false;
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
    Slow_Snapshot.Ialpha = Context->Ialpha;
    Slow_Snapshot.Ibeta = Context->Ibeta;
    Slow_Snapshot.Theta_Open = Context->Theta_Open;
    __DMB();
    Model_U_Valid = true;
}

static bool Slow_Snapshot_Read(Flux_Slow_Snapshot_T *Snapshot, Flux_State_e *State_Out)
{
    uint32_t Primask;
    bool Valid;

    /* The Motor Thread can only run after the ADC ISR completes Current_Loop.
     * Copy its dq sample and speed with interrupts masked; no model calculation
     * runs in this section. This also pairs the sample with its Flux stage. */
    Primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    *State_Out = State;
    Valid = Model_U_Valid;
    Snapshot->Ialpha = Slow_Snapshot.Ialpha;
    Snapshot->Ibeta = Slow_Snapshot.Ibeta;
    Snapshot->Theta_Open = Slow_Snapshot.Theta_Open;
    Snapshot->Id = Motor_Run.Id;
    Snapshot->Iq = Motor_Run.Iq;
    Snapshot->Ud = Motor_Run.Ud;
    Snapshot->Uq = Motor_Run.Uq;
    Snapshot->We_Open = Flux_IF.State.We;
    Snapshot->We_Obs = We_Obs_F;
    __DMB();
    __set_PRIMASK(Primask);

    return Valid;
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
        Obs_Active = false;
        PLL_Active = false;
    }
    else if (!PLL_Active ||
             !PLL_Run(&Ident_PLL,
                       Ident_Observer.State.PsiAlpha,
                       Ident_Observer.State.PsiBeta,
                       CUR_TS))
    {
        PLL_Active = false;
    }
    else
    {
        We_Obs_F += Flux_Config.Observer.We_Alpha * (Ident_PLL.State.We - We_Obs_F);
        Context->Theta_Obs = Ident_PLL.State.Theta;
        return;
    }

    if (State != FLUX_IF)
    {
        (void)Flux_Fail_Off();
    }
}

static Motor_Fast_Mode_e Flux_Align_Run(int8_t Dir, float *Id_Ref, float *Iq_Ref)
{
    if (Align_Current(Motor_Config.Align_Current_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
    {
        Current_Loop_State_Reset();
        IF_Init(&Flux_IF, -0.5f * PI_F * (float)Dir, 0.0f);
        IF_Target_Set(&Flux_IF, We_Target);
        if (Flux_IF.State.Mode == IF_FAILED)
        {
            return Flux_Fail_Off();
        }
        State = FLUX_IF;
    }
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Flux_Finish_Run(const Flux_Fast_Context_T *Context,
                                         float *Theta_e,
                                         float *Id_Ref,
                                         float *Iq_Ref)
{
    *Theta_e = Context->Theta_Obs;
    Finish_Iq = Handover_Ramp_Zero(Finish_Iq, Flux_Config.Workflow.Finish_Iq_Slew_A_S * CUR_TS);

    *Id_Ref = 0.0f;
    *Iq_Ref = Finish_Iq;
    if (Finish_Iq == 0.0f)
    {
        if (++Cnt >= Flux_Config.Workflow.Finish_Cnt)
        {
            Identification_Report(true);
            return FAST_OFF;
        }
    }
    else
    {
        Cnt = 0U;
    }
    return FAST_CURRENT;
}

static bool Flux_Open_Loop_Run(Flux_Fast_Context_T *Context,
                                      float *Theta_e,
                                      float *Id_Ref,
                                      float *Iq_Ref)
{
    float Theta_Rough;

    if (State == FLUX_IF)
    {
        /* 6d26e0a preset: reach half base speed in 1 s, then base speed in
         * another 5 s. Only the initial ramp uses this fixed profile. */
        Flux_IF.Para.Acc = IF_Acc_Command *
                           ((Abs_Value(Flux_IF.State.We) < 0.50f * Start_Para.We_Base) ? 3.0f : 0.60f);
    }

    IF_Run(&Flux_IF,
           Theta_e,
           Id_Ref,
           Iq_Ref,
           CUR_TS);
    if (Flux_IF.State.Mode == IF_FAILED)
    {
        Flux_Fail_Off();
        return false;
    }

    Context->Theta_Open = *Theta_e;
    Context->Id_Ref_Open = *Id_Ref;
    Context->Iq_Ref_Open = *Iq_Ref;

    /* A recovered PLL must complete one fast update before a new blend. */
    if ((State == FLUX_IF) && (Flux_IF.State.Mode == IF_HOLD) && PLL_Active)
    {
        Handover_Blend_Reset(&Handover);
        State = FLUX_HANDOVER_BLEND;
    }

    /* Restart the PLL while I/F still owns the control angle. */
    if (Obs_Active && !PLL_Active)
    {
        Theta_Rough = Angle_Wrap(__builtin_atan2f(Ident_Observer.State.PsiBeta,
                                                  Ident_Observer.State.PsiAlpha));
        PLL_Reset(&Ident_PLL, Theta_Rough, Flux_IF.State.We);
        We_Obs_F = Flux_IF.State.We;
        PLL_Active = true;
    }

    Context->Theta_Obs = Ident_PLL.State.Theta;
    Slow_Snapshot_Publish(Context);

    return true;
}

static Motor_Fast_Mode_e Flux_Handover_Blend_Run(Flux_Fast_Context_T *Context,
                                                  float *Theta_e,
                                                  float *Id_Ref,
                                                  float *Iq_Ref)
{
    float I_Max = (&Ident_Envelope)->I_Max;

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
        Speed_Loop_Track(We_Target, Ident_PLL.State.We, Obs_Iq_Ref,
                         -I_Max, I_Max);
        State = FLUX_HANDOVER_CURRENT;
    }

    return FAST_CURRENT;
}

static void Flux_Coarse_Control_Run(const Flux_Slow_Snapshot_T *Snapshot)
{
    float Theta_Rough;
    bool Coarse_Valid;

    Coarse_Valid = Flux_Estimator_Run(&Flux_Estimator,
                                      FLUX_EST_VECTOR,
                                      false,
                                      Snapshot->Ud,
                                      Snapshot->Uq,
                                      Snapshot->Id,
                                      Snapshot->Iq,
                                      Snapshot->We_Open,
                                      SPD_TS);
    if (Coarse_Valid)
    {
        Coarse_Valid = Emf_Update(Flux_Estimator.State.Yd, Flux_Estimator.State.Yq) &&
                       Flux_Estimator.State.Estimate_Valid;
    }
    if (!__builtin_isfinite(Flux_Estimator.State.Psi_d) ||
        !__builtin_isfinite(Flux_Estimator.State.Psi_q))
    {
        Identification_Report(false);
        return;
    }

    if (!Coarse_Valid)
    {
        return;
    }

    if (!Obs_Active)
    {
        float Pll_Wn = TWO_PI_F * Control_PLL_Bw_Hz;

        Theta_Rough = Angle_Wrap(Snapshot->Theta_Open +
                                 __builtin_atan2f(Flux_Estimator.State.Psi_q,
                                                  Flux_Estimator.State.Psi_d));
        Ident_Observer.Para.Rs = Motor_Para.Rs;
        Ident_Observer.Para.Ls = Motor_Para.Ld;
        Obs_Para_Update();
        Ident_PLL.Para.Kp = 2.0f * Control_PLL_Damp * Pll_Wn;
        Ident_PLL.Para.Ki = Pll_Wn * Pll_Wn;
        Flux_Observer_Reset(&Ident_Observer, Theta_Rough, Snapshot->Ialpha, Snapshot->Ibeta);
        PLL_Reset(&Ident_PLL, Theta_Rough, Snapshot->We_Open);
        We_Obs_F = Snapshot->We_Open;
        __DMB();
        PLL_Active = true;
        Obs_Active = true;
    }
    else
    {
        Obs_Para_Update();
    }

}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;
    float We_Max;

    Flux_Result = (Flux_Result_T){ 0 };
    We_Obs_F = 0.0f;
    Emf_Ratio_F = 0.0f;
    Model_U_Valid = false;
    Emf_Valid = false;
    Obs_Active = false;
    PLL_Active = false;
    Flux_Estimator.State = (Flux_Estimator_State_T){ 0 };

    Envelope = (&Ident_Envelope);
    We_Max = (float)Motor_Para.Pp * Motor_Wm_Limit_Effective_Get();
    if (!__builtin_isfinite(We_Max) || (We_Max <= 0.0f) ||
        !Motor_IF_Para_Build(ADC.Vbus_V, Motor_Config.IF_Current_A, &Start_Para) ||
        !__builtin_isfinite(Motor_Config.Align_Current_A) ||
        (Motor_Config.Align_Current_A <= 0.0f) ||
        (Motor_Config.Align_Current_A > Envelope->I_Max) ||
        !__builtin_isfinite(Motor_Config.IF_Current_A) ||
        (Motor_Config.IF_Current_A <= 0.0f) ||
        (Motor_Config.IF_Current_A > Envelope->I_Max))
    {
        return false;
    }

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * ((Start_Para.We_Base < We_Max) ? Start_Para.We_Base : We_Max);

    Flux_IF.Para.Iq_Min_A = Motor_Config.IF_Current_A;
    Flux_IF.Para.Iq_Max_A = Start_Para.Iq_Max_A;
    Flux_IF.Para.We_Base = Start_Para.We_Base;
    IF_Acc_Command = Abs_Value(We_Target) / Flux_Config.Workflow.Open_Accel_S;
    Flux_IF.Para.Acc = IF_Acc_Command;
    Flux_IF.Para.Iq_Slew_A_S = IF_IQ_SLEW_A_S;

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

void Flux_Control(void)
{
    const Ident_Envelope_T *Envelope = (&Ident_Envelope);
    Flux_State_e State_Local;
    Flux_State_e Next_State;
    Flux_Slow_Snapshot_T Snapshot;
    bool Fine_Valid;
    bool Fine_Sample_Valid;
    float Flux_Value;
    uint32_t Primask;

    if (!Slow_Snapshot_Read(&Snapshot, &State_Local))
    {
        return;
    }

    switch (State_Local)
    {
        case FLUX_IF:
        case FLUX_HANDOVER_BLEND:
            Flux_Coarse_Control_Run(&Snapshot);
            return;

        case FLUX_HANDOVER_CURRENT:
        case FLUX_OBS_ACCEL:
        case FLUX_REFINE_SETTLE:
        case FLUX_REFINE_MEASURE:
            break;

        default:
            return;
    }

    /* Change the estimator mode in the 2 kHz context after coarse fitting. */
    if ((State_Local == FLUX_HANDOVER_CURRENT) &&
        (Flux_Estimator.Para.Window_Samples == 0U))
    {
        Fine_Begin();
    }

    Obs_Iq_Ref = Speed_Loop(We_Target,
                            Ident_PLL.State.We,
                            -Envelope->I_Max,
                            Envelope->I_Max);

    Fine_Sample_Valid = false;
    if (State_Local == FLUX_REFINE_MEASURE)
    {
        float U_Mag2 = Snapshot.Ud * Snapshot.Ud + Snapshot.Uq * Snapshot.Uq;

        Fine_Sample_Valid = (U_Mag2 <= Envelope->U_Max * Envelope->U_Max);
    }

    Fine_Valid = Fine_Run(&Snapshot, Fine_Sample_Valid);
    Next_State = State_Local;

    switch (State_Local)
    {
        case FLUX_HANDOVER_CURRENT:
            if (Obs_Id_Ref == 0.0f)
            {
                Ramp_Reset(&Flux_Speed_Ref, We_Target / (float)Motor_Para.Pp);
                Next_State = FLUX_OBS_ACCEL;
            }
            break;

        case FLUX_OBS_ACCEL:
            if (Fine_Valid)
            {
                Flux_Target_Update();
            }
            if (Emf_Valid && (Emf_Ratio_F >= Flux_Config.Observer.Work_Ratio))
            {
                We_Target = We_Obs_F;
                Cnt = 0U;
                Fine_Flux_Pre = 0.0f;
                Next_State = FLUX_REFINE_SETTLE;
            }
            break;

        case FLUX_REFINE_SETTLE:
            if (++Cnt >= Flux_Config.Fine.Settle_Cnt)
            {
                Next_State = FLUX_REFINE_MEASURE;
            }
            break;

        case FLUX_REFINE_MEASURE:
            if (Fine_Valid && Flux_Estimator.State.Window_Ready)
            {
                float Flux_Fine = Flux_Estimator.State.Window_Flux;
                if ((Fine_Flux_Pre > 0.0f) &&
                    (Abs_Value(Flux_Fine - Fine_Flux_Pre) <=
                     Flux_Config.Fine.Stable_Ratio * Fine_Flux_Pre))
                {
                    Flux_Value = 0.5f * (Fine_Flux_Pre + Flux_Fine);
                    Next_State = FLUX_FINISH;
                }
                Fine_Flux_Pre = Flux_Fine;
            }
            break;

        default:
            break;
    }

    /* Commit transitions and the final result only if the fast loop still owns
     * the same state; never overwrite a fault raised during slow calculations. */
    if (Next_State != State_Local)
    {
        Primask = __get_PRIMASK();
        __disable_irq();
        if (State == State_Local)
        {
            if (Next_State == FLUX_FINISH)
            {
                Flux_Result.Flux_Wb = Flux_Value;
                Flux_Result.Valid = true;
                Cnt = 0U;
                Finish_Iq = Obs_Iq_Ref;
            }
            State = Next_State;
        }
        __set_PRIMASK(Primask);
    }
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float *Theta_e,
                                float *Id_Ref,
                                float *Iq_Ref,
                                float *Ualpha_V,
                                float *Ubeta_V)
{
    Flux_Fast_Context_T Context;
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;
    Context.Ialpha = Ia_A;
    Context.Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Context.Theta_Obs = Ident_PLL.State.Theta;

    if (State == FLUX_ALIGN)
    {
        return Flux_Align_Run((We_Target < 0.0f) ? -1 : 1, Id_Ref, Iq_Ref);
    }

    Flux_Observer_Runtime_Run(&Context);

    if (State == FLUX_FINISH)
    {
        return Flux_Finish_Run(&Context, Theta_e, Id_Ref, Iq_Ref);
    }

    if ((&Ident_Envelope)->U_Max <= 0.0f)
    {
        return Flux_Fail_Off();
    }

    if ((State == FLUX_IF) || (State == FLUX_HANDOVER_BLEND))
    {
        if (!Flux_Open_Loop_Run(&Context, Theta_e, Id_Ref, Iq_Ref))
        {
            return FAST_OFF;
        }
    }

    switch (State)
    {
        case FLUX_IF:
            return FAST_CURRENT;

        case FLUX_HANDOVER_BLEND:
            return Flux_Handover_Blend_Run(&Context, Theta_e, Id_Ref, Iq_Ref);

        case FLUX_HANDOVER_CURRENT:
            Obs_Id_Ref = Handover_Ramp_Zero(Obs_Id_Ref, Flux_Config.Handover.Id_Ramp_Step);
            *Theta_e = Context.Theta_Obs;
            *Id_Ref = Obs_Id_Ref;
            *Iq_Ref = Obs_Iq_Ref;
            return FAST_CURRENT;

        case FLUX_OBS_ACCEL:
        case FLUX_REFINE_SETTLE:
        case FLUX_REFINE_MEASURE:
            *Theta_e = Context.Theta_Obs;
            *Id_Ref = 0.0f;
            *Iq_Ref = Obs_Iq_Ref;
            return FAST_CURRENT;

        default:
            return Flux_Fail_Off();
    }
}

