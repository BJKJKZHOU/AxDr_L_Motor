/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "JB.h"

#include <stddef.h>
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
#include "main.h"

#define JB_OPEN_ACCEL_S        6.0f
#define JB_OBSERVER_BW_HZ      200.0f
#define JB_PLL_BW_HZ           50.0f
#define JB_PLL_DAMP            0.707f
#define JB_PLL_WN              (TWO_PI_F * JB_PLL_BW_HZ)
#define JB_PLL_KP              (2.0f * JB_PLL_DAMP * JB_PLL_WN)
#define JB_PLL_KI              (JB_PLL_WN * JB_PLL_WN)
#define JB_OBS_WE_ALPHA        (CUR_TS / (0.020f + CUR_TS))

#define JB_HANDOVER_READY_CNT  ((uint32_t)(0.20f / SPD_TS + 0.5f))
#define JB_HANDOVER_BLEND_CNT  ((uint32_t)(0.15f / CUR_TS + 0.5f))
#define JB_HANDOVER_ID_STEP    (5.0f * CUR_TS)
#define JB_HANDOVER_ALPHA      (SPD_TS / (0.050f + SPD_TS))
#define JB_HANDOVER_WE_MEAN    0.010f
#define JB_HANDOVER_WE_RMS     0.025f
#define JB_HANDOVER_PLL_RMS    0.08f
#define JB_HANDOVER_THETA_RMS  0.20f

#define JB_WORK_RATIO          0.30f
#define JB_SETTLE_CNT          ((uint32_t)(0.50f / SPD_TS + 0.5f))
#define JB_EXCITE_HZ           (Ident_JB_Excite_Hz)
#define JB_EXCITE_CYCLES       3U
#define JB_MEASURE_CYCLES      10U
#define JB_WM_EXCITE_RATIO     (Ident_JB_Excite_Ratio)
#define JB_IQ_CONTROL_RATIO    0.35f
#define JB_FINISH_IQ_SLEW_A_S  20.0f
#define JB_FINISH_CNT          ((uint32_t)(0.10f / CUR_TS + 0.5f))

typedef enum
{
    JB_IDLE = 0,
    JB_ALIGN,
    JB_IF_RUN,
    JB_OBS_WAIT,
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
    uint32_t Samples;

} JB_DFT_T;

typedef struct
{
    float Theta_Open;
    float We_Open;
    float Theta_Obs;
    float We_Obs;
    float PLL_Err;
    bool PLL_Active;

} JB_Slow_Snapshot_T;

static volatile JB_State_e State = JB_IDLE;
static JB_Result_T Result = { 0 };
static Motor_IF_Para_T Start_Para = { 0 };
static IF_T JB_IF = { 0 };
static Handover_T Handover = { 0 };
static JB_DFT_T DFT = { 0 };

static float We_Target = 0.0f;
static float We_Startup = 0.0f;
static float Wm_Bias = 0.0f;
static float Wm_Amp = 0.0f;
static volatile float We_Obs_F = 0.0f;
static volatile float Obs_Id_Ref = 0.0f;
static volatile float Obs_Iq_Ref = 0.0f;
static float Excite_Phase = 0.0f;
static float Excite_Phase_Step = 0.0f;
static uint32_t Samples_Per_Cycle = 0U;
static uint32_t Phase_Samples = 0U;
static uint32_t Handover_Ready_Cnt = 0U;
static uint32_t Settle_Cnt = 0U;
static uint32_t Finish_Cnt = 0U;
static float Finish_Iq = 0.0f;
static bool Finish_Init = false;
static volatile bool Model_U_Valid = false;
static volatile bool PLL_Active = false;
static volatile bool Obs_Control = false;

/* Match the validated Flux workflow boundary: the 20 kHz fast path publishes
 * one coherent open-loop/observer sample and the 2 kHz supervisory path only
 * consumes complete snapshots. */
static volatile uint32_t Slow_Snapshot_Seq = 0U;
static JB_Slow_Snapshot_T Slow_Snapshot = { 0 };

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void JB_Fail(void)
{
    Result.Valid = false;
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

static void Slow_Snapshot_Publish(float Theta_Open)
{
    Slow_Snapshot_Seq++;
    __DMB();
    Slow_Snapshot.Theta_Open = Theta_Open;
    Slow_Snapshot.We_Open = JB_IF.State.We;
    Slow_Snapshot.Theta_Obs = Ident_PLL.State.Theta;
    Slow_Snapshot.We_Obs = We_Obs_F;
    Slow_Snapshot.PLL_Err = Ident_PLL.State.Err;
    Slow_Snapshot.PLL_Active = PLL_Active;
    __DMB();
    Slow_Snapshot_Seq++;
}

static bool Slow_Snapshot_Read(JB_Slow_Snapshot_T *Snapshot, JB_State_e *State_Out)
{
    uint32_t Primask;
    bool Valid;

    if ((Snapshot == NULL) || (State_Out == NULL))
    {
        return false;
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    *State_Out = State;
    Valid = Slow_Snapshot_Seq != 0U;
    *Snapshot = Slow_Snapshot;
    Snapshot->We_Open = JB_IF.State.We;
    Snapshot->Theta_Obs = Ident_PLL.State.Theta;
    Snapshot->We_Obs = We_Obs_F;
    Snapshot->PLL_Err = Ident_PLL.State.Err;
    Snapshot->PLL_Active = PLL_Active;
    __DMB();
    __set_PRIMASK(Primask);

    return Valid;
}

static void Observer_Runtime_Run(float Ialpha, float Ibeta, float *Theta_Obs)
{
    if (!Model_U_Valid)
    {
        *Theta_Obs = Ident_PLL.State.Theta;
        return;
    }

    if (!Flux_Observer_Run(&Ident_Observer,
                           Motor_Run.Ualpha,
                           Motor_Run.Ubeta,
                           Ialpha,
                           Ibeta,
                           CUR_TS))
    {
        *Theta_Obs = Ident_PLL.State.Theta;
        return;
    }

    if (PLL_Run(&Ident_PLL,
                Ident_Observer.State.PsiAlpha,
                Ident_Observer.State.PsiBeta,
                CUR_TS))
    {
        PLL_Active = true;
        We_Obs_F += JB_OBS_WE_ALPHA * (Ident_PLL.State.We - We_Obs_F);
    }

    *Theta_Obs = Ident_PLL.State.Theta;
}

static bool Observer_Open_Stable(const JB_Slow_Snapshot_T *Snapshot)
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
                                  JB_HANDOVER_WE_MEAN,
                                  JB_HANDOVER_WE_RMS,
                                  JB_HANDOVER_PLL_RMS,
                                  JB_HANDOVER_THETA_RMS);
}

static void Open_Supervision_Run(JB_State_e State_Local,
                                 const JB_Slow_Snapshot_T *Snapshot)
{
    if ((State_Local != JB_OBS_WAIT) || (Snapshot == NULL))
    {
        return;
    }

    if (Snapshot->PLL_Active)
    {
        Handover_Source_Compare(&Handover,
                                Snapshot->Theta_Open,
                                Snapshot->We_Open,
                                Snapshot->Theta_Obs,
                                Snapshot->We_Obs,
                                Snapshot->PLL_Err,
                                JB_HANDOVER_ALPHA);
    }

    Handover_Qualification_Accumulate(&Handover_Ready_Cnt,
                                      JB_HANDOVER_READY_CNT,
                                      Observer_Open_Stable(Snapshot));
    if ((State == State_Local) &&
        (Handover_Ready_Cnt >= JB_HANDOVER_READY_CNT))
    {
        Handover_Blend_Reset(&Handover);
        State = JB_HANDOVER_BLEND;
    }
}

static void Speed_Control_Run(float We_Ref)
{
    float I_Max;

    I_Max = JB_IQ_CONTROL_RATIO * Start_Para.Iq_Max_A;
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
        Wm = Ident_PLL.State.We / (float)Motor_Para.Pp;
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
        State = JB_MEASURE;
    }
    else
    {
        Result_Calculate();
        State = JB_FINISH;
        Finish_Init = false;
        Finish_Cnt = 0U;
    }
}

bool JB_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float We_Max;
    float We_Work_Max;

    (void)Wm_Target;

    Result = (JB_Result_T){ 0 };
    Start_Para = (Motor_IF_Para_T){ 0 };
    JB_IF = (IF_T){ 0 };
    Handover_Reset(&Handover);
    DFT_Reset();

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
        !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &Start_Para))
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

    JB_IF.Para.Iq_Min_A = Start_Para.Iq_Start_A;
    JB_IF.Para.Iq_Max_A = Start_Para.Iq_Max_A;
    JB_IF.Para.We_Base = Start_Para.We_Base;
    JB_IF.Para.Acc = Abs_Value(We_Startup) / JB_OPEN_ACCEL_S;
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
        State = JB_FAILED;
        return false;
    }

    Excite_Phase_Step = TWO_PI_F / (float)Samples_Per_Cycle;

    State = JB_ALIGN;
    Handover_Ready_Cnt = 0U;
    Settle_Cnt = 0U;
    Finish_Cnt = 0U;
    Finish_Init = false;
    Model_U_Valid = false;
    PLL_Active = false;
    Obs_Control = false;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Excite_Phase = 0.0f;
    Phase_Samples = 0U;
    We_Obs_F = 0.0f;
    Slow_Snapshot_Seq = 0U;
    Slow_Snapshot = (JB_Slow_Snapshot_T){ 0 };

    Align_Reset();
    Current_Loop_State_Reset();
    return true;
}

void JB_Abort(void)
{
    Result.Valid = false;
    State = JB_IDLE;
}

bool JB_Active(void)
{
    return (State != JB_IDLE) && (State != JB_DONE) && (State != JB_FAILED);
}

void JB_Control(void)
{
    JB_State_e State_Local;
    JB_Slow_Snapshot_T Snapshot;

    if (!JB_Active())
    {
        return;
    }

    if ((State == JB_IF_RUN) || (State == JB_OBS_WAIT) ||
        (State == JB_HANDOVER_BLEND))
    {
        if (!Slow_Snapshot_Read(&Snapshot, &State_Local))
        {
            return;
        }
        Open_Supervision_Run(State_Local, &Snapshot);
        return;
    }

    if (State == JB_HANDOVER_CURRENT)
    {
        Speed_Control_Run(We_Target);
        if ((Obs_Id_Ref == 0.0f) && (++Settle_Cnt >= JB_SETTLE_CNT))
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

    if (!JB_Active())
    {
        return FAST_OFF;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;
    Dir = (We_Target < 0.0f) ? -1 : 1;

    if (State == JB_ALIGN)
    {
        if (Align_Current(Start_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Init(&JB_IF, -0.5f * PI_F * (float)Dir, 0.0f);
            JB_IF.State.Iq = (float)Dir * Start_Para.Iq_Start_A;
            IF_Target_Set(&JB_IF, We_Startup);
            if (JB_IF.State.Mode == IF_FAILED)
            {
                JB_Fail();
                return FAST_OFF;
            }

            Flux_Observer_Reset(&Ident_Observer, JB_IF.State.Theta_e, Ialpha, Ibeta);
            PLL_Reset(&Ident_PLL, JB_IF.State.Theta_e, 0.0f);
            We_Obs_F = 0.0f;
            Model_U_Valid = false;
            PLL_Active = false;
            State = JB_IF_RUN;
        }
        return FAST_CURRENT;
    }

    Theta_Obs = Ident_PLL.State.Theta;
    Observer_Runtime_Run(Ialpha, Ibeta, &Theta_Obs);

    if ((State == JB_IF_RUN) || (State == JB_OBS_WAIT) ||
        (State == JB_HANDOVER_BLEND))
    {
        Theta_Open = JB_IF.State.Theta_e;
        SinCos(Theta_Open, &Sin, &Cos);
        Id_Open = Ialpha * Cos + Ibeta * Sin;
        Iq_Open = -Ialpha * Sin + Ibeta * Cos;
        Ud_Open = Motor_Run.Ualpha * Cos + Motor_Run.Ubeta * Sin;
        Uq_Open = -Motor_Run.Ualpha * Sin + Motor_Run.Ubeta * Cos;

        if (State == JB_IF_RUN)
        {
            JB_IF.Para.Acc = Abs_Value(We_Startup) / JB_OPEN_ACCEL_S *
                             ((Abs_Value(JB_IF.State.We) < 0.50f * Start_Para.We_Base) ?
                                  3.0f : 0.60f);
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
            JB_Fail();
            return FAST_OFF;
        }

        Model_U_Valid = true;
        Slow_Snapshot_Publish(Theta_Open);

        if ((State == JB_IF_RUN) && (JB_IF.State.Mode == IF_HOLD))
        {
            JB_IF.Para.Acc = Abs_Value(We_Target) / JB_OPEN_ACCEL_S;
            IF_Target_Set(&JB_IF, We_Target);
            Handover_Ready_Cnt = 0U;
            Handover_Compare_Reset(&Handover);
            State = JB_OBS_WAIT;
        }

        if ((State == JB_IF_RUN) || (State == JB_OBS_WAIT))
        {
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
                I_Max = JB_IQ_CONTROL_RATIO * Start_Para.Iq_Max_A;
                Speed_Loop_Track(We_Target,
                                 Ident_PLL.State.We,
                                 Obs_Iq_Ref,
                                 -I_Max,
                                 I_Max);
                Settle_Cnt = 0U;
                Obs_Control = true;
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
                State = Result.Valid ? JB_DONE : JB_FAILED;
                return FAST_OFF;
            }
        }
        else
        {
            Finish_Cnt = 0U;
        }
        return FAST_CURRENT;
    }

    if (Obs_Control && ((State == JB_EXCITE) ||
                        (State == JB_MEASURE)))
    {
        *Theta_e = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        return FAST_CURRENT;
    }

    JB_Fail();
    return FAST_OFF;
}

const JB_Result_T *JB_Result_Get(void)
{
    return &Result;
}
