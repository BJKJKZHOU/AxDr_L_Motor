/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

#include <stddef.h>
#include <stdint.h>

#include "Align.h"
#include "Current_Loop.h"
#include "IF_Start.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "control_params.h"

#define FLUX_SETTLE_S           0.30f
#define FLUX_MEASURE_S          0.20f
#define FLUX_SETTLE_CNT         ((uint32_t)(FLUX_SETTLE_S / CUR_TS + 0.5f))
#define FLUX_MEASURE_CNT        ((uint32_t)(FLUX_MEASURE_S / CUR_TS + 0.5f))
#define FLUX_FINISH_IQ_SLEW_A_S 20.0f
#define FLUX_FINISH_I           0.20f
#define FLUX_FINISH_S           0.002f
#define FLUX_FINISH_CNT         ((uint32_t)(FLUX_FINISH_S / CUR_TS + 0.5f))

#define FLUX_EMF_TARGET_RATIO  0.35f
#define FLUX_EMF_MIN_RATIO     0.20f
#define FLUX_WE_STEP_MAX_RATIO 1.60f
#define FLUX_U_SEARCH_RATIO    0.90f

typedef enum
{
    FLUX_IDLE = 0,
    FLUX_ALIGN,
    FLUX_ACCEL,
    FLUX_SETTLE,
    FLUX_MEASURE,
    FLUX_FINISH,
    FLUX_DONE,
    FLUX_FAILED,

} Flux_State_e;

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T IF_Para = { 0 };
static float We_Target = 0.0f;
static uint32_t Cnt = 0U;
static uint32_t Meas_Cnt = 0U;
static float We_Sum = 0.0f;
static float Id_Sum = 0.0f;
static float Iq_Sum = 0.0f;
static float Ud_Sum = 0.0f;
static float Uq_Sum = 0.0f;
static float U_Mag_Sum = 0.0f;
static float Finish_Iq = 0.0f;
static uint8_t Finish_Init = 0U;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Measure_Reset(void)
{
    Cnt = 0U;
    Meas_Cnt = 0U;
    We_Sum = 0.0f;
    Id_Sum = 0.0f;
    Iq_Sum = 0.0f;
    Ud_Sum = 0.0f;
    Uq_Sum = 0.0f;
    U_Mag_Sum = 0.0f;
}

static void Measure(void)
{
    We_Sum += IF_Start_We_Get();
    Id_Sum += Motor_Run.Id;
    Iq_Sum += Motor_Run.Iq;
    Ud_Sum += Motor_Run.Ud;
    Uq_Sum += Motor_Run.Uq;
    U_Mag_Sum += __builtin_sqrtf(Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq);
    Meas_Cnt++;
}

static bool Flux_Calc(float *We_Out, float *Flux_Out, float *U_Util_Out)
{
    const Ident_Envelope_T *Envelope;
    float We;
    float Id;
    float Iq;
    float Ud;
    float Uq;
    float Psi_d;
    float Psi_q;

    if ((Meas_Cnt == 0U) || (We_Out == NULL) || (Flux_Out == NULL) || (U_Util_Out == NULL))
    {
        return false;
    }

    Envelope = Identification_Envelope_Get();
    if (Envelope->U_Available <= 0.0f)
    {
        return false;
    }

    We = We_Sum / (float)Meas_Cnt;
    Id = Id_Sum / (float)Meas_Cnt;
    Iq = Iq_Sum / (float)Meas_Cnt;
    Ud = Ud_Sum / (float)Meas_Cnt;
    Uq = Uq_Sum / (float)Meas_Cnt;

    if (Abs_Value(We) <= 0.0f)
    {
        return false;
    }

    /* Steady-state dq model in the I/F frame. The PM flux components depend
     * on load angle, while their vector magnitude does not. */
    Psi_d = (Uq - Motor_Para.Rs * Iq) / We - Motor_Para.Ld * Id;
    Psi_q = -(Ud - Motor_Para.Rs * Id) / We - Motor_Para.Lq * Iq;

    *We_Out = We;
    *Flux_Out = __builtin_sqrtf(Psi_d * Psi_d + Psi_q * Psi_q);
    *U_Util_Out = (U_Mag_Sum / (float)Meas_Cnt) / Envelope->U_Available;

    return __builtin_isfinite(*Flux_Out) && (*Flux_Out > 0.0f) && __builtin_isfinite(*U_Util_Out);
}

static bool Search_Next_Build(float We_Meas, float Flux_Meas, float U_Util, int8_t Dir, float *We_Next_Out)
{
    const Ident_Envelope_T *Envelope;
    float We_Abs;
    float Emf_Ratio;
    float Step_Ratio;
    float U_Search_Max;
    float U_Mag;
    float Voltage_Step_Ratio;

    if ((We_Next_Out == NULL) || (Abs_Value(We_Meas) <= 0.0f) || (Flux_Meas <= 0.0f) ||
        (IF_Para.U_Budget_V <= 0.0f))
    {
        return false;
    }

    Envelope = Identification_Envelope_Get();
    if ((Envelope->U_Available <= 0.0f) || (Envelope->U_Max <= 0.0f))
    {
        return false;
    }

    We_Abs = Abs_Value(We_Meas);
    Emf_Ratio = We_Abs * Flux_Meas / IF_Para.U_Budget_V;
    Step_Ratio = FLUX_EMF_TARGET_RATIO / Emf_Ratio;

    if (Step_Ratio > FLUX_WE_STEP_MAX_RATIO)
    {
        Step_Ratio = FLUX_WE_STEP_MAX_RATIO;
    }

    U_Mag = U_Util * Envelope->U_Available;
    U_Search_Max = FLUX_U_SEARCH_RATIO * Envelope->U_Max;
    if (U_Mag > 0.0f)
    {
        Voltage_Step_Ratio = U_Search_Max / U_Mag;
        if (Step_Ratio > Voltage_Step_Ratio)
        {
            Step_Ratio = Voltage_Step_Ratio;
        }
    }

    if (Step_Ratio <= 1.0f)
    {
        return false;
    }

    *We_Next_Out = (float)Dir * We_Abs * Step_Ratio;
    return true;
}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;

    Result = (Flux_Result_T){ 0 };
    IF_Para = (Motor_IF_Para_T){ 0 };
    We_Target = 0.0f;
    Finish_Iq = 0.0f;
    Finish_Init = 0U;
    Measure_Reset();

    Envelope = Identification_Envelope_Get();
    if ((Envelope->I_Max <= 0.0f) || !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Max, &IF_Para))
    {
        State = FLUX_FAILED;
        return false;
    }

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Target = Sign * IF_Para.We_Base;

    Align_Reset();
    Current_Loop_State_Reset();
    State = FLUX_ALIGN;
    return true;
}

bool Flux_Active(void)
{
    return (State != FLUX_IDLE) && (State != FLUX_DONE) && (State != FLUX_FAILED);
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A, float Ib_A, float Ic_A, float *Theta_e, float *Id_Ref, float *Iq_Ref)
{
    const Ident_Envelope_T *Envelope;
    float Theta_IF;
    float Iq_Step;
    float We_Meas;
    float Flux_Meas;
    float U_Util;
    float Emf_Ratio;
    float We_Next;
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
            State = FLUX_ACCEL;
        }

        return FAST_CURRENT;
    }

    if (State == FLUX_FINISH)
    {
        (void)IF_Start_Run(&Theta_IF, Id_Ref, Iq_Ref);
        *Theta_e = Theta_IF;

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

        if ((Finish_Iq == 0.0f) && (Abs_Value(Ia_A) <= FLUX_FINISH_I) && (Abs_Value(Ib_A) <= FLUX_FINISH_I) &&
            (Abs_Value(Ic_A) <= FLUX_FINISH_I))
        {
            if (++Cnt >= FLUX_FINISH_CNT)
            {
                State = Result.Valid ? FLUX_DONE : FLUX_FAILED;
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

    if (State == FLUX_MEASURE)
    {
        Measure();

        if (Meas_Cnt >= FLUX_MEASURE_CNT)
        {
            if (!Flux_Calc(&We_Meas, &Flux_Meas, &U_Util))
            {
                State = FLUX_FAILED;
                return FAST_OFF;
            }

            Emf_Ratio = Abs_Value(We_Meas) * Flux_Meas / IF_Para.U_Budget_V;

            if (Emf_Ratio >= FLUX_EMF_TARGET_RATIO)
            {
                Result.Flux_Wb = Flux_Meas;
                Result.Valid = true;
                Cnt = 0U;
                Finish_Init = 0U;
                State = FLUX_FINISH;
            }
            else if (Search_Next_Build(We_Meas, Flux_Meas, U_Util, Dir, &We_Next))
            {
                We_Target = We_Next;
                Measure_Reset();
                IF_Start_Target_Set(We_Target);
                State = FLUX_ACCEL;
            }
            else if (Emf_Ratio >= FLUX_EMF_MIN_RATIO)
            {
                Result.Flux_Wb = Flux_Meas;
                Result.Valid = true;
                Cnt = 0U;
                Finish_Init = 0U;
                State = FLUX_FINISH;
            }
            else
            {
                State = FLUX_FAILED;
                Result.Valid = false;
                return FAST_OFF;
            }
        }
    }

    (void)IF_Start_Run(&Theta_IF, Id_Ref, Iq_Ref);
    *Theta_e = Theta_IF;

    if (IF_Start_State_Get() == IF_FAILED)
    {
        State = FLUX_FAILED;
        Result.Valid = false;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return FAST_OFF;
    }

    if (State == FLUX_ACCEL)
    {
        if (IF_Start_State_Get() == IF_HOLD)
        {
            Cnt = 0U;
            State = FLUX_SETTLE;
        }
    }
    else if (State == FLUX_SETTLE)
    {
        if (++Cnt >= FLUX_SETTLE_CNT)
        {
            Measure_Reset();
            State = FLUX_MEASURE;
        }
    }

    return FAST_CURRENT;
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
