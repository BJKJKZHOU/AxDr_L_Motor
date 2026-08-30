/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

#include <stddef.h>

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
#define FLUX_WE_STEP_MIN_RATIO 1.15f
#define FLUX_WE_STEP_MAX_RATIO 1.60f
#define FLUX_U_SEARCH_RATIO    0.90f
#define FLUX_WE_SPAN_MIN_RATIO 1.05f

#define FLUX_FAIL_STAGE_POINT_CALC  9U
#define FLUX_FAIL_STAGE_SEARCH_NEXT 10U
#define FLUX_FAIL_STAGE_FIT_WINDOW  11U
#define FLUX_FAIL_STAGE_ENVELOPE    12U
#define FLUX_FAIL_STAGE_IF_PARA     13U

static float We_Point[FLUX_POINT_NUM] = { 0 };
static float Flux_Point[FLUX_POINT_NUM] = { 0 };
static float U_Util_Point[FLUX_POINT_NUM] = { 0 };

static volatile Flux_State_e State = FLUX_IDLE;
static volatile uint8_t Fail_Stage = (uint8_t)FLUX_FAILED;
static Flux_Result_T Result = { 0 };
static Motor_IF_Para_T IF_Para = { 0 };
static uint8_t Point = 0U;
static uint32_t Cnt = 0U;
static uint32_t Meas_Cnt = 0U;
static float We_Sum = 0.0f;
static float Id_Sum = 0.0f;
static float Iq_Sum = 0.0f;
static float Ud_Sum = 0.0f;
static float Uq_Sum = 0.0f;
static float U_Mag_Sum = 0.0f;
static float U_Mag_Max = 0.0f;
static float Finish_Iq = 0.0f;
static uint8_t Finish_Init = 0U;
static bool Search_Mode = true;
static bool Search_Have_Low = false;

static void Flux_Fail_Stage(uint8_t Stage);

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Max_Value(float A, float B)
{
    return (A > B) ? A : B;
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
    U_Mag_Max = 0.0f;
}

static void Measure(void)
{
    float U_Mag;

    We_Sum += IF_Start_We_Get();
    Id_Sum += Motor_Run.Id;
    Iq_Sum += Motor_Run.Iq;
    Ud_Sum += Motor_Run.Ud;
    Uq_Sum += Motor_Run.Uq;

    U_Mag = __builtin_sqrtf(Motor_Run.Ud * Motor_Run.Ud + Motor_Run.Uq * Motor_Run.Uq);
    U_Mag_Sum += U_Mag;
    U_Mag_Max = Max_Value(U_Mag_Max, U_Mag);
    Meas_Cnt++;
}

static bool Flux_Point_Calc(Flux_Point_T *Point_Out,
                            float *Flux_Out,
                            float *U_Util_Avg_Out,
                            float *U_Util_Peak_Out)
{
    const Ident_Envelope_T *Envelope;
    float We;
    float Id;
    float Iq;
    float Ud;
    float Uq;
    float Psi_d;
    float Psi_q;

    if ((Meas_Cnt == 0U) || (Point_Out == NULL) || (Flux_Out == NULL) || (U_Util_Avg_Out == NULL) ||
        (U_Util_Peak_Out == NULL))
    {
        return false;
    }

    Envelope = Identification_Envelope_Get();
    if (!Envelope->Valid || (Envelope->U_Available_V <= 0.0f))
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

    Point_Out->We = We;
    Point_Out->E = Uq - Motor_Para.Rs * Iq - We * Motor_Para.Ld * Id;
    Point_Out->Id = Id;
    Point_Out->Iq = Iq;
    Point_Out->Ud = Ud;
    Point_Out->Uq = Uq;
    *Flux_Out = __builtin_sqrtf(Psi_d * Psi_d + Psi_q * Psi_q);
    *U_Util_Avg_Out = (U_Mag_Sum / (float)Meas_Cnt) / Envelope->U_Available_V;
    *U_Util_Peak_Out = U_Mag_Max / Envelope->U_Available_V;

    return __builtin_isfinite(*Flux_Out) && (*Flux_Out > 0.0f) && __builtin_isfinite(*U_Util_Avg_Out) &&
           __builtin_isfinite(*U_Util_Peak_Out);
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
    if (!Envelope->Valid || (Envelope->U_Available_V <= 0.0f) || (Envelope->U_Hard_V <= 0.0f))
    {
        return false;
    }

    We_Abs = Abs_Value(We_Meas);
    Emf_Ratio = We_Abs * Flux_Meas / IF_Para.U_Budget_V;

    if (Emf_Ratio < FLUX_EMF_MIN_RATIO)
    {
        Step_Ratio = FLUX_WE_STEP_MAX_RATIO;
    }
    else
    {
        Step_Ratio = FLUX_EMF_TARGET_RATIO / Emf_Ratio;
        if (Step_Ratio < FLUX_WE_STEP_MIN_RATIO)
        {
            Step_Ratio = FLUX_WE_STEP_MIN_RATIO;
        }
        else if (Step_Ratio > FLUX_WE_STEP_MAX_RATIO)
        {
            Step_Ratio = FLUX_WE_STEP_MAX_RATIO;
        }
    }

    U_Mag = U_Util * Envelope->U_Available_V;
    U_Search_Max = FLUX_U_SEARCH_RATIO * Envelope->U_Hard_V;
    if (U_Mag > 0.0f)
    {
        Voltage_Step_Ratio = U_Search_Max / U_Mag;
        if (Step_Ratio > Voltage_Step_Ratio)
        {
            Step_Ratio = Voltage_Step_Ratio;
        }
    }

    if (Step_Ratio < FLUX_WE_SPAN_MIN_RATIO)
    {
        return false;
    }

    *We_Next_Out = (float)Dir * We_Abs * Step_Ratio;
    return true;
}

static bool Fit_Window_Build(float We_Low, float We_High, int8_t Dir)
{
    float Low_Abs;
    float High_Abs;
    float Span;

    Low_Abs = Abs_Value(We_Low);
    High_Abs = Abs_Value(We_High);

    if ((Low_Abs <= 0.0f) || (High_Abs < FLUX_WE_SPAN_MIN_RATIO * Low_Abs))
    {
        return false;
    }

    Span = (High_Abs - Low_Abs) / (float)(FLUX_POINT_NUM - 1U);
    We_Point[0] = (float)Dir * Low_Abs;
    We_Point[1] = (float)Dir * (Low_Abs + Span);
    We_Point[2] = (float)Dir * (Low_Abs + 2.0f * Span);
    We_Point[3] = (float)Dir * High_Abs;
    return true;
}

static float Median_4(const float Value[FLUX_POINT_NUM])
{
    float Sort[FLUX_POINT_NUM];
    float Tmp;

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Sort[n] = Value[n];
    }

    for (uint8_t i = 0U; i < (FLUX_POINT_NUM - 1U); i++)
    {
        for (uint8_t j = (uint8_t)(i + 1U); j < FLUX_POINT_NUM; j++)
        {
            if (Sort[j] < Sort[i])
            {
                Tmp = Sort[i];
                Sort[i] = Sort[j];
                Sort[j] = Tmp;
            }
        }
    }

    return 0.5f * (Sort[1] + Sort[2]);
}

bool Flux_Start(float Wm_Target)
{
    const Ident_Envelope_T *Envelope;
    float Sign;

    Flux_Reset();
    Envelope = Identification_Envelope_Get();
    if (!Envelope->Valid || !Motor_IF_Para_Build(ADC.Vbus_V, Envelope->I_Safe_A, &IF_Para))
    {
        Flux_Fail_Stage(FLUX_FAIL_STAGE_IF_PARA);
        return false;
    }

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    We_Point[0] = Sign * IF_Para.We_Base;

    Align_Reset();
    Current_Loop_State_Reset();
    State = FLUX_ALIGN;
    return true;
}

void Flux_Reset(void)
{
    State = FLUX_IDLE;
    Fail_Stage = (uint8_t)FLUX_FAILED;
    Result = (Flux_Result_T){ 0 };
    IF_Para = (Motor_IF_Para_T){ 0 };
    Point = 0U;
    Finish_Iq = 0.0f;
    Finish_Init = 0U;
    Search_Mode = true;
    Search_Have_Low = false;
    Measure_Reset();

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        We_Point[n] = 0.0f;
        Flux_Point[n] = 0.0f;
        U_Util_Point[n] = 0.0f;
    }
}

void Flux_Fail(void)
{
    State = FLUX_FAILED;
    Result.Valid = false;
}

static void Flux_Fail_Stage(uint8_t Stage)
{
    Fail_Stage = Stage;
    Flux_Fail();
}

void Flux_Control(void)
{
    float Median;
    float Max_Dev = 0.0f;
    float Dev;
    float U_Max = 0.0f;

    if (State != FLUX_CALC)
    {
        return;
    }

    Median = Median_4(Flux_Point);
    if (Median <= 0.0f)
    {
        Result.Valid = false;
        Cnt = 0U;
        Finish_Init = 0U;
        State = FLUX_FINISH;
        return;
    }

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Dev = Abs_Value(Flux_Point[n] - Median) / Median;
        Max_Dev = Max_Value(Max_Dev, Dev);
        U_Max = Max_Value(U_Max, U_Util_Point[n]);
    }

    Result.Flux_Wb = Median;
    Result.Point_Max_Rel_Dev = Max_Dev;
    Result.U_Util_Max = U_Max;
    Result.Valid = __builtin_isfinite(Result.Flux_Wb) && (Result.Flux_Wb > 0.0f) &&
                   __builtin_isfinite(Result.Point_Max_Rel_Dev) && __builtin_isfinite(Result.U_Util_Max);
    Result.Point_Num = Result.Valid ? FLUX_POINT_NUM : 0U;

    Cnt = 0U;
    Finish_Init = 0U;
    State = FLUX_FINISH;
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
    float U_Util_Peak;
    float Emf_Ratio;
    float We_Next;
    Flux_Point_T Point_Meas;
    int8_t Dir;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;

    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    Envelope = Identification_Envelope_Get();
    Dir = (We_Point[0] < 0.0f) ? -1 : 1;

    if (State == FLUX_ALIGN)
    {
        if (Align_Current(IF_Para.Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Start_Reset(-0.5f * PI_F * (float)Dir, 0.0f);
            IF_Start_Para_Set(IF_Para.Iq_Start_A, IF_Para.Iq_Max_A, IF_Para.We_Base, IF_Para.Acc);
            IF_Start_Target_Set(We_Point[0]);
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

    if (!Envelope->Valid || (Envelope->U_Hard_V <= 0.0f) || (Envelope->U_Available_V <= 0.0f))
    {
        Flux_Fail_Stage(FLUX_FAIL_STAGE_ENVELOPE);
        return FAST_OFF;
    }

    if (State == FLUX_MEASURE)
    {
        Measure();
        if (Meas_Cnt >= FLUX_MEASURE_CNT)
        {
            if (!Flux_Point_Calc(&Point_Meas, &Flux_Meas, &U_Util, &U_Util_Peak))
            {
                Flux_Fail_Stage(FLUX_FAIL_STAGE_POINT_CALC);
                return FAST_OFF;
            }

            We_Meas = Point_Meas.We;
            Result.Flux_Wb = Flux_Meas;
            Result.U_Util_Max = U_Util_Peak;

            if (Search_Mode)
            {
                Emf_Ratio = Abs_Value(We_Meas) * Flux_Meas / IF_Para.U_Budget_V;
                Result.Point_Max_Rel_Dev = Emf_Ratio;

                if (!Search_Have_Low)
                {
                    if (Emf_Ratio >= FLUX_EMF_MIN_RATIO)
                    {
                        We_Point[0] = We_Meas;
                        Flux_Point[0] = Flux_Meas;
                        U_Util_Point[0] = U_Util_Peak;
                        Result.Point[0] = Point_Meas;
                        Search_Have_Low = true;
                    }

                    if (!Search_Next_Build(We_Meas, Flux_Meas, U_Util, Dir, &We_Next))
                    {
                        Flux_Fail_Stage(FLUX_FAIL_STAGE_SEARCH_NEXT);
                        return FAST_OFF;
                    }

                    Measure_Reset();
                    IF_Start_Target_Set(We_Next);
                    State = FLUX_ACCEL;
                }
                else
                {
                    We_Point[3] = We_Meas;
                    Flux_Point[3] = Flux_Meas;
                    U_Util_Point[3] = U_Util_Peak;
                    Result.Point[3] = Point_Meas;

                    if (!Fit_Window_Build(We_Point[0], We_Point[3], Dir))
                    {
                        Flux_Fail_Stage(FLUX_FAIL_STAGE_FIT_WINDOW);
                        return FAST_OFF;
                    }

                    Search_Mode = false;
                    Point = 1U;
                    Measure_Reset();
                    IF_Start_Target_Set(We_Point[Point]);
                    State = FLUX_ACCEL;
                }
            }
            else
            {
                Flux_Point[Point] = Flux_Meas;
                U_Util_Point[Point] = U_Util_Peak;
                Result.Point[Point] = Point_Meas;

                if (Point == 1U)
                {
                    Point = 2U;
                    Measure_Reset();
                    IF_Start_Target_Set(We_Point[Point]);
                    State = FLUX_ACCEL;
                }
                else
                {
                    State = FLUX_CALC;
                }
            }
        }
    }

    (void)IF_Start_Run(&Theta_IF, Id_Ref, Iq_Ref);
    *Theta_e = Theta_IF;

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

Flux_State_e Flux_State_Get(void)
{
    if (State == FLUX_FAILED)
    {
        return (Flux_State_e)Fail_Stage;
    }
    return State;
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
