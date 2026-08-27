/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

#include "Align.h"
#include "Current_Loop.h"
#include "IF_Start.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_Type.h"
#include "control_params.h"

#define FLUX_POINT_NUM          4U
#define FLUX_SETTLE_S           0.30f
#define FLUX_MEASURE_S          0.20f
#define FLUX_SETTLE_CNT         ((uint32_t)(FLUX_SETTLE_S / CUR_TS + 0.5f))
#define FLUX_MEASURE_CNT        ((uint32_t)(FLUX_MEASURE_S / CUR_TS + 0.5f))
#define FLUX_FINISH_IQ_SLEW_A_S 20.0f
#define FLUX_FINISH_I           0.20f
#define FLUX_FINISH_S           0.002f
#define FLUX_FINISH_CNT         ((uint32_t)(FLUX_FINISH_S / CUR_TS + 0.5f))

/* Search uses back-EMF relative to the identification voltage budget. Search
 * samples are not all fit samples: once the first useful EMF point is found,
 * one more proven point defines the safe fit window. Four fit points are then
 * placed inside that already-tested window. */
#define FLUX_E_TARGET_RATIO    0.35f
#define FLUX_E_MIN_RATIO       0.08f
#define FLUX_WE_STEP_MIN_RATIO 1.15f
#define FLUX_WE_STEP_MAX_RATIO 1.60f
#define FLUX_WE_MARGIN_RATIO   0.80f
#define FLUX_WE_SPAN_MIN_RATIO 1.05f

static float We_Point[FLUX_POINT_NUM] = { 0 };

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static uint8_t Point = 0U;
static uint32_t Cnt = 0U;
static uint32_t Meas_Cnt = 0U;
static float E_Sum = 0.0f;
static float We_Sum = 0.0f;
static float We_Mean[FLUX_POINT_NUM] = { 0 };
static float E_Mean[FLUX_POINT_NUM] = { 0 };
static float Finish_Iq = 0.0f;
static uint8_t Finish_Init = 0U;
static bool Search_Mode = true;
static bool Search_Have_Low = false;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Measure_Reset(void)
{
    E_Sum = 0.0f;
    We_Sum = 0.0f;
    Meas_Cnt = 0U;
    Cnt = 0U;
}

static bool Search_Next_Build(float We_Meas, float E_Meas, int8_t Dir, float *We_Next_Out)
{
    const Ident_PreFlux_T *PreFlux;
    float We_Abs;
    float E_Abs;
    float E_Ratio;
    float Step_Ratio;
    float Flux_Rough;
    float U_Margin;
    float Den;
    float We_Voltage_Max;
    float We_Next;

    if ((We_Next_Out == NULL) || (Abs_Value(We_Meas) <= 0.0f) || (We_Meas * E_Meas <= 0.0f))
    {
        return false;
    }

    PreFlux = Identification_PreFlux_Get();
    if ((PreFlux->U_Budget_V <= 0.0f) || (PreFlux->Iq_Max_A <= 0.0f))
    {
        return false;
    }

    We_Abs = Abs_Value(We_Meas);
    E_Abs = Abs_Value(E_Meas);
    E_Ratio = E_Abs / PreFlux->U_Budget_V;

    if (E_Ratio < FLUX_E_MIN_RATIO)
    {
        Step_Ratio = FLUX_WE_STEP_MAX_RATIO;
    }
    else
    {
        Step_Ratio = FLUX_E_TARGET_RATIO / E_Ratio;
        if (Step_Ratio < FLUX_WE_STEP_MIN_RATIO)
        {
            Step_Ratio = FLUX_WE_STEP_MIN_RATIO;
        }
        else if (Step_Ratio > FLUX_WE_STEP_MAX_RATIO)
        {
            Step_Ratio = FLUX_WE_STEP_MAX_RATIO;
        }
    }

    We_Next = We_Abs * Step_Ratio;

    /* The latest EMF estimate only limits the next search step. It never plans
     * all remaining points from one low-speed sample. */
    Flux_Rough = E_Abs / We_Abs;
    U_Margin = PreFlux->U_Budget_V - Motor_Para.Rs * PreFlux->Iq_Max_A;
    Den = Flux_Rough + Motor_Para.Ld * PreFlux->Iq_Max_A;

    if ((U_Margin <= 0.0f) || (Den <= 0.0f))
    {
        return false;
    }

    We_Voltage_Max = FLUX_WE_MARGIN_RATIO * U_Margin / Den;
    if (We_Next > We_Voltage_Max)
    {
        We_Next = We_Voltage_Max;
    }

    if (We_Next < FLUX_WE_SPAN_MIN_RATIO * We_Abs)
    {
        return false;
    }

    *We_Next_Out = (float)Dir * We_Next;
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

static void Measure(void)
{
    float We;
    float E;

    We = IF_Start_We_Get();

    /* At constant I/F speed after settling, the dq voltage model reduces to:
     *   Uq - Rs*Iq - We*Ld*Id = We*Flux + Voffset
     * The signed quantities are retained so the same fit works in both
     * rotation directions without an observer or a current derivative. */
    E = Motor_Run.Uq - Motor_Para.Rs * Motor_Run.Iq - We * Motor_Para.Ld * Motor_Run.Id;

    E_Sum += E;
    We_Sum += We;
    Meas_Cnt++;
}

void Flux_Start(float Wm_Target)
{
    const Ident_PreFlux_T *PreFlux;
    float Sign;

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;
    PreFlux = Identification_PreFlux_Get();

    Flux_Reset();
    We_Point[0] = Sign * PreFlux->We_Base;

    Align_Reset();
    Current_Loop_State_Reset();
    State = FLUX_ALIGN;
}

void Flux_Reset(void)
{
    State = FLUX_IDLE;
    Result.Flux_Wb = 0.0f;
    Result.V_Offset_V = 0.0f;
    Result.Fit_R2 = 0.0f;
    Result.Valid = false;
    Point = 0U;
    Cnt = 0U;
    Meas_Cnt = 0U;
    E_Sum = 0.0f;
    We_Sum = 0.0f;
    Finish_Iq = 0.0f;
    Finish_Init = 0U;
    Search_Mode = true;
    Search_Have_Low = false;

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        We_Point[n] = 0.0f;
        We_Mean[n] = 0.0f;
        E_Mean[n] = 0.0f;
    }
}

void Flux_Fail(void)
{
    State = FLUX_FAILED;
    Result.Valid = false;
}

void Flux_Control(void)
{
    float Sum_X = 0.0f;
    float Sum_Y = 0.0f;
    float Sum_XX = 0.0f;
    float Sum_XY = 0.0f;
    float Y_Mean;
    float SS_Tot = 0.0f;
    float SS_Err = 0.0f;
    float Den;
    float Y_Est;

    if (State != FLUX_CALC)
    {
        return;
    }

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Sum_X += We_Mean[n];
        Sum_Y += E_Mean[n];
        Sum_XX += We_Mean[n] * We_Mean[n];
        Sum_XY += We_Mean[n] * E_Mean[n];
    }

    Den = (float)FLUX_POINT_NUM * Sum_XX - Sum_X * Sum_X;

    if (Den <= 0.0f)
    {
        Result.Valid = false;
        Cnt = 0U;
        Finish_Init = 0U;
        State = FLUX_FINISH;
        return;
    }

    Result.Flux_Wb = ((float)FLUX_POINT_NUM * Sum_XY - Sum_X * Sum_Y) / Den;
    Result.V_Offset_V = (Sum_Y - Result.Flux_Wb * Sum_X) / (float)FLUX_POINT_NUM;

    Y_Mean = Sum_Y / (float)FLUX_POINT_NUM;

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Y_Est = Result.Flux_Wb * We_Mean[n] + Result.V_Offset_V;
        SS_Tot += (E_Mean[n] - Y_Mean) * (E_Mean[n] - Y_Mean);
        SS_Err += (E_Mean[n] - Y_Est) * (E_Mean[n] - Y_Est);
    }

    Result.Fit_R2 = (SS_Tot > 0.0f) ? (1.0f - SS_Err / SS_Tot) : 0.0f;
    Result.Valid = (Result.Flux_Wb > 0.0f);
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
    const Ident_PreFlux_T *PreFlux;
    float Theta_IF;
    float Iq_Step;
    float We_Meas;
    float E_Meas;
    float E_Ratio;
    float We_Next;
    int8_t Dir;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;

    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    PreFlux = Identification_PreFlux_Get();
    Dir = (We_Point[0] < 0.0f) ? -1 : 1;

    if (State == FLUX_ALIGN)
    {
        if (Align_Current(PreFlux->Iq_Start_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Start_Reset(-0.5f * PI_F * (float)Dir, 0.0f);
            IF_Start_Para_Set(PreFlux->Iq_Start_A, PreFlux->Iq_Max_A, PreFlux->We_Base, PreFlux->Acc);
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

    if (State == FLUX_MEASURE)
    {
        Measure();

        if (Meas_Cnt >= FLUX_MEASURE_CNT)
        {
            We_Meas = We_Sum / (float)Meas_Cnt;
            E_Meas = E_Sum / (float)Meas_Cnt;

            if ((Abs_Value(We_Meas) <= 0.0f) || (We_Meas * E_Meas <= 0.0f) || (PreFlux->U_Budget_V <= 0.0f))
            {
                Flux_Fail();
                return FAST_OFF;
            }

            if (Search_Mode)
            {
                E_Ratio = Abs_Value(E_Meas) / PreFlux->U_Budget_V;

                if (!Search_Have_Low)
                {
                    if (E_Ratio >= FLUX_E_MIN_RATIO)
                    {
                        We_Mean[0] = We_Meas;
                        E_Mean[0] = E_Meas;
                        Search_Have_Low = true;
                    }

                    if (!Search_Next_Build(We_Meas, E_Meas, Dir, &We_Next))
                    {
                        Flux_Fail();
                        return FAST_OFF;
                    }

                    Measure_Reset();
                    IF_Start_Target_Set(We_Next);
                    State = FLUX_ACCEL;
                }
                else
                {
                    /* The first point after the useful low point is the proven
                     * high bound. Do not keep searching upward just to reach the
                     * target EMF ratio. Reuse both bounds in the final fit. */
                    if (E_Ratio < FLUX_E_MIN_RATIO)
                    {
                        Flux_Fail();
                        return FAST_OFF;
                    }

                    We_Mean[3] = We_Meas;
                    E_Mean[3] = E_Meas;

                    if (!Fit_Window_Build(We_Mean[0], We_Mean[3], Dir))
                    {
                        Flux_Fail();
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
                We_Mean[Point] = We_Meas;
                E_Mean[Point] = E_Meas;

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
    return State;
}

const Flux_Result_T *Flux_Result_Get(void)
{
    return &Result;
}
