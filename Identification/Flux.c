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

/* Flux work points are generated one-by-one from measured steady-state EMF.
 * Speed is only the excitation variable: the desired signal level is the
 * measured back-EMF relative to the available identification voltage budget. */
#define FLUX_E_TARGET_RATIO       0.35f
#define FLUX_E_MIN_RATIO          0.08f
#define FLUX_WE_STEP_MIN_RATIO    1.15f
#define FLUX_WE_STEP_MAX_RATIO    1.60f
#define FLUX_WE_MARGIN_RATIO      0.80f
#define FLUX_WE_SPAN_MIN_RATIO    1.05f

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

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static bool Next_Work_Point_Build(uint8_t Current_Point, int8_t Dir)
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

    if ((Current_Point >= (FLUX_POINT_NUM - 1U)) || (Abs_Value(We_Mean[Current_Point]) <= 0.0f) ||
        (We_Mean[Current_Point] * E_Mean[Current_Point] <= 0.0f))
    {
        return false;
    }

    PreFlux = Identification_PreFlux_Get();
    if ((PreFlux->U_Budget_V <= 0.0f) || (PreFlux->Iq_Max_A <= 0.0f))
    {
        return false;
    }

    We_Abs = Abs_Value(We_Mean[Current_Point]);
    E_Abs = Abs_Value(E_Mean[Current_Point]);
    E_Ratio = E_Abs / PreFlux->U_Budget_V;

    /* Drive the next point toward a useful EMF window. A very small EMF asks
     * for the largest allowed step, while an already useful point expands the
     * span only modestly so one noisy low-speed estimate cannot launch I/F far
     * beyond the proven operating region. */
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

    /* Use the latest measured EMF only as a voltage-safety estimate, not as a
     * one-shot planner for all remaining points. The per-point step limit above
     * keeps offset/error at the first point from producing a large speed jump. */
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

    We_Point[Current_Point + 1U] = (float)Dir * We_Next;
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
            We_Mean[Point] = We_Sum / (float)Meas_Cnt;
            E_Mean[Point] = E_Sum / (float)Meas_Cnt;

            if (Point < (FLUX_POINT_NUM - 1U))
            {
                if (!Next_Work_Point_Build(Point, Dir))
                {
                    Flux_Fail();
                    return FAST_OFF;
                }
            }

            Point++;

            if (Point >= FLUX_POINT_NUM)
            {
                State = FLUX_CALC;
            }
            else
            {
                E_Sum = 0.0f;
                We_Sum = 0.0f;
                Meas_Cnt = 0U;
                Cnt = 0U;
                IF_Start_Target_Set(We_Point[Point]);
                State = FLUX_ACCEL;
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
            Cnt = 0U;
            E_Sum = 0.0f;
            We_Sum = 0.0f;
            Meas_Cnt = 0U;
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
