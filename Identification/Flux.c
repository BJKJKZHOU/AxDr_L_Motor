/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux.h"

#include "Align.h"
#include "Current_Loop.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define FLUX_POINT_NUM            4U
#define FLUX_WE_1                 120.0f
#define FLUX_WE_2                 160.0f
#define FLUX_WE_3                 200.0f
#define FLUX_WE_4                 240.0f
#define FLUX_SETTLE_S             0.30f
#define FLUX_MEASURE_S            0.20f
#define FLUX_SETTLE_CNT           ((uint32_t)(FLUX_SETTLE_S / CUR_TS + 0.5f))
#define FLUX_MEASURE_CNT          ((uint32_t)(FLUX_MEASURE_S / CUR_TS + 0.5f))
#define FLUX_FINISH_IQ_SLEW_A_S   20.0f
#define FLUX_FINISH_I             0.20f
#define FLUX_FINISH_S             0.002f
#define FLUX_FINISH_CNT           ((uint32_t)(FLUX_FINISH_S / CUR_TS + 0.5f))

static float We_Point[FLUX_POINT_NUM] = { 0 };

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static uint8_t Point = 0U;
static uint32_t Cnt = 0U;
static uint32_t Meas_Cnt = 0U;
static float E_Sum = 0.0f;
static float We_Mean[FLUX_POINT_NUM] = { 0 };
static float E_Mean[FLUX_POINT_NUM] = { 0 };
static float Theta_Pre = 0.0f;
static float Finish_Iq = 0.0f;
static uint8_t U_Valid = 0U;
static uint8_t Finish_Init = 0U;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Measure(float Ia_A, float Ib_A)
{
    float Ialpha;
    float Ibeta;
    float Sin;
    float Cos;
    float Ualpha;
    float Ubeta;
    float Ealpha;
    float Ebeta;
    float E;

    if (U_Valid == 0U)
    {
        return;
    }

    SinCos(Theta_Pre, &Sin, &Cos);

    Ualpha = Motor_Run.Ud * Cos - Motor_Run.Uq * Sin;
    Ubeta = Motor_Run.Ud * Sin + Motor_Run.Uq * Cos;

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;

    Ealpha = Ualpha - Motor_Para.Rs * Ialpha;
    Ebeta = Ubeta - Motor_Para.Rs * Ibeta;
    E = __builtin_sqrtf(Ealpha * Ealpha + Ebeta * Ebeta);

    E_Sum += E;
    Meas_Cnt++;
}

void Flux_Start(float Wm_Target)
{
    float Sign;

    Sign = (Wm_Target < 0.0f) ? -1.0f : 1.0f;

    Flux_Reset();
    We_Point[0] = Sign * FLUX_WE_1;
    We_Point[1] = Sign * FLUX_WE_2;
    We_Point[2] = Sign * FLUX_WE_3;
    We_Point[3] = Sign * FLUX_WE_4;

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
    Theta_Pre = 0.0f;
    Finish_Iq = 0.0f;
    U_Valid = 0U;
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

    /* First hardware version keeps validity deliberately minimal.
     * Fit_R2 is reported for tuning; a quality threshold will be set from data. */
    Result.Valid = (Result.Flux_Wb > 0.0f);
    Cnt = 0U;
    Finish_Init = 0U;
    State = FLUX_FINISH;
}

bool Flux_Active(void)
{
    return (State != FLUX_IDLE) && (State != FLUX_DONE) && (State != FLUX_FAILED);
}

Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A, float Ib_A, float Ic_A, float *Id_Ref, float *Iq_Ref)
{
    float Theta_e;
    float Iq_Step;
    int8_t Dir;

    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;

    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    if (State == FLUX_ALIGN)
    {
        Motor_Run.Theta_e = 0.0f;

        if (Align_Current(IF_ALIGN_ID_A, IF_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Current_Loop_State_Reset();
            Dir = (We_Point[0] < 0.0f) ? -1 : 1;
            IF_Start_Reset(-0.5f * PI_F * (float)Dir, 0.0f);
            IF_Start_Target_Set(We_Point[0]);
            State = FLUX_ACCEL;
        }

        return FAST_CURRENT;
    }

    if (State == FLUX_FINISH)
    {
        (void)IF_Start_Run(&Theta_e, Id_Ref, Iq_Ref);
        Motor_Run.Theta_e = Theta_e;

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
        Measure(Ia_A, Ib_A);

        if (Meas_Cnt >= FLUX_MEASURE_CNT)
        {
            /* Back-EMF magnitude is fitted against electrical-speed magnitude. */
            We_Mean[Point] = Abs_Value(IF_Start_We_Get());
            E_Mean[Point] = E_Sum / (float)Meas_Cnt;

            Point++;

            if (Point >= FLUX_POINT_NUM)
            {
                State = FLUX_CALC;
            }
            else
            {
                E_Sum = 0.0f;
                Meas_Cnt = 0U;
                Cnt = 0U;
                IF_Start_Target_Set(We_Point[Point]);
                State = FLUX_ACCEL;
            }
        }
    }

    (void)IF_Start_Run(&Theta_e, Id_Ref, Iq_Ref);
    Motor_Run.Theta_e = Theta_e;
    Theta_Pre = Theta_e;
    U_Valid = 1U;

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
