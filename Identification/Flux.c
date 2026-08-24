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
#include "PLL.h"
#include "control_params.h"

#define FLUX_POINT_NUM          4U
#define FLUX_SETTLE_S           0.30f
#define FLUX_MEASURE_S          0.20f
#define FLUX_SETTLE_CNT         ((uint32_t)(FLUX_SETTLE_S / CUR_TS + 0.5f))
#define FLUX_MEASURE_CNT        ((uint32_t)(FLUX_MEASURE_S / CUR_TS + 0.5f))
#define FLUX_SYNC_S             0.10f
#define FLUX_SYNC_CNT           ((uint32_t)(FLUX_SYNC_S / CUR_TS + 0.5f))
#define FLUX_FINISH_IQ_SLEW_A_S 20.0f
#define FLUX_FINISH_I           0.20f
#define FLUX_FINISH_S           0.002f
#define FLUX_FINISH_CNT         ((uint32_t)(FLUX_FINISH_S / CUR_TS + 0.5f))

#define FLUX_EMF_MIN_RATIO      0.02f
#define FLUX_EMF_TAU_MIN_TS     5.0f
#define FLUX_PLL_POLE_RATIO     0.05f
#define FLUX_PLL_WN_MIN         (TWO_PI_F * 5.0f)
#define FLUX_PLL_WN_MAX         (TWO_PI_F * 100.0f)
#define FLUX_PLL_DAMP           0.707f
#define FLUX_SYNC_WE_RATIO      0.25f
#define FLUX_SYNC_ERR_MAX       0.25f
#define FLUX_WE_MARGIN_RATIO    0.80f
#define FLUX_WE_SPAN_MIN_RATIO  1.50f

static float We_Point[FLUX_POINT_NUM] = { 0 };

static volatile Flux_State_e State = FLUX_IDLE;
static Flux_Result_T Result = { 0 };
static uint8_t Point = 0U;
static uint32_t Cnt = 0U;
static uint32_t Meas_Cnt = 0U;
static uint32_t Sync_Cnt = 0U;
static float E_Sum = 0.0f;
static float We_Sum = 0.0f;
static float We_Mean[FLUX_POINT_NUM] = { 0 };
static float E_Mean[FLUX_POINT_NUM] = { 0 };
static float Finish_Iq = 0.0f;
static uint8_t Finish_Init = 0U;

static PLL_T Emf_PLL = { 0 };
static float Ialpha_Pre = 0.0f;
static float Ibeta_Pre = 0.0f;
static float Ealpha_F = 0.0f;
static float Ebeta_F = 0.0f;
static float E_Mag = 0.0f;
static uint8_t Emf_Init = 0U;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static void Emf_Reset(float Theta, int8_t Dir)
{
    float Wn;

    Ialpha_Pre = 0.0f;
    Ibeta_Pre = 0.0f;
    Ealpha_F = 0.0f;
    Ebeta_F = 0.0f;
    E_Mag = 0.0f;
    Emf_Init = 0U;
    Sync_Cnt = 0U;

    Wn = FLUX_PLL_POLE_RATIO * Motor_Para.Rs / Motor_Para.Ld;
    if (Wn < FLUX_PLL_WN_MIN)
    {
        Wn = FLUX_PLL_WN_MIN;
    }
    else if (Wn > FLUX_PLL_WN_MAX)
    {
        Wn = FLUX_PLL_WN_MAX;
    }

    Emf_PLL.Para.Kp = 2.0f * FLUX_PLL_DAMP * Wn;
    Emf_PLL.Para.Ki = Wn * Wn;
    PLL_Reset(&Emf_PLL, Angle_Wrap(Theta + (float)Dir * 0.5f * PI_F), 0.0f);
}

static void Emf_Update(float Ia_A, float Ib_A)
{
    const Ident_PreFlux_T *PreFlux;
    float Ialpha;
    float Ibeta;
    float dIalpha;
    float dIbeta;
    float Ealpha;
    float Ebeta;
    float Tau;
    float Alpha;
    float Mag;
    float Mag_Min;

    PreFlux = Identification_PreFlux_Get();
    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;

    if (Emf_Init == 0U)
    {
        Ialpha_Pre = Ialpha;
        Ibeta_Pre = Ibeta;
        Emf_Init = 1U;
        return;
    }

    dIalpha = (Ialpha - Ialpha_Pre) / CUR_TS;
    dIbeta = (Ibeta - Ibeta_Pre) / CUR_TS;
    Ialpha_Pre = Ialpha;
    Ibeta_Pre = Ibeta;

    Ealpha = Motor_Run.Ualpha - Motor_Para.Rs * Ialpha - Motor_Para.Ld * dIalpha;
    Ebeta = Motor_Run.Ubeta - Motor_Para.Rs * Ibeta - Motor_Para.Ld * dIbeta;

    Tau = Motor_Para.Ld / Motor_Para.Rs;
    if (Tau < FLUX_EMF_TAU_MIN_TS * CUR_TS)
    {
        Tau = FLUX_EMF_TAU_MIN_TS * CUR_TS;
    }

    Alpha = CUR_TS / (Tau + CUR_TS);
    Ealpha_F += Alpha * (Ealpha - Ealpha_F);
    Ebeta_F += Alpha * (Ebeta - Ebeta_F);

    Mag = __builtin_sqrtf(Ealpha_F * Ealpha_F + Ebeta_F * Ebeta_F);
    E_Mag = Mag;
    Mag_Min = FLUX_EMF_MIN_RATIO * PreFlux->U_Budget_V;

    if (Mag >= Mag_Min)
    {
        PLL_Run(&Emf_PLL, Ealpha_F, Ebeta_F, Mag, CUR_TS);
    }
}

static bool Emf_Sync(void)
{
    const Ident_PreFlux_T *PreFlux;
    float We_IF;
    float We_Err;
    float Mag_Min;

    PreFlux = Identification_PreFlux_Get();
    We_IF = IF_Start_We_Get();
    We_Err = Emf_PLL.State.We - We_IF;
    Mag_Min = FLUX_EMF_MIN_RATIO * PreFlux->U_Budget_V;

    if ((E_Mag < Mag_Min) || (Abs_Value(We_IF) <= 0.0f))
    {
        Sync_Cnt = 0U;
        return false;
    }

    if ((Abs_Value(We_Err) > FLUX_SYNC_WE_RATIO * Abs_Value(We_IF)) ||
        (Abs_Value(Emf_PLL.State.Err) > FLUX_SYNC_ERR_MAX))
    {
        Sync_Cnt = 0U;
        return false;
    }

    if (Sync_Cnt < FLUX_SYNC_CNT)
    {
        Sync_Cnt++;
    }

    return Sync_Cnt >= FLUX_SYNC_CNT;
}

static bool Work_Points_Build(int8_t Dir)
{
    const Ident_PreFlux_T *PreFlux;
    float Flux_Rough;
    float U_Margin;
    float Den;
    float We_Max;
    float Span;

    PreFlux = Identification_PreFlux_Get();

    if ((We_Mean[0] <= 0.0f) || (E_Mean[0] <= 0.0f))
    {
        return false;
    }

    Flux_Rough = E_Mean[0] / We_Mean[0];
    U_Margin = PreFlux->U_Budget_V - Motor_Para.Rs * PreFlux->Iq_Max_A;
    Den = Flux_Rough + Motor_Para.Ld * PreFlux->Iq_Max_A;

    if ((U_Margin <= 0.0f) || (Den <= 0.0f))
    {
        return false;
    }

    We_Max = FLUX_WE_MARGIN_RATIO * U_Margin / Den;

    if (We_Max < FLUX_WE_SPAN_MIN_RATIO * PreFlux->We_Base)
    {
        return false;
    }

    Span = (We_Max - PreFlux->We_Base) / (float)(FLUX_POINT_NUM - 1U);

    for (uint8_t n = 1U; n < FLUX_POINT_NUM; n++)
    {
        We_Point[n] = (float)Dir * (PreFlux->We_Base + Span * (float)n);
    }

    return true;
}

static void Measure(void)
{
    E_Sum += E_Mag;
    We_Sum += Abs_Value(Emf_PLL.State.We);
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
    Sync_Cnt = 0U;
    E_Sum = 0.0f;
    We_Sum = 0.0f;
    Finish_Iq = 0.0f;
    Finish_Init = 0U;
    Emf_Init = 0U;

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
            Emf_Reset(-0.5f * PI_F * (float)Dir, Dir);
            State = FLUX_ACCEL;
        }

        return FAST_CURRENT;
    }

    Emf_Update(Ia_A, Ib_A);

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

            if ((Point == 0U) && !Work_Points_Build(Dir))
            {
                Flux_Fail();
                return FAST_OFF;
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
                Sync_Cnt = 0U;
                IF_Start_Target_Set(We_Point[Point]);
                State = FLUX_ACCEL;
            }
        }
    }

    (void)IF_Start_Run(&Theta_IF, Id_Ref, Iq_Ref);
    *Theta_e = Theta_IF;

    if (State == FLUX_ACCEL)
    {
        if ((IF_Start_State_Get() == IF_HOLD) && Emf_Sync())
        {
            Cnt = 0U;
            State = FLUX_SETTLE;
        }
    }
    else if (State == FLUX_SETTLE)
    {
        if (!Emf_Sync())
        {
            Cnt = 0U;
            State = FLUX_ACCEL;
        }
        else if (++Cnt >= FLUX_SETTLE_CNT)
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
