/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Rs_Ls.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define RS_LS_PROBE_FREQ_HZ       100.0f
#define RS_LS_PROBE_FREQ_STEP_HZ  100.0f
#define RS_LS_PROBE_FREQ_MAX_HZ   300.0f
#define RS_LS_FREQ_MIN_HZ         20.0f
#define RS_LS_FREQ_MAX_HZ         500.0f
#define RS_LS_FREQ_RL_RATIO       0.5f
#define RS_LS_U_RAMP_TIME_S       0.20f
#define RS_LS_PROBE_MEASURE_CYCLE 5U
#define RS_LS_MEASURE_CYCLE       10U

#define RS_LS_PROBE_I_RATIO       0.10f
#define RS_LS_MEASURE_I_RATIO     0.15f
#define RS_LS_MEASURE_I_MAX_RATIO 0.20f
#define RS_LS_ALIGN_I_RATIO       0.20f
#define RS_LS_ALIGN_I_MAX_RATIO   0.50f
#define RS_LS_I_MIN_RATIO         0.02f

#define RS_LS_ALIGN_TIME_S  0.5f
#define RS_LS_ALIGN_CNT     ((uint32_t)(RS_LS_ALIGN_TIME_S / CUR_TS + 0.5f))
#define RS_LS_SETTLE_TIME_S 0.2f
#define RS_LS_SETTLE_CNT    ((uint32_t)(RS_LS_SETTLE_TIME_S / CUR_TS + 0.5f))

#define RS_LS_RS_MIN_OHM    0.0001f
#define RS_LS_RS_MAX_OHM    20.0f
#define RS_LS_LS_MIN_H      1.0e-7f
#define RS_LS_LS_MAX_H      0.1f
#define RS_LS_RS_REPEAT_MAX 0.10f
#define RS_LS_LS_REPEAT_MAX 0.15f

typedef enum
{
    RS_LS_IDLE = 0,
    RS_LS_PROBE_RAMP,
    RS_LS_PROBE_MEASURE,
    RS_LS_ALIGN,
    RS_LS_RAMP,
    RS_LS_SETTLE,
    RS_LS_MEASURE_A,
    RS_LS_MEASURE_B,
    RS_LS_MEASURE_C,
    RS_LS_DONE,
    RS_LS_FAILED,

} Rs_Ls_State_e;

static Rs_Ls_State_e State;
static Rs_Ls_Result_T Result;

static uint32_t Cnt;
static float Phase;
static float Phase_Step;
static float Probe_Freq;
static uint32_t Sample_Per_Cycle;
static float U_Hold;
static float U_Ac;

static uint32_t Ramp_Cnt;
static float Ramp_I_Re;
static float Ramp_I_Im;

static float U_Re;
static float U_Im;
static float I_Re;
static float I_Im;

static float Rs_A;
static float Ls_A;
static float Rs_B;
static float Ls_B;
static float Rs_C;
static float Ls_C;

static bool RL_Temporary;
static bool Align_Pending;
static float Rs_Save;
static float Ld_Save;
static float Lq_Save;

static void Frequency_Set(float Freq_Target)
{
    if (Freq_Target < RS_LS_FREQ_MIN_HZ)
    {
        Freq_Target = RS_LS_FREQ_MIN_HZ;
    }
    else if (Freq_Target > RS_LS_FREQ_MAX_HZ)
    {
        Freq_Target = RS_LS_FREQ_MAX_HZ;
    }

    Sample_Per_Cycle = (uint32_t)(CUR_FREQ_HZ_DEFAULT / Freq_Target + 0.5f);

    if (Sample_Per_Cycle == 0U)
    {
        Sample_Per_Cycle = 1U;
    }

    Phase_Step = TWO_PI_F / (float)Sample_Per_Cycle;
}

static void Ramp_Reset(void)
{
    Ramp_Cnt = 0U;
    Ramp_I_Re = 0.0f;
    Ramp_I_Im = 0.0f;
}

static void Measure_Reset(void)
{
    Cnt = 0U;
    U_Re = 0.0f;
    U_Im = 0.0f;
    I_Re = 0.0f;
    I_Im = 0.0f;
}

static void Voltage_Ramp(float U_Max)
{
    float U_Step;

    if (U_Max <= 0.0f)
    {
        return;
    }

    U_Step = U_Max * CUR_TS / RS_LS_U_RAMP_TIME_S;
    U_Ac += U_Step;

    if (U_Ac > U_Max)
    {
        U_Ac = U_Max;
    }
}

static void Voltage_Backoff(float U_Max)
{
    float U_Step;

    if ((U_Max <= 0.0f) || (U_Ac <= 0.0f))
    {
        return;
    }

    U_Step = U_Max * CUR_TS / RS_LS_U_RAMP_TIME_S;
    U_Ac -= U_Step;

    if (U_Ac < 0.0f)
    {
        U_Ac = 0.0f;
    }
}

static void Rough_RL_Apply(float Rs, float Ls)
{
    if (!RL_Temporary)
    {
        Rs_Save = Motor_Para.Rs;
        Ld_Save = Motor_Para.Ld;
        Lq_Save = Motor_Para.Lq;
        RL_Temporary = true;
    }

    Motor_Para.Rs = Rs;
    Motor_Para.Ld = Ls;
    Motor_Para.Lq = Ls;
    Motor_Para_Update();
    Current_Loop_State_Reset();
}

static void Rough_RL_Restore(void)
{
    if (!RL_Temporary)
    {
        return;
    }

    Motor_Para.Rs = Rs_Save;
    Motor_Para.Ld = Ld_Save;
    Motor_Para.Lq = Lq_Save;
    Motor_Para_Update();
    Current_Loop_State_Reset();
    RL_Temporary = false;
}

static void Fail(void)
{
    Rough_RL_Restore();
    Align_Pending = false;
    Result.Valid = false;
    State = RS_LS_FAILED;
}

static bool Measure_Calc(uint32_t Sample_Cnt, float I_Min, float *Rs, float *Ls)
{
    float Den;
    float Freq;
    float I_Amp;
    float Scale;
    float Z_Re;
    float Z_Im;

    Den = I_Re * I_Re + I_Im * I_Im;

    if (Den <= 0.0f)
    {
        return false;
    }

    Scale = 2.0f / (float)Sample_Cnt;
    I_Amp = Scale * __builtin_sqrtf(Den);

    if (I_Amp < I_Min)
    {
        return false;
    }

    Freq = CUR_FREQ_HZ_DEFAULT / (float)Sample_Per_Cycle;
    Z_Re = (U_Re * I_Re + U_Im * I_Im) / Den;
    Z_Im = (U_Im * I_Re - U_Re * I_Im) / Den;

    *Rs = Z_Re;
    *Ls = Z_Im / (TWO_PI_F * Freq);

    return true;
}

static float Repeat_Ratio(float A, float B)
{
    float Ref;
    float Diff;

    Ref = 0.5f * (A + B);
    Diff = A - B;

    if (Diff < 0.0f)
    {
        Diff = -Diff;
    }

    return (Ref > 0.0f) ? (Diff / Ref) : 1.0e30f;
}

static bool Repeat_Valid(float A, float B, float Max_Ratio)
{
    return Repeat_Ratio(A, B) <= Max_Ratio;
}

static bool Pair_Valid(float Rs_1, float Ls_1, float Rs_2, float Ls_2)
{
    return Repeat_Valid(Rs_1, Rs_2, RS_LS_RS_REPEAT_MAX) &&
           Repeat_Valid(Ls_1, Ls_2, RS_LS_LS_REPEAT_MAX);
}

static float Pair_Score(float Rs_1, float Ls_1, float Rs_2, float Ls_2)
{
    float Rs_Score;
    float Ls_Score;

    Rs_Score = Repeat_Ratio(Rs_1, Rs_2) / RS_LS_RS_REPEAT_MAX;
    Ls_Score = Repeat_Ratio(Ls_1, Ls_2) / RS_LS_LS_REPEAT_MAX;

    return (Rs_Score > Ls_Score) ? Rs_Score : Ls_Score;
}

static bool Third_Result(float *Rs, float *Ls)
{
    bool AC_Valid;
    bool BC_Valid;

    AC_Valid = Pair_Valid(Rs_A, Ls_A, Rs_C, Ls_C);
    BC_Valid = Pair_Valid(Rs_B, Ls_B, Rs_C, Ls_C);

    if (!AC_Valid && !BC_Valid)
    {
        return false;
    }

    if (AC_Valid && (!BC_Valid || (Pair_Score(Rs_A, Ls_A, Rs_C, Ls_C) <= Pair_Score(Rs_B, Ls_B, Rs_C, Ls_C))))
    {
        *Rs = 0.5f * (Rs_A + Rs_C);
        *Ls = 0.5f * (Ls_A + Ls_C);
    }
    else
    {
        *Rs = 0.5f * (Rs_B + Rs_C);
        *Ls = 0.5f * (Ls_B + Ls_C);
    }

    return true;
}

static bool Result_Valid(float Rs, float Ls)
{
    return (Rs > RS_LS_RS_MIN_OHM) && (Rs < RS_LS_RS_MAX_OHM) &&
           (Ls > RS_LS_LS_MIN_H) && (Ls < RS_LS_LS_MAX_H);
}

static bool Probe_Retry(void)
{
    if (Probe_Freq >= RS_LS_PROBE_FREQ_MAX_HZ)
    {
        return false;
    }

    Probe_Freq += RS_LS_PROBE_FREQ_STEP_HZ;
    if (Probe_Freq > RS_LS_PROBE_FREQ_MAX_HZ)
    {
        Probe_Freq = RS_LS_PROBE_FREQ_MAX_HZ;
    }

    Frequency_Set(Probe_Freq);
    Phase = 0.0f;
    U_Ac = 0.0f;
    Ramp_Reset();
    Measure_Reset();
    State = RS_LS_PROBE_RAMP;
    return true;
}

void Rs_Ls_Start(void)
{
    Rough_RL_Restore();

    Result.Rs_Ohm = 0.0f;
    Result.Ls_H = 0.0f;
    Result.Valid = false;

    Phase = 0.0f;
    U_Hold = 0.0f;
    U_Ac = 0.0f;
    Align_Pending = false;

    Probe_Freq = RS_LS_PROBE_FREQ_HZ;
    Frequency_Set(Probe_Freq);
    Ramp_Reset();
    Measure_Reset();

    State = RS_LS_PROBE_RAMP;
}

void Rs_Ls_Abort(void)
{
    Rough_RL_Restore();
    Align_Pending = false;
    Result.Valid = false;
    State = RS_LS_IDLE;
}

bool Rs_Ls_Active(void)
{
    return (State != RS_LS_IDLE) && (State != RS_LS_DONE) && (State != RS_LS_FAILED);
}

Motor_Fast_Mode_e Rs_Ls_Run(float Ialpha_A,
                            float *Theta_e,
                            float *Id_Ref,
                            float *Iq_Ref,
                            float *Ualpha_V,
                            float *Ubeta_V)
{
    const Ident_Envelope_T *Envelope;
    float Sin;
    float Cos;
    float Ramp_I_Amp;
    float Rs;
    float Ls;
    float Freq_Target;
    float I_Probe;
    float I_Measure;
    float I_Measure_Max;
    float I_Align;
    float I_Align_Max;
    float I_Min;
    float I_Target;
    float I_Soft;
    float I_Abs;
    float U_Ac_Max;
    float U_Hold_Abs;
    uint32_t Sample_Cnt;
    bool Align_Done;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if ((State == RS_LS_IDLE) || (State == RS_LS_DONE) || (State == RS_LS_FAILED))
    {
        return FAST_OFF;
    }

    Envelope = Identification_Envelope_Get();
    if ((Envelope->I_Max <= 0.0f) || (Envelope->U_Max <= 0.0f))
    {
        Fail();
        return FAST_OFF;
    }

    I_Min = RS_LS_I_MIN_RATIO * Envelope->I_Max;
    I_Probe = RS_LS_PROBE_I_RATIO * Envelope->I_Max;
    if (I_Probe < I_Min)
    {
        I_Probe = I_Min;
    }

    I_Measure = RS_LS_MEASURE_I_RATIO * Envelope->I_Max;
    if (I_Measure < I_Min)
    {
        I_Measure = I_Min;
    }

    I_Measure_Max = RS_LS_MEASURE_I_MAX_RATIO * Envelope->I_Max;
    I_Align = RS_LS_ALIGN_I_RATIO * Envelope->I_Max;
    I_Align_Max = RS_LS_ALIGN_I_MAX_RATIO * Envelope->I_Max;

    if (State == RS_LS_ALIGN)
    {
        if (!Align_Pending)
        {
            Align_Done = Align_Current(I_Align, RS_LS_ALIGN_CNT, Id_Ref, Iq_Ref);

            if (Align_Done)
            {
                Align_Pending = true;
            }

            return FAST_CURRENT;
        }

        /* Capture Ud after the final FAST_CURRENT cycle has completed. */
        U_Hold = Motor_Run.Ud;
        Rough_RL_Restore();
        U_Ac = 0.0f;
        Phase = 0.0f;
        Ramp_Reset();
        Align_Pending = false;
        State = RS_LS_RAMP;
    }

    if ((State == RS_LS_PROBE_RAMP) || (State == RS_LS_PROBE_MEASURE))
    {
        U_Ac_Max = Envelope->U_Max;
        I_Soft = I_Measure_Max;
    }
    else
    {
        U_Hold_Abs = (U_Hold >= 0.0f) ? U_Hold : -U_Hold;

        if (U_Hold_Abs >= Envelope->U_Max)
        {
            Fail();
            return FAST_OFF;
        }

        U_Ac_Max = Envelope->U_Max - U_Hold_Abs;
        I_Soft = I_Align_Max;
    }

    if (U_Ac > U_Ac_Max)
    {
        U_Ac = U_Ac_Max;
    }

    I_Abs = (Ialpha_A >= 0.0f) ? Ialpha_A : -Ialpha_A;
    SinCos(Phase, &Sin, &Cos);

    if ((State == RS_LS_PROBE_RAMP) || (State == RS_LS_PROBE_MEASURE))
    {
        *Ualpha_V = U_Ac * Sin;
    }
    else
    {
        *Ualpha_V = U_Hold + U_Ac * Sin;
    }

    if ((State == RS_LS_PROBE_RAMP) || (State == RS_LS_RAMP))
    {
        Ramp_I_Re += Ialpha_A * Cos;
        Ramp_I_Im -= Ialpha_A * Sin;
        Ramp_Cnt++;

        if (I_Abs < I_Soft)
        {
            Voltage_Ramp(U_Ac_Max);
        }
        else
        {
            Voltage_Backoff(U_Ac_Max);
        }

        if (Ramp_Cnt >= Sample_Per_Cycle)
        {
            Ramp_I_Amp = (2.0f / (float)Sample_Per_Cycle) *
                         __builtin_sqrtf(Ramp_I_Re * Ramp_I_Re + Ramp_I_Im * Ramp_I_Im);

            Ramp_Reset();
            I_Target = (State == RS_LS_PROBE_RAMP) ? I_Probe : I_Measure;

            if (Ramp_I_Amp > I_Measure_Max)
            {
                if (Ramp_I_Amp > 0.0f)
                {
                    U_Ac *= I_Target / Ramp_I_Amp;
                }
            }
            else if (Ramp_I_Amp >= I_Target)
            {
                if (State == RS_LS_PROBE_RAMP)
                {
                    Measure_Reset();
                    State = RS_LS_PROBE_MEASURE;
                }
                else
                {
                    Cnt = 0U;
                    State = RS_LS_SETTLE;
                }
            }
            else if (U_Ac >= U_Ac_Max)
            {
                if (Ramp_I_Amp >= I_Min)
                {
                    if (State == RS_LS_PROBE_RAMP)
                    {
                        Measure_Reset();
                        State = RS_LS_PROBE_MEASURE;
                    }
                    else
                    {
                        Cnt = 0U;
                        State = RS_LS_SETTLE;
                    }
                }
                else
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }
            }
        }
    }
    else if (State == RS_LS_PROBE_MEASURE)
    {
        U_Re += *Ualpha_V * Cos;
        U_Im -= *Ualpha_V * Sin;
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Cnt++;

        Sample_Cnt = Sample_Per_Cycle * RS_LS_PROBE_MEASURE_CYCLE;
        if (Cnt >= Sample_Cnt)
        {
            if (!Measure_Calc(Sample_Cnt, I_Min, &Rs, &Ls))
            {
                Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

            if ((Rs <= RS_LS_RS_MIN_OHM) || (Rs >= RS_LS_RS_MAX_OHM))
            {
                Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

            if ((Ls <= RS_LS_LS_MIN_H) || (Ls >= RS_LS_LS_MAX_H))
            {
                if (Probe_Retry())
                {
                    *Ualpha_V = 0.0f;
                    return FAST_VOLTAGE;
                }

                Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

            Freq_Target = RS_LS_FREQ_RL_RATIO * Rs / (TWO_PI_F * Ls);
            Frequency_Set(Freq_Target);
            Rough_RL_Apply(Rs, Ls);
            Align_Reset();
            Align_Pending = false;
            State = RS_LS_ALIGN;
            *Ualpha_V = 0.0f;
            return FAST_VOLTAGE;
        }
    }
    else if (State == RS_LS_SETTLE)
    {
        Cnt++;

        if (Cnt >= RS_LS_SETTLE_CNT)
        {
            Measure_Reset();
            State = RS_LS_MEASURE_A;
        }
    }
    else
    {
        U_Re += *Ualpha_V * Cos;
        U_Im -= *Ualpha_V * Sin;
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Cnt++;

        Sample_Cnt = Sample_Per_Cycle * RS_LS_MEASURE_CYCLE;
        if (Cnt >= Sample_Cnt)
        {
            if (State == RS_LS_MEASURE_A)
            {
                if (!Measure_Calc(Sample_Cnt, I_Min, &Rs_A, &Ls_A) || !Result_Valid(Rs_A, Ls_A))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Measure_Reset();
                State = RS_LS_MEASURE_B;
            }
            else if (State == RS_LS_MEASURE_B)
            {
                if (!Measure_Calc(Sample_Cnt, I_Min, &Rs_B, &Ls_B) || !Result_Valid(Rs_B, Ls_B))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (Pair_Valid(Rs_A, Ls_A, Rs_B, Ls_B))
                {
                    Result.Rs_Ohm = 0.5f * (Rs_A + Rs_B);
                    Result.Ls_H = 0.5f * (Ls_A + Ls_B);
                    Result.Valid = true;
                    State = RS_LS_DONE;
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Measure_Reset();
                State = RS_LS_MEASURE_C;
            }
            else
            {
                if (!Measure_Calc(Sample_Cnt, I_Min, &Rs_C, &Ls_C) || !Result_Valid(Rs_C, Ls_C))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (Third_Result(&Result.Rs_Ohm, &Result.Ls_H))
                {
                    Result.Valid = true;
                    State = RS_LS_DONE;
                }
                else
                {
                    Fail();
                }

                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }
        }
    }

    Phase += Phase_Step;

    if (Phase >= TWO_PI_F)
    {
        Phase -= TWO_PI_F;
    }

    return FAST_VOLTAGE;
}

const Rs_Ls_Result_T *Rs_Ls_Result_Get(void)
{
    return &Result;
}