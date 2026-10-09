/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Rs_Ls.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_Config.h"
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
#define RS_LS_NOISE_CYCLE         20U
#define RS_LS_PROBE_MEASURE_CYCLE 5U
#define RS_LS_MEASURE_CYCLE       10U

/* Rs/Ls working point is defined in the current domain. */
#define RS_LS_I_DC_SHARE          0.65f
#define RS_LS_I_AC_SHARE          0.25f
#define RS_LS_SNR_POWER_MIN       100.0f
#define RS_LS_COH_SQ_MIN          0.95f

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
    RS_LS_NOISE,
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
static float U_DC;
static float U_AC;

static uint32_t Ramp_Cnt;
static float Ramp_I_Re;
static float Ramp_I_Im;

static float I_DC_Sum;
static float U_Re;
static float U_Im;
static float I_Re;
static float I_Im;

static uint32_t Coh_Cnt;
static float Coh_I_Pow;

static float I_Noise;
static float SNR_Pow;
static float Coh_Sq;

static float Rs_A;
static float Ls_A;
static float Rs_B;
static float Ls_B;
static float Rs_C;
static float Ls_C;

static bool RL_Temporary;
static bool Align_Pending;
static bool Freq_Refined;
static float Rs_Save;
static float Ld_Save;
static float Lq_Save;
static uint8_t Current_Tune_Source_Save;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

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
    I_DC_Sum = 0.0f;
    U_Re = 0.0f;
    U_Im = 0.0f;
    I_Re = 0.0f;
    I_Im = 0.0f;
    Coh_Cnt = 0U;
    Coh_I_Pow = 0.0f;
}

static float Voltage_Target_Update(float U,
                                   float I_Meas,
                                   float I_Target,
                                   float U_Max,
                                   float U_Share,
                                   float Window_S)
{
    float I_Next;
    float I_Step;
    float U_Step;

    if ((I_Target <= 0.0f) || (U_Max <= 0.0f))
    {
        return 0.0f;
    }

    if ((U > 0.0f) && (I_Meas > 0.0f))
    {
        I_Step = I_Target * Window_S / RS_LS_U_RAMP_TIME_S;
        I_Next = I_Meas + I_Step;

        if (I_Next > I_Target)
        {
            I_Next = I_Target;
        }

        U *= I_Next / I_Meas;
    }
    else
    {
        /* Zero voltage must bootstrap additively; measured AC noise is non-zero. */
        U_Step = U_Max * U_Share * CUR_TS / RS_LS_U_RAMP_TIME_S;
        U += U_Step;
    }

    if (U > U_Max)
    {
        U = U_Max;
    }

    return U;
}

static void Probe_Voltage_Limit(float U_Max)
{
    float Scale;
    float U_Sum;

    U_Sum = U_DC + U_AC;
    if ((U_Sum > U_Max) && (U_Sum > 0.0f))
    {
        Scale = U_Max / U_Sum;
        U_DC *= Scale;
        U_AC *= Scale;
    }
}

static void Rough_RL_Apply(float Rs, float Ls)
{
    if (!RL_Temporary)
    {
        Rs_Save = Motor_Para.Rs;
        Ld_Save = Motor_Para.Ld;
        Lq_Save = Motor_Para.Lq;
        Current_Tune_Source_Save = Control_Current_Tune_Source;
        RL_Temporary = true;
    }

    Motor_Para.Rs = Rs;
    Motor_Para.Ld = Ls;
    Motor_Para.Lq = Ls;
    Control_Current_Tune_Source = CTRL_TUNE_BANDWIDTH;
    Current_Tuning_Update();
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
    Control_Current_Tune_Source = Current_Tune_Source_Save;

    Current_Tuning_Update();
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

static bool Measure_Calc(uint32_t Sample_Cnt, float I_Min, float *I_Amp, float *Rs, float *Ls)
{
    float Den;
    float Freq;
    float Scale;
    float Z_Re;
    float Z_Im;

    Den = I_Re * I_Re + I_Im * I_Im;

    if (Den <= 0.0f)
    {
        return false;
    }

    Scale = 2.0f / (float)Sample_Cnt;
    *I_Amp = Scale * __builtin_sqrtf(Den);

    if (*I_Amp < I_Min)
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
    U_DC = 0.0f;
    U_AC = 0.0f;
    I_Noise = 0.0f;
    SNR_Pow = 0.0f;
    Coh_Sq = 0.0f;
    Ramp_Reset();
    Measure_Reset();
    State = RS_LS_NOISE;
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
    U_DC = 0.0f;
    U_AC = 0.0f;
    I_Noise = 0.0f;
    SNR_Pow = 0.0f;
    Coh_Sq = 0.0f;
    Align_Pending = false;
    Freq_Refined = false;

    Probe_Freq = RS_LS_PROBE_FREQ_HZ;
    Frequency_Set(Probe_Freq);
    Ramp_Reset();
    Measure_Reset();

    State = RS_LS_NOISE;
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
    float Scale;
    float Ramp_I_Amp;
    float I_Cyc_Re;
    float I_Cyc_Im;
    float Coh_Num;
    float Coh_Den;
    float I_Amp;
    float I_DC_Meas;
    float Rs;
    float Ls;
    float Xs;
    float Freq_Target;
    float Freq;
    float I_Min;
    float I_DC_Target;
    float I_AC_Target;
    float U_AC_Max;
    float U_Hold_Abs;
    float Window_S;
    uint32_t Sample_Cnt;
    bool Align_Done;
    bool DC_Ready;
    bool AC_Ready;

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

    I_Min = IDENT_RL_IAC_MIN_A;
    I_AC_Target = Motor_Config.RL_I_Peak_A * IDENT_RL_IAC_RATIO;
    I_DC_Target = Motor_Config.RL_I_Peak_A - I_AC_Target;

    if ((I_DC_Target <= 0.0f) || (I_AC_Target < I_Min))
    {
        Fail();
        return FAST_OFF;
    }

    if (State == RS_LS_ALIGN)
    {
        if (!Align_Pending)
        {
            Align_Done = Align_Current(I_DC_Target, RS_LS_ALIGN_CNT, Id_Ref, Iq_Ref);

            if (Align_Done)
            {
                Align_Pending = true;
                Cnt = 0U;
                U_Hold = 0.0f;
            }

            return FAST_CURRENT;
        }

        /* Average the completed FAST_CURRENT Ud over one injection cycle. */
        U_Hold += Motor_Run.Ud;
        Cnt++;

        if (Cnt < Sample_Per_Cycle)
        {
            *Id_Ref = I_DC_Target;
            *Iq_Ref = 0.0f;
            return FAST_CURRENT;
        }

        U_Hold /= (float)Cnt;
        Rough_RL_Restore();
        Phase = 0.0f;
        Ramp_Reset();
        Align_Pending = false;
        State = RS_LS_RAMP;
    }

    if ((State == RS_LS_NOISE) || (State == RS_LS_PROBE_RAMP) || (State == RS_LS_PROBE_MEASURE))
    {
        Probe_Voltage_Limit(Envelope->U_Max);
    }
    else
    {
        U_Hold_Abs = Abs_Value(U_Hold);
        if (U_Hold_Abs >= Envelope->U_Max)
        {
            Fail();
            return FAST_OFF;
        }

        U_AC_Max = Envelope->U_Max - U_Hold_Abs;
        if (U_AC > U_AC_Max)
        {
            U_AC = U_AC_Max;
        }
    }

    SinCos(Phase, &Sin, &Cos);

    if ((State == RS_LS_NOISE) || (State == RS_LS_PROBE_RAMP) || (State == RS_LS_PROBE_MEASURE))
    {
        *Ualpha_V = U_DC + U_AC * Sin;
    }
    else
    {
        *Ualpha_V = U_Hold + U_AC * Sin;
    }

    if (State == RS_LS_NOISE)
    {
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Cnt++;

        Sample_Cnt = Sample_Per_Cycle * RS_LS_PROBE_MEASURE_CYCLE;
        if (Cnt >= Sample_Cnt)
        {
            Scale = 2.0f / (float)Sample_Cnt;
            I_Amp = Scale * __builtin_sqrtf(I_Re * I_Re + I_Im * I_Im);
            Coh_I_Pow += I_Amp * I_Amp;
            Coh_Cnt++;
            Cnt = 0U;
            I_Re = 0.0f;
            I_Im = 0.0f;

            if ((Coh_Cnt * RS_LS_PROBE_MEASURE_CYCLE) >= RS_LS_NOISE_CYCLE)
            {
                I_Noise = __builtin_sqrtf(Coh_I_Pow / (float)Coh_Cnt);
                Measure_Reset();
                Ramp_Reset();
                State = RS_LS_PROBE_RAMP;
            }
        }
    }
    else if (State == RS_LS_PROBE_RAMP)
    {
        I_DC_Sum += Ialpha_A;
        Ramp_I_Re += Ialpha_A * Cos;
        Ramp_I_Im -= Ialpha_A * Sin;
        Ramp_Cnt++;

        if (Ramp_Cnt >= Sample_Per_Cycle)
        {
            I_DC_Meas = I_DC_Sum / (float)Sample_Per_Cycle;
            I_Amp = (2.0f / (float)Sample_Per_Cycle) *
                    __builtin_sqrtf(Ramp_I_Re * Ramp_I_Re + Ramp_I_Im * Ramp_I_Im);
            DC_Ready = I_DC_Meas >= I_DC_Target;
            AC_Ready = I_Amp >= I_AC_Target;

            if (DC_Ready && AC_Ready)
            {
                Ramp_Reset();
                Measure_Reset();
                State = RS_LS_PROBE_MEASURE;
            }
            else
            {
                if (((U_DC + U_AC) >= (0.999f * Envelope->U_Max)) &&
                    (!DC_Ready || !AC_Ready))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Window_S = (float)Sample_Per_Cycle * CUR_TS;

                if (!DC_Ready)
                {
                    U_DC = Voltage_Target_Update(U_DC, I_DC_Meas, I_DC_Target,
                                                 Envelope->U_Max, RS_LS_I_DC_SHARE, Window_S);
                }

                if (!AC_Ready)
                {
                    U_AC = Voltage_Target_Update(U_AC, I_Amp, I_AC_Target,
                                                 Envelope->U_Max, RS_LS_I_AC_SHARE, Window_S);
                }

                Probe_Voltage_Limit(Envelope->U_Max);
                I_DC_Sum = 0.0f;
                Ramp_Reset();
            }
        }
    }
    else if (State == RS_LS_RAMP)
    {
        Ramp_I_Re += Ialpha_A * Cos;
        Ramp_I_Im -= Ialpha_A * Sin;
        Ramp_Cnt++;

        if (Ramp_Cnt >= Sample_Per_Cycle)
        {
            Ramp_I_Amp = (2.0f / (float)Sample_Per_Cycle) *
                         __builtin_sqrtf(Ramp_I_Re * Ramp_I_Re + Ramp_I_Im * Ramp_I_Im);
            Ramp_Reset();

            if (Ramp_I_Amp >= I_AC_Target)
            {
                Cnt = 0U;
                State = RS_LS_SETTLE;
            }
            else
            {
                U_Hold_Abs = Abs_Value(U_Hold);
                U_AC_Max = Envelope->U_Max - U_Hold_Abs;

                if (U_AC >= U_AC_Max)
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Window_S = (float)Sample_Per_Cycle * CUR_TS;
                U_AC = Voltage_Target_Update(U_AC, Ramp_I_Amp, I_AC_Target,
                                             U_AC_Max, 1.0f, Window_S);
            }
        }
    }
    else if (State == RS_LS_PROBE_MEASURE)
    {
        I_DC_Sum += Ialpha_A;
        U_Re += *Ualpha_V * Cos;
        U_Im -= *Ualpha_V * Sin;
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Ramp_I_Re += Ialpha_A * Cos;
        Ramp_I_Im -= Ialpha_A * Sin;
        Ramp_Cnt++;
        Cnt++;

        if (Ramp_Cnt >= Sample_Per_Cycle)
        {
            Scale = 2.0f / (float)Sample_Per_Cycle;
            I_Cyc_Re = Scale * Ramp_I_Re;
            I_Cyc_Im = Scale * Ramp_I_Im;
            Coh_I_Pow += I_Cyc_Re * I_Cyc_Re + I_Cyc_Im * I_Cyc_Im;
            Coh_Cnt++;
            Ramp_Reset();
        }

        Sample_Cnt = Sample_Per_Cycle * RS_LS_PROBE_MEASURE_CYCLE;
        if (Cnt >= Sample_Cnt)
        {
            Scale = 2.0f / (float)Sample_Cnt;
            I_Amp = Scale * __builtin_sqrtf(I_Re * I_Re + I_Im * I_Im);
            I_DC_Meas = I_DC_Sum / (float)Sample_Cnt;
            Scale = 2.0f / (float)Sample_Per_Cycle;
            I_Cyc_Re = Scale * I_Re;
            I_Cyc_Im = Scale * I_Im;
            Coh_Num = I_Cyc_Re * I_Cyc_Re + I_Cyc_Im * I_Cyc_Im;
            Coh_Den = (float)Coh_Cnt * Coh_I_Pow;
            SNR_Pow = (I_Noise > 0.0f) ? (I_Amp * I_Amp / (I_Noise * I_Noise)) : 1.0e30f;
            Coh_Sq = (Coh_Den > 0.0f) ? (Coh_Num / Coh_Den) : 0.0f;

            DC_Ready = I_DC_Meas >= I_DC_Target;
            AC_Ready = I_Amp >= I_AC_Target;
            if (!DC_Ready || !AC_Ready)
            {
                if (((U_DC + U_AC) >= (0.999f * Envelope->U_Max)) &&
                    (!DC_Ready || !AC_Ready))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Window_S = (float)Sample_Cnt * CUR_TS;

                if (!DC_Ready)
                {
                    U_DC = Voltage_Target_Update(U_DC, I_DC_Meas, I_DC_Target,
                                                 Envelope->U_Max, RS_LS_I_DC_SHARE, Window_S);
                }

                if (!AC_Ready)
                {
                    U_AC = Voltage_Target_Update(U_AC, I_Amp, I_AC_Target,
                                                 Envelope->U_Max, RS_LS_I_AC_SHARE, Window_S);
                }

                Probe_Voltage_Limit(Envelope->U_Max);
                Ramp_Reset();
                Measure_Reset();
                State = RS_LS_PROBE_RAMP;
            }
            else if ((I_Amp < I_Min) || (SNR_Pow < RS_LS_SNR_POWER_MIN) || (Coh_Sq < RS_LS_COH_SQ_MIN))
            {
                if (!Probe_Retry())
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                *Ualpha_V = 0.0f;
                return FAST_VOLTAGE;
            }
            else if (!Measure_Calc(Sample_Cnt, I_Min, &I_Amp, &Rs, &Ls))
            {
                Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }
            else if ((Rs <= RS_LS_RS_MIN_OHM) || (Rs >= RS_LS_RS_MAX_OHM))
            {
                Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }
            else if ((Ls <= RS_LS_LS_MIN_H) || (Ls >= RS_LS_LS_MAX_H))
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
            else
            {
                Freq_Target = RS_LS_FREQ_RL_RATIO * Rs / (TWO_PI_F * Ls);
                Frequency_Set(Freq_Target);
                Freq = CUR_FREQ_HZ_DEFAULT / (float)Sample_Per_Cycle;
                Xs = TWO_PI_F * Freq * Ls;
                U_AC = I_AC_Target * __builtin_sqrtf(Rs * Rs + Xs * Xs);
                Rough_RL_Apply(Rs, Ls);
                Align_Reset();
                Align_Pending = false;
                State = RS_LS_ALIGN;
                *Ualpha_V = 0.0f;
                return FAST_VOLTAGE;
            }
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
                if (!Measure_Calc(Sample_Cnt, I_Min, &I_Amp, &Rs_A, &Ls_A) || !Result_Valid(Rs_A, Ls_A))
                {
                    Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (!Freq_Refined)
                {
                    Freq_Target = RS_LS_FREQ_RL_RATIO * Rs_A / (TWO_PI_F * Ls_A);
                    Frequency_Set(Freq_Target);
                    Phase = 0.0f;
                    Freq_Refined = true;
                    Ramp_Reset();
                    Measure_Reset();
                    State = RS_LS_RAMP;
                    *Ualpha_V = 0.0f;
                    return FAST_VOLTAGE;
                }

                Measure_Reset();
                State = RS_LS_MEASURE_B;
            }
            else if (State == RS_LS_MEASURE_B)
            {
                if (!Measure_Calc(Sample_Cnt, I_Min, &I_Amp, &Rs_B, &Ls_B) || !Result_Valid(Rs_B, Ls_B))
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
                if (!Measure_Calc(Sample_Cnt, I_Min, &I_Amp, &Rs_C, &Ls_C) || !Result_Valid(Rs_C, Ls_C))
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
