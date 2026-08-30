/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Rs_Ls.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Identification.h"
#include "Math.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define RS_LS_PROBE_FREQ_HZ       100.0f
#define RS_LS_FREQ_MIN_HZ         20.0f
#define RS_LS_FREQ_MAX_HZ         500.0f
#define RS_LS_FREQ_RL_RATIO       0.5f
#define RS_LS_U_RAMP_TIME_S       0.20f
#define RS_LS_PROBE_MEASURE_CYCLE 5U
#define RS_LS_MEASURE_CYCLE       10U

#define RS_LS_ALIGN_BW_RATIO 0.20f
#define RS_LS_ALIGN_TIME_S   0.5f
#define RS_LS_ALIGN_CNT      ((uint32_t)(RS_LS_ALIGN_TIME_S / CUR_TS + 0.5f))
#define RS_LS_SETTLE_TIME_S  0.2f
#define RS_LS_SETTLE_CNT     ((uint32_t)(RS_LS_SETTLE_TIME_S / CUR_TS + 0.5f))

#define RS_LS_RS_MIN_OHM    0.0001f
#define RS_LS_RS_MAX_OHM    20.0f
#define RS_LS_LS_MIN_H      1.0e-7f
#define RS_LS_LS_MAX_H      0.1f
#define RS_LS_RS_REPEAT_MAX 0.10f
#define RS_LS_LS_REPEAT_MAX 0.15f

/* Diagnostic-only terminal stage values returned after RS_LS_FAILED. */
#define RS_LS_FAIL_STAGE_NO_SIGNAL       10U
#define RS_LS_FAIL_STAGE_CURRENT_LOW     11U
#define RS_LS_FAIL_STAGE_INVALID_RS      12U
#define RS_LS_FAIL_STAGE_INVALID_LS      13U
#define RS_LS_FAIL_STAGE_REPEATABILITY   14U
#define RS_LS_FAIL_STAGE_ENVELOPE        15U
#define RS_LS_FAIL_STAGE_ALIGN_VOLTAGE   16U

static Rs_Ls_State_e Rs_Ls_State = RS_LS_IDLE;
static volatile uint8_t Fail_Stage = (uint8_t)RS_LS_FAILED;
static Rs_Ls_Result_T Rs_Ls_Result = { 0 };

static uint32_t Rs_Ls_Cnt = 0U;
static float Rs_Ls_Phase = 0.0f;
static float Rs_Ls_Phase_Step = 0.0f;
static float Rs_Ls_Freq_Hz = RS_LS_PROBE_FREQ_HZ;
static uint32_t Rs_Ls_Sample_Per_Cycle = 0U;
static uint32_t Rs_Ls_Probe_Measure_Cnt = 0U;
static uint32_t Rs_Ls_Measure_Cnt = 0U;
static float Rs_Ls_U_Hold_V = 0.0f;
static float Rs_Ls_U_Ac_V = 0.0f;

static uint32_t Ramp_Cnt = 0U;
static float Ramp_I_Re = 0.0f;
static float Ramp_I_Im = 0.0f;

static float U_Re = 0.0f;
static float U_Im = 0.0f;
static float I_Re = 0.0f;
static float I_Im = 0.0f;

static float Rs_Rough = 0.0f;
static float Ls_Rough = 0.0f;
static float Rs_A = 0.0f;
static float Ls_A = 0.0f;
static float Rs_B = 0.0f;
static float Ls_B = 0.0f;
static float Rs_C = 0.0f;
static float Ls_C = 0.0f;
static bool Measure_Third = false;

static bool PI_Saved = false;
static bool Align_Pending = false;
static float Id_Kp_Save = 0.0f;
static float Id_Ki_Save = 0.0f;
static float Iq_Kp_Save = 0.0f;
static float Iq_Ki_Save = 0.0f;

static void Frequency_Set(float Freq_Target_Hz)
{
    if (Freq_Target_Hz < RS_LS_FREQ_MIN_HZ)
    {
        Freq_Target_Hz = RS_LS_FREQ_MIN_HZ;
    }
    else if (Freq_Target_Hz > RS_LS_FREQ_MAX_HZ)
    {
        Freq_Target_Hz = RS_LS_FREQ_MAX_HZ;
    }

    Rs_Ls_Sample_Per_Cycle = (uint32_t)(CUR_FREQ_HZ_DEFAULT / Freq_Target_Hz + 0.5f);

    if (Rs_Ls_Sample_Per_Cycle == 0U)
    {
        Rs_Ls_Sample_Per_Cycle = 1U;
    }

    Rs_Ls_Freq_Hz = CUR_FREQ_HZ_DEFAULT / (float)Rs_Ls_Sample_Per_Cycle;
    Rs_Ls_Phase_Step = TWO_PI_F / (float)Rs_Ls_Sample_Per_Cycle;
    Rs_Ls_Probe_Measure_Cnt = Rs_Ls_Sample_Per_Cycle * RS_LS_PROBE_MEASURE_CYCLE;
    Rs_Ls_Measure_Cnt = Rs_Ls_Sample_Per_Cycle * RS_LS_MEASURE_CYCLE;
}

static void Ramp_Reset(void)
{
    Ramp_Cnt = 0U;
    Ramp_I_Re = 0.0f;
    Ramp_I_Im = 0.0f;
}

static void Measure_Reset(void)
{
    Rs_Ls_Cnt = 0U;
    U_Re = 0.0f;
    U_Im = 0.0f;
    I_Re = 0.0f;
    I_Im = 0.0f;
}

static void Voltage_Ramp(float U_Max_V)
{
    float U_Step_V;

    if (U_Max_V <= 0.0f)
    {
        return;
    }

    U_Step_V = U_Max_V * CUR_TS / RS_LS_U_RAMP_TIME_S;
    Rs_Ls_U_Ac_V += U_Step_V;

    if (Rs_Ls_U_Ac_V > U_Max_V)
    {
        Rs_Ls_U_Ac_V = U_Max_V;
    }
}

static void Voltage_Backoff(float U_Max_V)
{
    float U_Step_V;

    if ((U_Max_V <= 0.0f) || (Rs_Ls_U_Ac_V <= 0.0f))
    {
        return;
    }

    U_Step_V = U_Max_V * CUR_TS / RS_LS_U_RAMP_TIME_S;
    Rs_Ls_U_Ac_V -= U_Step_V;

    if (Rs_Ls_U_Ac_V < 0.0f)
    {
        Rs_Ls_U_Ac_V = 0.0f;
    }
}

static void PI_State_Reset(void)
{
    Id_Ctrl.State.Int = 0.0f;
    Iq_Ctrl.State.Int = 0.0f;
    Id_Ctrl.Sig.Out = 0.0f;
    Iq_Ctrl.Sig.Out = 0.0f;
}

static void PI_Temporary_Set(float Rs_Ohm, float Ls_H)
{
    float Wc;

    if (!PI_Saved)
    {
        Id_Kp_Save = Id_Ctrl.Para.Kp;
        Id_Ki_Save = Id_Ctrl.Para.Ki;
        Iq_Kp_Save = Iq_Ctrl.Para.Kp;
        Iq_Ki_Save = Iq_Ctrl.Para.Ki;
        PI_Saved = true;
    }

    Wc = TWO_PI_F * CUR_BW_HZ_DEFAULT * RS_LS_ALIGN_BW_RATIO;

    Id_Ctrl.Para.Kp = Ls_H * Wc;
    Id_Ctrl.Para.Ki = Rs_Ohm * Wc;
    Iq_Ctrl.Para.Kp = Ls_H * Wc;
    Iq_Ctrl.Para.Ki = Rs_Ohm * Wc;

    PI_State_Reset();
}

static void PI_Restore(void)
{
    if (!PI_Saved)
    {
        return;
    }

    Id_Ctrl.Para.Kp = Id_Kp_Save;
    Id_Ctrl.Para.Ki = Id_Ki_Save;
    Iq_Ctrl.Para.Kp = Iq_Kp_Save;
    Iq_Ctrl.Para.Ki = Iq_Ki_Save;

    PI_State_Reset();
    PI_Saved = false;
}

static bool Measure_Calc(uint32_t Sample_Cnt, float *Rs_Ohm, float *Ls_H)
{
    const Ident_Envelope_T *Envelope;
    float Den;
    float I_Amp;
    float Scale;
    float Z_Re;
    float Z_Im;

    Envelope = Identification_Envelope_Get();
    Den = I_Re * I_Re + I_Im * I_Im;

    if (Den <= 0.0f)
    {
        Fail_Stage = RS_LS_FAIL_STAGE_NO_SIGNAL;
        return false;
    }

    Scale = 2.0f / (float)Sample_Cnt;
    I_Amp = Scale * __builtin_sqrtf(Den);

    if (I_Amp < Envelope->I_Min_A)
    {
        Fail_Stage = RS_LS_FAIL_STAGE_CURRENT_LOW;
        return false;
    }

    Z_Re = (U_Re * I_Re + U_Im * I_Im) / Den;
    Z_Im = (U_Im * I_Re - U_Re * I_Im) / Den;

    *Rs_Ohm = Z_Re;
    *Ls_H = Z_Im / (TWO_PI_F * Rs_Ls_Freq_Hz);

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

static bool Best_Pair_Result(float *Rs_Ohm, float *Ls_H)
{
    float Score_AB;
    float Score_AC;
    float Score_BC;

    Score_AB = Pair_Valid(Rs_A, Ls_A, Rs_B, Ls_B) ? Pair_Score(Rs_A, Ls_A, Rs_B, Ls_B) : 1.0e30f;
    Score_AC = Pair_Valid(Rs_A, Ls_A, Rs_C, Ls_C) ? Pair_Score(Rs_A, Ls_A, Rs_C, Ls_C) : 1.0e30f;
    Score_BC = Pair_Valid(Rs_B, Ls_B, Rs_C, Ls_C) ? Pair_Score(Rs_B, Ls_B, Rs_C, Ls_C) : 1.0e30f;

    if ((Score_AB <= Score_AC) && (Score_AB <= Score_BC))
    {
        if (Score_AB >= 1.0e30f)
        {
            return false;
        }

        *Rs_Ohm = 0.5f * (Rs_A + Rs_B);
        *Ls_H = 0.5f * (Ls_A + Ls_B);
        return true;
    }

    if (Score_AC <= Score_BC)
    {
        if (Score_AC >= 1.0e30f)
        {
            return false;
        }

        *Rs_Ohm = 0.5f * (Rs_A + Rs_C);
        *Ls_H = 0.5f * (Ls_A + Ls_C);
        return true;
    }

    if (Score_BC >= 1.0e30f)
    {
        return false;
    }

    *Rs_Ohm = 0.5f * (Rs_B + Rs_C);
    *Ls_H = 0.5f * (Ls_B + Ls_C);
    return true;
}

static void Rs_Ls_Fail_Stage(uint8_t Stage)
{
    Fail_Stage = Stage;
    Rs_Ls_Fail();
}

static bool Result_Range_Check(float Rs_Ohm, float Ls_H)
{
    if ((Rs_Ohm <= RS_LS_RS_MIN_OHM) || (Rs_Ohm >= RS_LS_RS_MAX_OHM))
    {
        Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_INVALID_RS);
        return false;
    }

    if ((Ls_H <= RS_LS_LS_MIN_H) || (Ls_H >= RS_LS_LS_MAX_H))
    {
        Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_INVALID_LS);
        return false;
    }

    return true;
}

void Rs_Ls_Reset(void)
{
    PI_Restore();

    Fail_Stage = (uint8_t)RS_LS_FAILED;
    Rs_Ls_Result.Rs_Ohm = 0.0f;
    Rs_Ls_Result.Ls_H = 0.0f;
    Rs_Ls_Result.Valid = false;
    Rs_Ls_Result.Probe_U_Re = 0.0f;
    Rs_Ls_Result.Probe_U_Im = 0.0f;
    Rs_Ls_Result.Probe_I_Re = 0.0f;
    Rs_Ls_Result.Probe_I_Im = 0.0f;

    Rs_Ls_Cnt = 0U;
    Rs_Ls_Phase = 0.0f;
    Rs_Ls_U_Hold_V = 0.0f;
    Rs_Ls_U_Ac_V = 0.0f;
    Align_Pending = false;

    Frequency_Set(RS_LS_PROBE_FREQ_HZ);
    Ramp_Reset();
    Measure_Reset();
    Align_Reset();

    Rs_Rough = 0.0f;
    Ls_Rough = 0.0f;
    Rs_A = 0.0f;
    Ls_A = 0.0f;
    Rs_B = 0.0f;
    Ls_B = 0.0f;
    Rs_C = 0.0f;
    Ls_C = 0.0f;
    Measure_Third = false;

    Rs_Ls_State = RS_LS_IDLE;
}

void Rs_Ls_Start(void)
{
    Rs_Ls_Reset();
    Rs_Ls_State = RS_LS_PROBE_RAMP;
}

void Rs_Ls_Fail(void)
{
    PI_Restore();
    Align_Pending = false;
    Rs_Ls_Result.Valid = false;
    Rs_Ls_State = RS_LS_FAILED;
}

bool Rs_Ls_Active(void)
{
    return (Rs_Ls_State != RS_LS_IDLE) && (Rs_Ls_State != RS_LS_DONE) && (Rs_Ls_State != RS_LS_FAILED);
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
    float Freq_Target_Hz;
    float I_Target_A;
    float I_Soft_A;
    float I_Abs;
    float U_Ac_Max_V;
    float U_Hold_Abs_V;
    bool Align_Done;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if ((Rs_Ls_State == RS_LS_IDLE) || (Rs_Ls_State == RS_LS_DONE) || (Rs_Ls_State == RS_LS_FAILED))
    {
        return FAST_OFF;
    }

    Envelope = Identification_Envelope_Get();

    if (!Envelope->Valid || (Envelope->U_Hard_V <= 0.0f))
    {
        Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_ENVELOPE);
        return FAST_OFF;
    }

    if (Rs_Ls_State == RS_LS_ALIGN)
    {
        if (!Align_Pending)
        {
            Align_Done = Align_Current(Envelope->I_Align_A, RS_LS_ALIGN_CNT, Id_Ref, Iq_Ref);

            if (Align_Done)
            {
                Align_Pending = true;
            }

            return FAST_CURRENT;
        }

        /* Previous FAST_CURRENT cycle has now produced the final align Ud. */
        Rs_Ls_U_Hold_V = Motor_Run.Ud;
        PI_Restore();
        Rs_Ls_U_Ac_V = 0.0f;
        Rs_Ls_Phase = 0.0f;
        Ramp_Reset();
        Align_Pending = false;
        Rs_Ls_State = RS_LS_RAMP;
    }

    if ((Rs_Ls_State == RS_LS_PROBE_RAMP) || (Rs_Ls_State == RS_LS_PROBE_MEASURE))
    {
        U_Ac_Max_V = Envelope->U_Hard_V;
        I_Soft_A = Envelope->I_Measure_Max_A;
    }
    else
    {
        U_Hold_Abs_V = (Rs_Ls_U_Hold_V >= 0.0f) ? Rs_Ls_U_Hold_V : -Rs_Ls_U_Hold_V;

        if (U_Hold_Abs_V >= Envelope->U_Hard_V)
        {
            Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_ALIGN_VOLTAGE);
            return FAST_OFF;
        }

        U_Ac_Max_V = Envelope->U_Hard_V - U_Hold_Abs_V;
        I_Soft_A = Envelope->I_Align_Max_A;
    }

    if (Rs_Ls_U_Ac_V > U_Ac_Max_V)
    {
        Rs_Ls_U_Ac_V = U_Ac_Max_V;
    }

    I_Abs = (Ialpha_A >= 0.0f) ? Ialpha_A : -Ialpha_A;
    SinCos(Rs_Ls_Phase, &Sin, &Cos);

    if ((Rs_Ls_State == RS_LS_PROBE_RAMP) || (Rs_Ls_State == RS_LS_PROBE_MEASURE))
    {
        *Ualpha_V = Rs_Ls_U_Ac_V * Sin;
    }
    else
    {
        *Ualpha_V = Rs_Ls_U_Hold_V + Rs_Ls_U_Ac_V * Sin;
    }

    if ((Rs_Ls_State == RS_LS_PROBE_RAMP) || (Rs_Ls_State == RS_LS_RAMP))
    {
        Ramp_I_Re += Ialpha_A * Cos;
        Ramp_I_Im -= Ialpha_A * Sin;
        Ramp_Cnt++;

        if (I_Abs < I_Soft_A)
        {
            Voltage_Ramp(U_Ac_Max_V);
        }
        else
        {
            Voltage_Backoff(U_Ac_Max_V);
        }

        if (Ramp_Cnt >= Rs_Ls_Sample_Per_Cycle)
        {
            Ramp_I_Amp = (2.0f / (float)Rs_Ls_Sample_Per_Cycle) *
                         __builtin_sqrtf(Ramp_I_Re * Ramp_I_Re + Ramp_I_Im * Ramp_I_Im);

            Ramp_Reset();
            I_Target_A = (Rs_Ls_State == RS_LS_PROBE_RAMP) ? Envelope->I_Probe_A : Envelope->I_Measure_A;

            if (Ramp_I_Amp > Envelope->I_Measure_Max_A)
            {
                if (Ramp_I_Amp > 0.0f)
                {
                    Rs_Ls_U_Ac_V *= I_Target_A / Ramp_I_Amp;
                }
            }
            else if (Ramp_I_Amp >= I_Target_A)
            {
                if (Rs_Ls_State == RS_LS_PROBE_RAMP)
                {
                    Measure_Reset();
                    Rs_Ls_State = RS_LS_PROBE_MEASURE;
                }
                else
                {
                    Rs_Ls_Cnt = 0U;
                    Rs_Ls_State = RS_LS_SETTLE;
                }
            }
            else if (Rs_Ls_U_Ac_V >= U_Ac_Max_V)
            {
                if (Ramp_I_Amp >= Envelope->I_Min_A)
                {
                    if (Rs_Ls_State == RS_LS_PROBE_RAMP)
                    {
                        Measure_Reset();
                        Rs_Ls_State = RS_LS_PROBE_MEASURE;
                    }
                    else
                    {
                        Rs_Ls_Cnt = 0U;
                        Rs_Ls_State = RS_LS_SETTLE;
                    }
                }
                else
                {
                    Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_CURRENT_LOW);
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }
            }
        }
    }
    else if (Rs_Ls_State == RS_LS_PROBE_MEASURE)
    {
        U_Re += *Ualpha_V * Cos;
        U_Im -= *Ualpha_V * Sin;
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Rs_Ls_Cnt++;

        if (Rs_Ls_Cnt >= Rs_Ls_Probe_Measure_Cnt)
        {
            if (!Measure_Calc(Rs_Ls_Probe_Measure_Cnt, &Rs_Rough, &Ls_Rough))
            {
                Rs_Ls_Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

            Rs_Ls_Result.Rs_Ohm = Rs_Rough;
            Rs_Ls_Result.Ls_H = Ls_Rough;
            Rs_Ls_Result.Valid = false;
            Rs_Ls_Result.Probe_U_Re = U_Re;
            Rs_Ls_Result.Probe_U_Im = U_Im;
            Rs_Ls_Result.Probe_I_Re = I_Re;
            Rs_Ls_Result.Probe_I_Im = I_Im;

            if (!Result_Range_Check(Rs_Rough, Ls_Rough))
            {
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

            Freq_Target_Hz = RS_LS_FREQ_RL_RATIO * Rs_Rough / (TWO_PI_F * Ls_Rough);
            Frequency_Set(Freq_Target_Hz);
            PI_Temporary_Set(Rs_Rough, Ls_Rough);
            Align_Reset();
            Align_Pending = false;
            Rs_Ls_State = RS_LS_ALIGN;
            *Ualpha_V = 0.0f;
            return FAST_VOLTAGE;
        }
    }
    else if (Rs_Ls_State == RS_LS_SETTLE)
    {
        Rs_Ls_Cnt++;

        if (Rs_Ls_Cnt >= RS_LS_SETTLE_CNT)
        {
            Measure_Reset();
            Rs_Ls_State = RS_LS_MEASURE_A;
        }
    }
    else
    {
        U_Re += *Ualpha_V * Cos;
        U_Im -= *Ualpha_V * Sin;
        I_Re += Ialpha_A * Cos;
        I_Im -= Ialpha_A * Sin;
        Rs_Ls_Cnt++;

        if (Rs_Ls_Cnt >= Rs_Ls_Measure_Cnt)
        {
            if (Rs_Ls_State == RS_LS_MEASURE_A)
            {
                if (!Measure_Calc(Rs_Ls_Measure_Cnt, &Rs_A, &Ls_A))
                {
                    Rs_Ls_Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (!Result_Range_Check(Rs_A, Ls_A))
                {
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Measure_Reset();
                Rs_Ls_State = RS_LS_MEASURE_B;
            }
            else if (!Measure_Third)
            {
                if (!Measure_Calc(Rs_Ls_Measure_Cnt, &Rs_B, &Ls_B))
                {
                    Rs_Ls_Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (!Result_Range_Check(Rs_B, Ls_B))
                {
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (Pair_Valid(Rs_A, Ls_A, Rs_B, Ls_B))
                {
                    Rs_Ls_Result.Rs_Ohm = 0.5f * (Rs_A + Rs_B);
                    Rs_Ls_Result.Ls_H = 0.5f * (Ls_A + Ls_B);
                    Rs_Ls_Result.Valid = true;
                    Rs_Ls_State = RS_LS_DONE;
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Measure_Third = true;
                Measure_Reset();
            }
            else
            {
                if (!Measure_Calc(Rs_Ls_Measure_Cnt, &Rs_C, &Ls_C))
                {
                    Rs_Ls_Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (!Result_Range_Check(Rs_C, Ls_C))
                {
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (Best_Pair_Result(&Rs_Ls_Result.Rs_Ohm, &Rs_Ls_Result.Ls_H))
                {
                    Rs_Ls_Result.Valid = true;
                    Rs_Ls_State = RS_LS_DONE;
                }
                else
                {
                    Rs_Ls_Fail_Stage(RS_LS_FAIL_STAGE_REPEATABILITY);
                }

                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }
        }
    }

    Rs_Ls_Phase += Rs_Ls_Phase_Step;

    if (Rs_Ls_Phase >= TWO_PI_F)
    {
        Rs_Ls_Phase -= TWO_PI_F;
    }

    return FAST_VOLTAGE;
}

Rs_Ls_State_e Rs_Ls_State_Get(void)
{
    if (Rs_Ls_State == RS_LS_FAILED)
    {
        return (Rs_Ls_State_e)Fail_Stage;
    }

    return Rs_Ls_State;
}

const Rs_Ls_Result_T *Rs_Ls_Result_Get(void)
{
    return &Rs_Ls_Result;
}
