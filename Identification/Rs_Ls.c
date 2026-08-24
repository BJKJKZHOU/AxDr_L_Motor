/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Rs_Ls.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Math.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define RS_LS_FREQ_HZ    100.0f
#define RS_LS_PHASE_STEP (TWO_PI_F * RS_LS_FREQ_HZ * CUR_TS)

#define RS_LS_PROBE_I_TARGET_A    0.2f
#define RS_LS_PROBE_U_MAX_V       0.5f
#define RS_LS_PROBE_U_STEP_V      0.01f
#define RS_LS_PROBE_MEASURE_CYCLE 5U

#define RS_LS_ALIGN_I_TARGET_A 1.0f
#define RS_LS_ALIGN_BW_HZ      200.0f
#define RS_LS_ALIGN_TIME_S     0.5f
#define RS_LS_ALIGN_CNT        ((uint32_t)(RS_LS_ALIGN_TIME_S / CUR_TS + 0.5f))

#define RS_LS_AC_I_TARGET_A 0.4f
#define RS_LS_AC_U_MAX_V    1.0f
#define RS_LS_AC_U_STEP_V   0.005f

#define RS_LS_SETTLE_TIME_S 0.2f
#define RS_LS_SETTLE_CNT    ((uint32_t)(RS_LS_SETTLE_TIME_S / CUR_TS + 0.5f))

#define RS_LS_SAMPLE_PER_CYCLE  ((uint32_t)(CUR_FREQ_HZ_DEFAULT / RS_LS_FREQ_HZ + 0.5f))
#define RS_LS_PROBE_MEASURE_CNT (RS_LS_SAMPLE_PER_CYCLE * RS_LS_PROBE_MEASURE_CYCLE)
#define RS_LS_MEASURE_CYCLE     10U
#define RS_LS_MEASURE_CNT       (RS_LS_SAMPLE_PER_CYCLE * RS_LS_MEASURE_CYCLE)

#define RS_LS_I_AC_MIN_A    0.05f
#define RS_LS_RS_MIN_OHM    0.0001f
#define RS_LS_RS_MAX_OHM    20.0f
#define RS_LS_LS_MIN_H      1.0e-7f
#define RS_LS_LS_MAX_H      0.1f
#define RS_LS_RS_REPEAT_MAX 0.10f
#define RS_LS_LS_REPEAT_MAX 0.15f

static Rs_Ls_State_e Rs_Ls_State = RS_LS_IDLE;
static Rs_Ls_Result_T Rs_Ls_Result = { 0 };

static uint32_t Rs_Ls_Cnt = 0U;
static float Rs_Ls_Phase = 0.0f;
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

static bool PI_Saved = false;
static bool Align_Pending = false;
static float Id_Kp_Save = 0.0f;
static float Id_Ki_Save = 0.0f;
static float Iq_Kp_Save = 0.0f;
static float Iq_Ki_Save = 0.0f;

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

    Wc = TWO_PI_F * RS_LS_ALIGN_BW_HZ;

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
    float Den;
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

    if (I_Amp < RS_LS_I_AC_MIN_A)
    {
        return false;
    }

    Z_Re = (U_Re * I_Re + U_Im * I_Im) / Den;
    Z_Im = (U_Im * I_Re - U_Re * I_Im) / Den;

    *Rs_Ohm = Z_Re;
    *Ls_H = Z_Im / (TWO_PI_F * RS_LS_FREQ_HZ);

    return true;
}

static bool Result_Valid(float Rs_Ohm, float Ls_H)
{
    return (Rs_Ohm > RS_LS_RS_MIN_OHM) && (Rs_Ohm < RS_LS_RS_MAX_OHM) && (Ls_H > RS_LS_LS_MIN_H) &&
           (Ls_H < RS_LS_LS_MAX_H);
}

static bool Repeat_Valid(float A, float B, float Max_Ratio)
{
    float Ref;
    float Diff;

    Ref = 0.5f * (A + B);
    Diff = A - B;

    if (Diff < 0.0f)
    {
        Diff = -Diff;
    }

    return (Ref > 0.0f) && ((Diff / Ref) <= Max_Ratio);
}

void Rs_Ls_Reset(void)
{
    PI_Restore();

    Rs_Ls_Result.Rs_Ohm = 0.0f;
    Rs_Ls_Result.Ls_H = 0.0f;
    Rs_Ls_Result.Valid = false;

    Rs_Ls_Cnt = 0U;
    Rs_Ls_Phase = 0.0f;
    Rs_Ls_U_Hold_V = 0.0f;
    Rs_Ls_U_Ac_V = 0.0f;
    Align_Pending = false;

    Ramp_Reset();
    Measure_Reset();
    Align_Reset();

    Rs_Rough = 0.0f;
    Ls_Rough = 0.0f;
    Rs_A = 0.0f;
    Ls_A = 0.0f;
    Rs_B = 0.0f;
    Ls_B = 0.0f;

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
    float Sin;
    float Cos;
    float Ramp_I_Amp;
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

    if (Rs_Ls_State == RS_LS_ALIGN)
    {
        if (!Align_Pending)
        {
            Align_Done = Align_Current(RS_LS_ALIGN_I_TARGET_A, RS_LS_ALIGN_CNT, Id_Ref, Iq_Ref);

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
        Ramp_Reset();
        Align_Pending = false;
        Rs_Ls_State = RS_LS_RAMP;
    }

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

        if (Ramp_Cnt >= RS_LS_SAMPLE_PER_CYCLE)
        {
            Ramp_I_Amp =
                (2.0f / (float)RS_LS_SAMPLE_PER_CYCLE) * __builtin_sqrtf(Ramp_I_Re * Ramp_I_Re + Ramp_I_Im * Ramp_I_Im);

            Ramp_Reset();

            if (Rs_Ls_State == RS_LS_PROBE_RAMP)
            {
                if (Ramp_I_Amp >= RS_LS_PROBE_I_TARGET_A)
                {
                    Measure_Reset();
                    Rs_Ls_State = RS_LS_PROBE_MEASURE;
                }
                else if (Rs_Ls_U_Ac_V >= RS_LS_PROBE_U_MAX_V)
                {
                    if (Ramp_I_Amp >= RS_LS_I_AC_MIN_A)
                    {
                        Measure_Reset();
                        Rs_Ls_State = RS_LS_PROBE_MEASURE;
                    }
                    else
                    {
                        Rs_Ls_Fail();
                        *Ualpha_V = 0.0f;
                        return FAST_OFF;
                    }
                }
                else
                {
                    Rs_Ls_U_Ac_V += RS_LS_PROBE_U_STEP_V;

                    if (Rs_Ls_U_Ac_V > RS_LS_PROBE_U_MAX_V)
                    {
                        Rs_Ls_U_Ac_V = RS_LS_PROBE_U_MAX_V;
                    }
                }
            }
            else
            {
                if (Ramp_I_Amp >= RS_LS_AC_I_TARGET_A)
                {
                    Rs_Ls_Cnt = 0U;
                    Rs_Ls_State = RS_LS_SETTLE;
                }
                else if (Rs_Ls_U_Ac_V >= RS_LS_AC_U_MAX_V)
                {
                    if (Ramp_I_Amp >= RS_LS_I_AC_MIN_A)
                    {
                        Rs_Ls_Cnt = 0U;
                        Rs_Ls_State = RS_LS_SETTLE;
                    }
                    else
                    {
                        Rs_Ls_Fail();
                        *Ualpha_V = 0.0f;
                        return FAST_OFF;
                    }
                }
                else
                {
                    Rs_Ls_U_Ac_V += RS_LS_AC_U_STEP_V;

                    if (Rs_Ls_U_Ac_V > RS_LS_AC_U_MAX_V)
                    {
                        Rs_Ls_U_Ac_V = RS_LS_AC_U_MAX_V;
                    }
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

        if (Rs_Ls_Cnt >= RS_LS_PROBE_MEASURE_CNT)
        {
            if (!Measure_Calc(RS_LS_PROBE_MEASURE_CNT, &Rs_Rough, &Ls_Rough) || !Result_Valid(Rs_Rough, Ls_Rough))
            {
                Rs_Ls_Fail();
                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }

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

        if (Rs_Ls_Cnt >= RS_LS_MEASURE_CNT)
        {
            if (Rs_Ls_State == RS_LS_MEASURE_A)
            {
                if (!Measure_Calc(RS_LS_MEASURE_CNT, &Rs_A, &Ls_A))
                {
                    Rs_Ls_Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                Measure_Reset();
                Rs_Ls_State = RS_LS_MEASURE_B;
            }
            else
            {
                if (!Measure_Calc(RS_LS_MEASURE_CNT, &Rs_B, &Ls_B))
                {
                    Rs_Ls_Fail();
                    *Ualpha_V = 0.0f;
                    return FAST_OFF;
                }

                if (Result_Valid(Rs_A, Ls_A) && Result_Valid(Rs_B, Ls_B) &&
                    Repeat_Valid(Rs_A, Rs_B, RS_LS_RS_REPEAT_MAX) && Repeat_Valid(Ls_A, Ls_B, RS_LS_LS_REPEAT_MAX))
                {
                    Rs_Ls_Result.Rs_Ohm = 0.5f * (Rs_A + Rs_B);
                    Rs_Ls_Result.Ls_H = 0.5f * (Ls_A + Ls_B);
                    Rs_Ls_Result.Valid = true;
                    Rs_Ls_State = RS_LS_DONE;
                }
                else
                {
                    Rs_Ls_Fail();
                }

                *Ualpha_V = 0.0f;
                return FAST_OFF;
            }
        }
    }

    Rs_Ls_Phase += RS_LS_PHASE_STEP;

    if (Rs_Ls_Phase >= TWO_PI_F)
    {
        Rs_Ls_Phase -= TWO_PI_F;
    }

    return FAST_VOLTAGE;
}

Rs_Ls_State_e Rs_Ls_State_Get(void)
{
    return Rs_Ls_State;
}

const Rs_Ls_Result_T *Rs_Ls_Result_Get(void)
{
    return &Rs_Ls_Result;
}
