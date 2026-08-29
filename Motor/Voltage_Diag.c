/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Voltage_Diag.h"

#include "Math.h"
#include "Sin_LUT.h"
#include "control_params.h"

#define VOLT_DIAG_WE_SLEW_RAD_S2  240.0f
#define VOLT_DIAG_U_SLEW_V_S      8.0f
#define VOLT_DIAG_ALIGN_S          0.25f
#define VOLT_DIAG_ALIGN_CNT        ((uint32_t)(VOLT_DIAG_ALIGN_S / CUR_TS + 0.5f))
#define VOLT_DIAG_WE_MAX_RAD_S     400.0f
#define VOLT_DIAG_U_MAX_V          6.0f

static bool Enabled = false;
static bool Active = false;
static float We_Target = 0.0f;
static float U_Target = 0.0f;
static float U_Align = 0.5f;
static float We_Ref = 0.0f;
static float U_Ref = 0.0f;
static float Theta = 0.0f;
static uint32_t Align_Cnt = 0U;

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Slew(float Current, float Target, float Step)
{
    float Delta = Target - Current;
    if (Delta > Step) return Current + Step;
    if (Delta < -Step) return Current - Step;
    return Target;
}

bool Voltage_Diag_Config(float We_Target_Set, float U_Target_Set, float U_Align_Set)
{
    if (Active || !__builtin_isfinite(We_Target_Set) || !__builtin_isfinite(U_Target_Set) ||
        !__builtin_isfinite(U_Align_Set) || (Abs_F(We_Target_Set) > VOLT_DIAG_WE_MAX_RAD_S) ||
        (U_Target_Set <= 0.0f) || (U_Target_Set > VOLT_DIAG_U_MAX_V) ||
        (U_Align_Set <= 0.0f) || (U_Align_Set > U_Target_Set))
    {
        return false;
    }

    We_Target = We_Target_Set;
    U_Target = U_Target_Set;
    U_Align = U_Align_Set;
    return true;
}

bool Voltage_Diag_Enable(bool Enable)
{
    if (Active) return false;
    Enabled = Enable;
    return true;
}

bool Voltage_Diag_Enabled(void)
{
    return Enabled;
}

void Voltage_Diag_Begin(void)
{
    We_Ref = 0.0f;
    U_Ref = U_Align;
    Theta = 0.0f;
    Align_Cnt = 0U;
    Active = Enabled;
}

void Voltage_Diag_Stop(void)
{
    Active = false;
    We_Ref = 0.0f;
    U_Ref = 0.0f;
    Theta = 0.0f;
    Align_Cnt = 0U;
}

bool Voltage_Diag_Run(float Ia, float Ib, float Ic, float I_Max,
                      float *Theta_Out, float *Ualpha, float *Ubeta)
{
    float Sin;
    float Cos;

    if (!Active || (I_Max <= 0.0f) || (Abs_F(Ia) > I_Max) ||
        (Abs_F(Ib) > I_Max) || (Abs_F(Ic) > I_Max))
    {
        Voltage_Diag_Stop();
        *Theta_Out = Theta;
        *Ualpha = 0.0f;
        *Ubeta = 0.0f;
        return false;
    }

    if (Align_Cnt < VOLT_DIAG_ALIGN_CNT)
    {
        Align_Cnt++;
        We_Ref = 0.0f;
        U_Ref = U_Align;
        Theta = 0.0f;
    }
    else
    {
        We_Ref = Slew(We_Ref, We_Target, VOLT_DIAG_WE_SLEW_RAD_S2 * CUR_TS);
        U_Ref = Slew(U_Ref, U_Target, VOLT_DIAG_U_SLEW_V_S * CUR_TS);
        Theta = Angle_Wrap(Theta + We_Ref * CUR_TS);
    }

    SinCos(Theta, &Sin, &Cos);
    *Theta_Out = Theta;
    *Ualpha = U_Ref * Cos;
    *Ubeta = U_Ref * Sin;
    return true;
}

void Voltage_Diag_Get(float *We_Target_Out, float *U_Target_Out, float *U_Align_Out,
                      float *We_Ref_Out, float *U_Ref_Out)
{
    *We_Target_Out = We_Target;
    *U_Target_Out = U_Target;
    *U_Align_Out = U_Align;
    *We_Ref_Out = We_Ref;
    *U_Ref_Out = U_Ref;
}
