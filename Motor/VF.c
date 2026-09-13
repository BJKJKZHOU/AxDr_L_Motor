/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "VF.h"

#include <stddef.h>

#include "Math.h"
#include "Sin_LUT.h"

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Sign_F(float X)
{
    return (X < 0.0f) ? -1.0f : 1.0f;
}

static float Slew_F(float Value, float Target, float Step)
{
    if (Value < Target)
    {
        Value += Step;
        if (Value > Target)
        {
            Value = Target;
        }
    }
    else if (Value > Target)
    {
        Value -= Step;
        if (Value < Target)
        {
            Value = Target;
        }
    }

    return Value;
}

void VF_Run(VF_T *VF,
            float We_Target,
            float U_Target_V,
            float *Ualpha_V,
            float *Ubeta_V,
            float Ts)
{
    float Sin;
    float Cos;
    float Uq;
    float We_Step;
    float U_Step;

    if ((VF == NULL) || (Ualpha_V == NULL) || (Ubeta_V == NULL) ||
        (Ts <= 0.0f) || !__builtin_isfinite(Ts))
    {
        return;
    }

    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if ((VF->Para.U_Max_V <= 0.0f) ||
        (VF->Para.U_Slew_V_S <= 0.0f) ||
        (VF->Para.We_Acc <= 0.0f) ||
        !__builtin_isfinite(VF->Para.U_Max_V) ||
        !__builtin_isfinite(VF->Para.U_Slew_V_S) ||
        !__builtin_isfinite(VF->Para.We_Acc) ||
        !__builtin_isfinite(VF->State.Theta_e) ||
        !__builtin_isfinite(VF->State.We) ||
        !__builtin_isfinite(VF->State.U) ||
        !__builtin_isfinite(We_Target) ||
        !__builtin_isfinite(U_Target_V))
    {
        return;
    }

    U_Target_V = Abs_F(U_Target_V);
    if (U_Target_V > VF->Para.U_Max_V)
    {
        U_Target_V = VF->Para.U_Max_V;
    }

    We_Step = VF->Para.We_Acc * Ts;
    U_Step = VF->Para.U_Slew_V_S * Ts;

    VF->State.We = Slew_F(VF->State.We, We_Target, We_Step);
    VF->State.U = Slew_F(VF->State.U, U_Target_V, U_Step);

    Uq = Sign_F(We_Target) * VF->State.U;
    SinCos(VF->State.Theta_e, &Sin, &Cos);
    *Ualpha_V = -Uq * Sin;
    *Ubeta_V = Uq * Cos;

    VF->State.Theta_e = Angle_Wrap(VF->State.Theta_e + VF->State.We * Ts);
}
