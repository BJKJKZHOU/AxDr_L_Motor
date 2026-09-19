/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Trapezoid.h"

#include "Math.h"
#include "Ramp.h"

static float Abs(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Position_Error(int32_t Turn_Target,
                            float Theta_Target,
                            int32_t Turn_Ref,
                            float Theta_Ref)
{
    int32_t Turn_Err;

    Turn_Err = Turn_Target - Turn_Ref;
    return (float)Turn_Err * TWO_PI_F + Theta_Target - Theta_Ref;
}

static void Position_Add(Motion_Ref_T *Ref, float Delta)
{
    Ref->Theta += Delta;

    while (Ref->Theta >= TWO_PI_F)
    {
        Ref->Theta -= TWO_PI_F;
        Ref->Turn++;
    }

    while (Ref->Theta < 0.0f)
    {
        Ref->Theta += TWO_PI_F;
        Ref->Turn--;
    }
}

void Trapezoid_Reset(Motion_Ref_T *Ref, int32_t Turn, float Theta, float Wm)
{
    Ref->Turn = Turn;
    Ref->Theta = Theta;
    Ref->Wm = Wm;
    Ref->Am = 0.0f;
}

void Trapezoid_Run(Motion_Ref_T *Ref,
                   int32_t Turn_Target,
                   float Theta_Target,
                   float Wm_Max,
                   float Acc,
                   float Dec,
                   float Ts)
{
    float Err;
    float Err_Abs;
    float Dir;
    float Wm_Old;
    float Wm_Target;
    float Wm_Brake;
    float Pos_Tol;
    float Delta;

    if ((Wm_Max < 0.0f) || (Acc <= 0.0f) || (Dec <= 0.0f) || (Ts <= 0.0f))
    {
        Ref->Am = 0.0f;
        return;
    }

    Err = Position_Error(Turn_Target, Theta_Target, Ref->Turn, Ref->Theta);
    Err_Abs = Abs(Err);
    Wm_Old = Ref->Wm;
    Pos_Tol = Dec * Ts * Ts;

    if ((Err_Abs <= Pos_Tol) && (Abs(Wm_Old) <= Dec * Ts))
    {
        Ref->Turn = Turn_Target;
        Ref->Theta = Theta_Target;
        Ref->Wm = 0.0f;
        Ref->Am = -Wm_Old / Ts;
        return;
    }

    Dir = (Err >= 0.0f) ? 1.0f : -1.0f;

    if ((Wm_Max == 0.0f) || (Wm_Old * Dir < 0.0f))
    {
        Wm_Target = 0.0f;
    }
    else
    {
        Wm_Brake = __builtin_sqrtf(2.0f * Dec * Err_Abs);
        Wm_Target = (Wm_Brake < Wm_Max) ? Wm_Brake : Wm_Max;
        Wm_Target *= Dir;
    }

    Ramp_Run(Ref, Wm_Target, Acc, Dec, Ts);
    Delta = 0.5f * (Wm_Old + Ref->Wm) * Ts;
    Position_Add(Ref, Delta);
}

bool Trapezoid_Stop(Motion_Ref_T *Ref, float Dec, float Ts)
{
    float Wm_Old;
    float Delta;

    if ((Dec <= 0.0f) || (Ts <= 0.0f))
    {
        Ref->Am = 0.0f;
        return false;
    }

    Wm_Old = Ref->Wm;
    Ramp_Run(Ref, 0.0f, Dec, Dec, Ts);
    Delta = 0.5f * (Wm_Old + Ref->Wm) * Ts;
    Position_Add(Ref, Delta);

    return (Ref->Wm == 0.0f);
}
