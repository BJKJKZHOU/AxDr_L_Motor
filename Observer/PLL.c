/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "PLL.h"

#include "Math.h"
#include "Sin_LUT.h"

void PLL_Reset(PLL_T *Pll, float Theta, float We)
{
    Pll->State.Theta = Angle_Wrap(Theta);
    Pll->State.We = We;
    Pll->State.Err = 0.0f;
}

bool PLL_Run(PLL_T *Pll, float X, float Y, float Ts)
{
    float Sin;
    float Cos;
    float Mag2;
    float Mag;
    float Err;
    float We_Next;
    float Theta_Next;

    if ((Pll == 0) || !__builtin_isfinite(X) || !__builtin_isfinite(Y) ||
        !__builtin_isfinite(Ts) ||
        !__builtin_isfinite(Pll->Para.Kp) || !__builtin_isfinite(Pll->Para.Ki) ||
        !__builtin_isfinite(Pll->State.Theta) || !__builtin_isfinite(Pll->State.We) ||
        (Ts <= 0.0f))
    {
        return false;
    }

    Mag2 = X * X + Y * Y;
    if (!__builtin_isfinite(Mag2) || (Mag2 <= 0.0f))
    {
        return false;
    }

    Mag = __builtin_sqrtf(Mag2);
    if (!__builtin_isfinite(Mag) || (Mag <= 0.0f))
    {
        return false;
    }

    SinCos(Pll->State.Theta, &Sin, &Cos);

    Err = (Y * Cos - X * Sin) / Mag;
    We_Next = Pll->State.We + Pll->Para.Ki * Err * Ts;
    Theta_Next = Pll->State.Theta + (We_Next + Pll->Para.Kp * Err) * Ts;

    if (!__builtin_isfinite(Err) || !__builtin_isfinite(We_Next) || !__builtin_isfinite(Theta_Next))
    {
        return false;
    }

    Theta_Next = Angle_Wrap(Theta_Next);
    if (!__builtin_isfinite(Theta_Next))
    {
        return false;
    }

    Pll->State.Err = Err;
    Pll->State.We = We_Next;
    Pll->State.Theta = Theta_Next;
    return true;
}
