/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "PLL.h"

#include "Math.h"
#include "Sin_LUT.h"

bool PLL_Run(PLL_T *Pll, float X, float Y, float Ts)
{
    float Sin;
    float Cos;
    float Mag2;
    float Mag;
    float Err;
    float We_Next;
    float Theta_Next;

    if (!__builtin_isfinite(Pll->State.Theta))
    {
        return false;
    }

    Mag2 = X * X + Y * Y;
    if (!__builtin_isfinite(Mag2) || (Mag2 <= 0.0f))
    {
        return false;
    }

    Mag = __builtin_sqrtf(Mag2);

    SinCos(Pll->State.Theta, &Sin, &Cos);

    Err = (Y * Cos - X * Sin) / Mag;
    We_Next = Pll->State.We + Pll->Para.Ki * Err * Ts;
    Theta_Next = Pll->State.Theta + (We_Next + Pll->Para.Kp * Err) * Ts;

    if (!__builtin_isfinite(Theta_Next))
    {
        return false;
    }

    Theta_Next = Angle_Wrap(Theta_Next);

    Pll->State.Err = Err;
    Pll->State.We = We_Next;
    Pll->State.Theta = Theta_Next;
    return true;
}
