/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux_Observer.h"

#include "Math.h"
#include "Sin_LUT.h"

static void Flux_Observer_Calc_Update(Flux_Observer_T *Obs)
{
    float Flux2;
    float Gamma;

    if ((Obs->Para.Flux == Obs->Calc.Flux_Used) && (Obs->Para.BW_Hz == Obs->Calc.BW_Used))
    {
        return;
    }

    Obs->Calc.Flux_Used = Obs->Para.Flux;
    Obs->Calc.BW_Used = Obs->Para.BW_Hz;
    Obs->Calc.Gamma = 0.0f;

    if (!__builtin_isfinite(Obs->Para.Flux) || !__builtin_isfinite(Obs->Para.BW_Hz) ||
        (Obs->Para.Flux <= 0.0f) || (Obs->Para.BW_Hz <= 0.0f))
    {
        return;
    }

    Flux2 = Obs->Para.Flux * Obs->Para.Flux;
    Gamma = TWO_PI_F * Obs->Para.BW_Hz / Flux2;
    if (__builtin_isfinite(Gamma))
    {
        Obs->Calc.Gamma = Gamma;
    }
}

void Flux_Observer_Reset(Flux_Observer_T *Obs, float Theta_e, float Ialpha, float Ibeta)
{
    float Sin;
    float Cos;

    Flux_Observer_Calc_Update(Obs);
    SinCos(Angle_Wrap(Theta_e), &Sin, &Cos);

    Obs->State.PsiAlpha = Obs->Para.Flux * Cos;
    Obs->State.PsiBeta = Obs->Para.Flux * Sin;

    Obs->State.LambdaAlpha = Obs->Para.Ls * Ialpha + Obs->State.PsiAlpha;
    Obs->State.LambdaBeta = Obs->Para.Ls * Ibeta + Obs->State.PsiBeta;

    Obs->State.Flux_Err = 0.0f;
}

void Flux_Observer_Run(Flux_Observer_T *Obs, float Ualpha, float Ubeta, float Ialpha, float Ibeta, float Ts)
{
    float Corr;
    float LambdaAlpha_Dot;
    float LambdaBeta_Dot;

    Flux_Observer_Calc_Update(Obs);

    Obs->State.PsiAlpha = Obs->State.LambdaAlpha - Obs->Para.Ls * Ialpha;
    Obs->State.PsiBeta = Obs->State.LambdaBeta - Obs->Para.Ls * Ibeta;

    Obs->State.Flux_Err = Obs->Para.Flux * Obs->Para.Flux - Obs->State.PsiAlpha * Obs->State.PsiAlpha -
                          Obs->State.PsiBeta * Obs->State.PsiBeta;

    Corr = 0.5f * Obs->Calc.Gamma * Obs->State.Flux_Err;

    LambdaAlpha_Dot = Ualpha - Obs->Para.Rs * Ialpha + Corr * Obs->State.PsiAlpha;
    LambdaBeta_Dot = Ubeta - Obs->Para.Rs * Ibeta + Corr * Obs->State.PsiBeta;

    Obs->State.LambdaAlpha += LambdaAlpha_Dot * Ts;
    Obs->State.LambdaBeta += LambdaBeta_Dot * Ts;

    Obs->State.PsiAlpha = Obs->State.LambdaAlpha - Obs->Para.Ls * Ialpha;
    Obs->State.PsiBeta = Obs->State.LambdaBeta - Obs->Para.Ls * Ibeta;
}
