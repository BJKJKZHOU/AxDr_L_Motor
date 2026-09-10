/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux_Observer.h"

#include "Math.h"
#include "Sin_LUT.h"

static bool Flux_Observer_Calc_Update(Flux_Observer_T *Obs)
{
    float Wc;

    if (Obs->Para.BW_Hz == Obs->Calc.BW_Used)
    {
        return __builtin_isfinite(Obs->Calc.Wc) && (Obs->Calc.Wc > 0.0f);
    }

    Obs->Calc.BW_Used = Obs->Para.BW_Hz;
    Obs->Calc.Wc = 0.0f;

    if (!__builtin_isfinite(Obs->Para.BW_Hz) || (Obs->Para.BW_Hz <= 0.0f))
    {
        return false;
    }

    Wc = TWO_PI_F * Obs->Para.BW_Hz;
    if (!__builtin_isfinite(Wc) || (Wc <= 0.0f))
    {
        return false;
    }

    Obs->Calc.Wc = Wc;
    return true;
}

void Flux_Observer_Reset(Flux_Observer_T *Obs, float Theta_e, float Ialpha, float Ibeta)
{
    float Sin;
    float Cos;

    (void)Flux_Observer_Calc_Update(Obs);
    SinCos(Angle_Wrap(Theta_e), &Sin, &Cos);

    Obs->State.PsiAlpha = Obs->Para.Flux * Cos;
    Obs->State.PsiBeta = Obs->Para.Flux * Sin;

    Obs->State.LambdaAlpha = Obs->Para.Ls * Ialpha + Obs->State.PsiAlpha;
    Obs->State.LambdaBeta = Obs->Para.Ls * Ibeta + Obs->State.PsiBeta;

    Obs->State.Flux_Err = 0.0f;
}

bool Flux_Observer_Run(Flux_Observer_T *Obs,
                       float Ualpha,
                       float Ubeta,
                       float Ialpha,
                       float Ibeta,
                       float Ts)
{
    float PsiAlpha;
    float PsiBeta;
    float Flux2;
    float Psi2;
    float Den;
    float Flux_Err;
    float Corr;
    float LambdaAlpha_Dot;
    float LambdaBeta_Dot;
    float LambdaAlpha_Next;
    float LambdaBeta_Next;
    float PsiAlpha_Next;
    float PsiBeta_Next;

    if ((Obs == 0) || !__builtin_isfinite(Ualpha) || !__builtin_isfinite(Ubeta) ||
        !__builtin_isfinite(Ialpha) || !__builtin_isfinite(Ibeta) ||
        !__builtin_isfinite(Ts) || (Ts <= 0.0f) ||
        !__builtin_isfinite(Obs->Para.Rs) || !__builtin_isfinite(Obs->Para.Ls) ||
        !__builtin_isfinite(Obs->Para.Flux) || (Obs->Para.Flux <= 0.0f) ||
        !Flux_Observer_Calc_Update(Obs))
    {
        return false;
    }

    PsiAlpha = Obs->State.LambdaAlpha - Obs->Para.Ls * Ialpha;
    PsiBeta = Obs->State.LambdaBeta - Obs->Para.Ls * Ibeta;
    Flux2 = Obs->Para.Flux * Obs->Para.Flux;
    Psi2 = PsiAlpha * PsiAlpha + PsiBeta * PsiBeta;
    Den = Flux2 + Psi2;

    if (!__builtin_isfinite(PsiAlpha) || !__builtin_isfinite(PsiBeta) ||
        !__builtin_isfinite(Flux2) || !__builtin_isfinite(Psi2) ||
        !__builtin_isfinite(Den) || (Den <= 0.0f))
    {
        return false;
    }

    Flux_Err = Flux2 - Psi2;

    /*
     * Use a normalized nonlinear flux-magnitude correction:
     *
     *   Corr = Wc * (Flux^2 - |Psi|^2) / (Flux^2 + |Psi|^2)
     *
     * Unlike the previous Gamma = Wc / Flux^2 form, the correction gain is
     * bounded by |Corr| <= Wc. This prevents a temporarily small positive Flux
     * estimate from creating an extremely large correction and driving the
     * observer state to Inf/NaN. Around |Psi| = Flux the first-order radial
     * convergence remains approximately Wc, so BW_Hz keeps its local meaning.
     */
    Corr = Obs->Calc.Wc * Flux_Err / Den;

    LambdaAlpha_Dot = Ualpha - Obs->Para.Rs * Ialpha + Corr * PsiAlpha;
    LambdaBeta_Dot = Ubeta - Obs->Para.Rs * Ibeta + Corr * PsiBeta;

    LambdaAlpha_Next = Obs->State.LambdaAlpha + LambdaAlpha_Dot * Ts;
    LambdaBeta_Next = Obs->State.LambdaBeta + LambdaBeta_Dot * Ts;
    PsiAlpha_Next = LambdaAlpha_Next - Obs->Para.Ls * Ialpha;
    PsiBeta_Next = LambdaBeta_Next - Obs->Para.Ls * Ibeta;

    if (!__builtin_isfinite(Flux_Err) || !__builtin_isfinite(Corr) ||
        !__builtin_isfinite(LambdaAlpha_Dot) || !__builtin_isfinite(LambdaBeta_Dot) ||
        !__builtin_isfinite(LambdaAlpha_Next) || !__builtin_isfinite(LambdaBeta_Next) ||
        !__builtin_isfinite(PsiAlpha_Next) || !__builtin_isfinite(PsiBeta_Next))
    {
        return false;
    }

    Obs->State.LambdaAlpha = LambdaAlpha_Next;
    Obs->State.LambdaBeta = LambdaBeta_Next;
    Obs->State.PsiAlpha = PsiAlpha_Next;
    Obs->State.PsiBeta = PsiBeta_Next;
    Obs->State.Flux_Err = Flux_Err;
    return true;
}
