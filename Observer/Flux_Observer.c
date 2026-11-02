/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux_Observer.h"

#include "Math.h"
#include "Sin_LUT.h"

FAST_CODE void Flux_Observer_Calc_Update(const Flux_Observer_Para_T *Para, Flux_Observer_Calc_T *Calc)
{
    float Flux2;
    float Gamma;

    if ((Para->Flux == Calc->Flux_Used) && (Para->BW_Hz == Calc->BW_Used))
    {
        return;
    }

    Calc->Flux_Used = Para->Flux;
    Calc->BW_Used = Para->BW_Hz;
    Calc->Gamma = 0.0f;

    if (!__builtin_isfinite(Para->Flux) || !__builtin_isfinite(Para->BW_Hz) ||
        (Para->Flux <= 0.0f) || (Para->BW_Hz <= 0.0f))
    {
        return;
    }

    Flux2 = Para->Flux * Para->Flux;
    Gamma = TWO_PI_F * Para->BW_Hz / Flux2;
    if (!__builtin_isfinite(Gamma) || (Gamma <= 0.0f))
    {
        return;
    }

    Calc->Gamma = Gamma;
}

void Flux_Observer_Reset(Flux_Observer_T *Obs, float Theta_e, float Ialpha, float Ibeta)
{
    float Sin;
    float Cos;
    float Id;
    float Flux;

    Flux_Observer_Calc_Update(&Obs->Para, &Obs->Calc);
    SinCos(Angle_Wrap(Theta_e), &Sin, &Cos);

    Id = Ialpha * Cos + Ibeta * Sin;
    Flux = Obs->Para.Flux + (Obs->Para.Ld - Obs->Para.Lq) * Id;
    Obs->State.PsiAlpha = Flux * Cos;
    Obs->State.PsiBeta = Flux * Sin;

    Obs->State.LambdaAlpha = Obs->Para.Lq * Ialpha + Obs->State.PsiAlpha;
    Obs->State.LambdaBeta = Obs->Para.Lq * Ibeta + Obs->State.PsiBeta;

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
    float Psi2;
    float Id;
    float Flux;
    float Flux_Err;
    float Corr;
    float LambdaAlpha_Dot;
    float LambdaBeta_Dot;
    float LambdaAlpha_Next;
    float LambdaBeta_Next;
    float PsiAlpha_Next;
    float PsiBeta_Next;

    if (Obs->Calc.Gamma == 0.0f)
    {
        return false;
    }

    PsiAlpha = Obs->State.LambdaAlpha - Obs->Para.Lq * Ialpha;
    PsiBeta = Obs->State.LambdaBeta - Obs->Para.Lq * Ibeta;
    Psi2 = PsiAlpha * PsiAlpha + PsiBeta * PsiBeta;
    if (Psi2 <= 0.0f)
    {
        return false;
    }
    /* Project onto active flux; no PLL angle is needed by the model. */
    Id = (Ialpha * PsiAlpha + Ibeta * PsiBeta) / __builtin_sqrtf(Psi2);
    Flux = Obs->Para.Flux + (Obs->Para.Ld - Obs->Para.Lq) * Id;
    if (Flux <= 0.0f)
    {
        return false;
    }
    Flux_Err = Flux * Flux - Psi2;
    Corr = 0.5f * Obs->Calc.Gamma * Flux_Err;

    LambdaAlpha_Dot = Ualpha - Obs->Para.Rs * Ialpha + Corr * PsiAlpha;
    LambdaBeta_Dot = Ubeta - Obs->Para.Rs * Ibeta + Corr * PsiBeta;

    LambdaAlpha_Next = Obs->State.LambdaAlpha + LambdaAlpha_Dot * Ts;
    LambdaBeta_Next = Obs->State.LambdaBeta + LambdaBeta_Dot * Ts;
    PsiAlpha_Next = LambdaAlpha_Next - Obs->Para.Lq * Ialpha;
    PsiBeta_Next = LambdaBeta_Next - Obs->Para.Lq * Ibeta;

    if (!__builtin_isfinite(Flux_Err) ||
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
