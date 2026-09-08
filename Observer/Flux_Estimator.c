/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flux_Estimator.h"

#include "Math.h"

bool Flux_Estimator_Run(Flux_Estimator_T *Est,
                        Flux_Estimator_Mode_e Mode,
                        float Ud,
                        float Uq,
                        float Id,
                        float Iq,
                        float We,
                        float Ts)
{
    float W_I;
    float Id_Dot;
    float Iq_Dot;
    float Norm;
    float Gain;
    float Err_d;
    float Err_q;
    float Flux_Next;

    if ((Est == 0) || (Ts <= 0.0f) || !__builtin_isfinite(Ud) || !__builtin_isfinite(Uq) ||
        !__builtin_isfinite(Id) || !__builtin_isfinite(Iq) || !__builtin_isfinite(We))
    {
        return false;
    }

    Est->State.Estimate_Valid = false;

    if (!Est->State.I_Valid)
    {
        Est->State.Id_F = Id;
        Est->State.Iq_F = Iq;
        Est->State.I_Valid = true;
        Est->State.Model_Valid = false;
        return false;
    }

    W_I = TWO_PI_F * Est->Para.I_BW_Hz;
    Id_Dot = W_I * (Id - Est->State.Id_F);
    Iq_Dot = W_I * (Iq - Est->State.Iq_F);
    Est->State.Id_F += Id_Dot * Ts;
    Est->State.Iq_F += Iq_Dot * Ts;

    Est->State.Yd = Uq - Est->Para.Rs * Iq - Est->Para.Lq * Iq_Dot - We * Est->Para.Ld * Id;
    Est->State.Yq = -Ud + Est->Para.Rs * Id + Est->Para.Ld * Id_Dot - We * Est->Para.Lq * Iq;
    Est->State.Model_Valid = __builtin_isfinite(Est->State.Yd) && __builtin_isfinite(Est->State.Yq);
    if (!Est->State.Model_Valid)
    {
        return false;
    }

    if (Mode == FLUX_EST_MODEL)
    {
        Est->State.Estimate_Valid = true;
        return true;
    }

    Norm = We / (We * We + Est->Para.We_Min * Est->Para.We_Min);
    Gain = TWO_PI_F * Est->Para.Est_BW_Hz * Ts;

    if (Mode == FLUX_EST_VECTOR)
    {
        Err_d = Est->State.Yd - We * Est->State.Psi_d;
        Err_q = Est->State.Yq - We * Est->State.Psi_q;
        Est->State.Psi_d += Gain * Norm * Err_d;
        Est->State.Psi_q += Gain * Norm * Err_q;
        Est->State.Flux = __builtin_sqrtf(Est->State.Psi_d * Est->State.Psi_d +
                                          Est->State.Psi_q * Est->State.Psi_q);
        Est->State.Estimate_Valid = __builtin_isfinite(Est->State.Flux) && (Est->State.Flux > 0.0f);
        return true;
    }

    if (Mode != FLUX_EST_SCALAR || !__builtin_isfinite(Est->State.Flux) || (Est->State.Flux <= 0.0f))
    {
        return true;
    }

    Flux_Next = Est->State.Flux + Gain * Norm * (Est->State.Yd - We * Est->State.Flux);
    if (!__builtin_isfinite(Flux_Next) || (Flux_Next <= 0.0f))
    {
        return true;
    }

    Est->State.Flux = Flux_Next;
    Est->State.Estimate_Valid = true;
    return true;
}
