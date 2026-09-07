/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Handover.h"

#include "Math.h"
#include "Sin_LUT.h"

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

float Handover_Angle_Diff(float A, float B)
{
    float Diff;

    Diff = A - B;
    while (Diff > PI_F)
    {
        Diff -= TWO_PI_F;
    }
    while (Diff < -PI_F)
    {
        Diff += TWO_PI_F;
    }
    return Diff;
}

void Handover_Reset(Handover_T *Handover)
{
    if (Handover == 0)
    {
        return;
    }

    Handover_Compare_Reset(Handover);
    Handover->Blend_Cnt = 0U;
}

void Handover_Compare_Reset(Handover_T *Handover)
{
    if (Handover == 0)
    {
        return;
    }

    Handover->We_Err_F = 0.0f;
    Handover->We_Err2_F = 0.0f;
    Handover->PLL_Err2_F = 0.0f;
    Handover->Theta_Err_F = 0.0f;
    Handover->Theta_Err2_F = 0.0f;
    Handover->Speed_Valid = false;
    Handover->Theta_Valid = false;
}

void Handover_Speed_Compare(Handover_T *Handover,
                            float We_Ref,
                            float We_Obs,
                            float PLL_Err,
                            float Alpha)
{
    float We_Err;
    float PLL_Err2;

    if (Handover == 0)
    {
        return;
    }

    We_Err = We_Obs - We_Ref;
    PLL_Err2 = PLL_Err * PLL_Err;

    if (!Handover->Speed_Valid)
    {
        Handover->We_Err_F = We_Err;
        Handover->We_Err2_F = We_Err * We_Err;
        Handover->PLL_Err2_F = PLL_Err2;
        Handover->Speed_Valid = true;
        return;
    }

    Handover->We_Err_F += Alpha * (We_Err - Handover->We_Err_F);
    Handover->We_Err2_F += Alpha * (We_Err * We_Err - Handover->We_Err2_F);
    Handover->PLL_Err2_F += Alpha * (PLL_Err2 - Handover->PLL_Err2_F);
}

void Handover_IF_Compare(Handover_T *Handover,
                         float Theta_IF,
                         float We_IF,
                         float Theta_Obs,
                         float We_Obs,
                         float PLL_Err,
                         float Alpha)
{
    float Theta_Err;
    float Theta_Ripple;

    if (Handover == 0)
    {
        return;
    }

    Handover_Speed_Compare(Handover, We_IF, We_Obs, PLL_Err, Alpha);

    Theta_Err = Handover_Angle_Diff(Theta_Obs, Theta_IF);
    if (!Handover->Theta_Valid)
    {
        Handover->Theta_Err_F = Angle_Wrap(Theta_Err);
        Handover->Theta_Err2_F = 0.0f;
        Handover->Theta_Valid = true;
        return;
    }

    Handover->Theta_Err_F = Angle_Wrap(Handover->Theta_Err_F +
                                       Alpha * Handover_Angle_Diff(Theta_Err, Handover->Theta_Err_F));
    Theta_Ripple = Handover_Angle_Diff(Theta_Err, Handover->Theta_Err_F);
    Handover->Theta_Err2_F += Alpha * (Theta_Ripple * Theta_Ripple - Handover->Theta_Err2_F);
}

bool Handover_Speed_Stable(const Handover_T *Handover,
                           float We_Ref,
                           float We_Min,
                           float We_Mean_Ratio,
                           float We_Rms_Ratio,
                           float PLL_Rms_Max)
{
    float We_Scale;

    if ((Handover == 0) || !Handover->Speed_Valid ||
        !__builtin_isfinite(Handover->We_Err_F) ||
        !__builtin_isfinite(Handover->We_Err2_F) ||
        !__builtin_isfinite(Handover->PLL_Err2_F))
    {
        return false;
    }

    We_Scale = Abs_Value(We_Ref);
    if (We_Scale < We_Min)
    {
        We_Scale = We_Min;
    }

    return (Abs_Value(Handover->We_Err_F) <= We_Mean_Ratio * We_Scale) &&
           (Handover->We_Err2_F <= We_Rms_Ratio * We_Rms_Ratio * We_Scale * We_Scale) &&
           (Handover->PLL_Err2_F <= PLL_Rms_Max * PLL_Rms_Max);
}

bool Handover_IF_Stable(const Handover_T *Handover,
                        float We_Ref,
                        float We_Min,
                        float We_Mean_Ratio,
                        float We_Rms_Ratio,
                        float PLL_Rms_Max,
                        float Theta_Rms_Max)
{
    return Handover_Speed_Stable(Handover,
                                 We_Ref,
                                 We_Min,
                                 We_Mean_Ratio,
                                 We_Rms_Ratio,
                                 PLL_Rms_Max) &&
           Handover->Theta_Valid && __builtin_isfinite(Handover->Theta_Err2_F) &&
           (Handover->Theta_Err2_F <= Theta_Rms_Max * Theta_Rms_Max);
}

void Handover_Qualification_Accumulate(uint32_t *Count, uint32_t Limit, bool Good)
{
    if (Count == 0)
    {
        return;
    }

    if (Good)
    {
        if (*Count < Limit)
        {
            (*Count)++;
        }
    }
    else if (*Count > 0U)
    {
        (*Count)--;
    }
}

void Handover_Blend_Reset(Handover_T *Handover)
{
    if (Handover != 0)
    {
        Handover->Blend_Cnt = 0U;
    }
}

bool Handover_Blend_Run(Handover_T *Handover,
                        uint32_t Blend_Limit,
                        float Theta_IF,
                        float Theta_Obs,
                        float Id_IF,
                        float Iq_IF,
                        float *Theta_Use,
                        float *Id_Ref,
                        float *Iq_Ref)
{
    float Blend;
    float Theta_Err;
    float Diff;
    float Sin;
    float Cos;

    if ((Handover == 0) || (Theta_Use == 0) || (Id_Ref == 0) || (Iq_Ref == 0))
    {
        return false;
    }

    Blend = (Blend_Limit > 0U) ? (float)(Handover->Blend_Cnt + 1U) / (float)Blend_Limit : 1.0f;
    if (Blend > 1.0f)
    {
        Blend = 1.0f;
    }

    Theta_Err = Handover_Angle_Diff(Theta_Obs, Theta_IF);
    *Theta_Use = Angle_Wrap(Theta_IF + Blend * Theta_Err);

    Diff = Handover_Angle_Diff(Theta_IF, *Theta_Use);
    SinCos(Angle_Wrap(Diff), &Sin, &Cos);
    *Id_Ref = Id_IF * Cos - Iq_IF * Sin;
    *Iq_Ref = Id_IF * Sin + Iq_IF * Cos;

    if (Handover->Blend_Cnt < Blend_Limit)
    {
        Handover->Blend_Cnt++;
    }

    return Handover->Blend_Cnt >= Blend_Limit;
}

float Handover_Ramp_Zero(float Value, float Step)
{
    if (Value > Step)
    {
        return Value - Step;
    }
    if (Value < -Step)
    {
        return Value + Step;
    }
    return 0.0f;
}
