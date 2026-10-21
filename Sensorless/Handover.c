/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Handover.h"

#include "Math.h"
#include "Sin_LUT.h"

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

void Handover_Blend_Reset(Handover_T *Handover)
{
    Handover->Blend_Cnt = 0U;
}

bool Handover_Blend_Run(Handover_T *Handover,
                        uint32_t Blend_Limit,
                        float Theta_Source,
                        float Theta_Obs,
                        float Id_Source,
                        float Iq_Source,
                        float *Theta_Use,
                        float *Id_Ref,
                        float *Iq_Ref)
{
    float Blend;
    float Theta_Err;
    float Diff;
    float Sin;
    float Cos;

    Blend = (Blend_Limit > 0U) ? (float)(Handover->Blend_Cnt + 1U) / (float)Blend_Limit : 1.0f;
    if (Blend > 1.0f)
    {
        Blend = 1.0f;
    }

    Theta_Err = Handover_Angle_Diff(Theta_Obs, Theta_Source);
    *Theta_Use = Angle_Wrap(Theta_Source + Blend * Theta_Err);

    Diff = Handover_Angle_Diff(Theta_Source, *Theta_Use);
    SinCos(Angle_Wrap(Diff), &Sin, &Cos);
    *Id_Ref = Id_Source * Cos - Iq_Source * Sin;
    *Iq_Ref = Id_Source * Sin + Iq_Source * Cos;

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
