/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Handover.h"

#include "Math.h"
#include "Sin_LUT.h"

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

    Blend = (float)(Handover->Blend_Cnt + 1U) / (float)Blend_Limit;

    Theta_Err = Angle_Diff(Theta_Obs, Theta_Source);
    *Theta_Use = Angle_Wrap(Theta_Source + Blend * Theta_Err);

    Diff = Angle_Diff(Theta_Source, *Theta_Use);
    SinCos(Angle_Wrap(Diff), &Sin, &Cos);
    *Id_Ref = Id_Source * Cos - Iq_Source * Sin;
    *Iq_Ref = Id_Source * Sin + Iq_Source * Cos;

    return ++Handover->Blend_Cnt >= Blend_Limit;
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
