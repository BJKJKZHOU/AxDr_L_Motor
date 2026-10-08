/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Signal_Injection.h"

#include "Math.h"
#include "Sin_LUT.h"
#include "control_params.h"

volatile Signal_Injection_T Signal_Injection = {
    .Para = { .Freq_Hz = 100.0f, .Amp_A = 0.1f, .Time_S = 2.0f },
};

bool Signal_Injection_Start(float I_Max)
{
    float Freq = Signal_Injection.Para.Freq_Hz;
    float Amp = Signal_Injection.Para.Amp_A;
    float Time = Signal_Injection.Para.Time_S;
    uint32_t Samples;

    if ((Signal_Injection.State.Active != 0U) ||
        !__builtin_isfinite(Freq) || (Freq <= 0.0f) ||
        (Freq >= CUR_FREQ_HZ_DEFAULT * 0.5f) ||
        !__builtin_isfinite(Amp) || (Amp <= 0.0f) ||
        !__builtin_isfinite(I_Max) || (Amp > I_Max) ||
        !__builtin_isfinite(Time) || (Time < CUR_TS) || (Time > 60.0f))
    {
        return false;
    }

    Samples = (uint32_t)(Time * CUR_FREQ_HZ_DEFAULT + 0.5f);
    if (Samples == 0U)
    {
        return false;
    }

    /* Write effective parameters before publishing Active to the 20 kHz ISR. */
    Signal_Injection.State.Phase = 0.0f;
    Signal_Injection.State.Phase_Step = TWO_PI_F * Freq * CUR_TS;
    Signal_Injection.State.Amp_A = Amp;
    Signal_Injection.State.Out = 0.0f;
    Signal_Injection.State.Sample_Cnt = 0U;
    Signal_Injection.State.Sample_Max = Samples;
    Signal_Injection.State.Active = 1U;
    return true;
}

void Signal_Injection_Stop(void)
{
    Signal_Injection.State.Active = 0U;
    Signal_Injection.State.Out = 0.0f;
}

float Signal_Injection_Run(void)
{
    float Sin;
    float Cos;
    float Out;

    if (Signal_Injection.State.Active == 0U)
    {
        return 0.0f;
    }

    SinCos(Signal_Injection.State.Phase, &Sin, &Cos);
    Out = Signal_Injection.State.Amp_A * Sin;
    Signal_Injection.State.Out = Out;

    Signal_Injection.State.Phase += Signal_Injection.State.Phase_Step;
    if (Signal_Injection.State.Phase >= TWO_PI_F)
    {
        Signal_Injection.State.Phase -= TWO_PI_F;
    }

    Signal_Injection.State.Sample_Cnt++;
    if (Signal_Injection.State.Sample_Cnt >= Signal_Injection.State.Sample_Max)
    {
        /* This last sample remains visible to Plot; next cycle is zero. */
        Signal_Injection.State.Active = 0U;
    }

    return Out;
}
