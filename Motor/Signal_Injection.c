/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Signal_Injection.h"

#include "Math.h"
#include "Sin_LUT.h"
#include "control_params.h"

volatile Signal_Injection_T Signal_Injection = {
    .Para = { .Freq_Hz = 100.0f, .Amp_A = 0.1f, .Time_S = 2.0f, .Speed_Amp = 1.0f, .Target = SIGNAL_ID },
};

bool Signal_Injection_Start(float I_Max, float Wm_Max)
{
    uint8_t Target = Signal_Injection.Para.Target;
    float Freq = Signal_Injection.Para.Freq_Hz;
    float Time = Signal_Injection.Para.Time_S;
    float Amp;
    float Rate;
    uint32_t Samples;

    if ((Signal_Injection.State.Active != 0U) ||
        !__builtin_isfinite(Freq) || (Freq <= 0.0f) ||
        !__builtin_isfinite(Time) || (Time < CUR_TS) || (Time > 60.0f))
    {
        return false;
    }

    if (Target == SIGNAL_SPEED)
    {
        Rate = SPD_FREQ_HZ_DEFAULT;
        Amp = Signal_Injection.Para.Speed_Amp;
        /* Accept the requested sweep range up to 1 Hz below Nyquist. */
        if ((Freq > 999.0f) || !__builtin_isfinite(Wm_Max) ||
            !__builtin_isfinite(Amp) || (Amp <= 0.0f) || (Amp > Wm_Max))
        {
            return false;
        }
    }
    else if ((Target == SIGNAL_ID) || (Target == SIGNAL_IQ))
    {
        Rate = CUR_FREQ_HZ_DEFAULT;
        Amp = Signal_Injection.Para.Amp_A;
        if ((Freq > 9999.0f) || !__builtin_isfinite(I_Max) ||
            !__builtin_isfinite(Amp) || (Amp <= 0.0f) || (Amp > I_Max))
        {
            return false;
        }
    }
    else
    {
        return false;
    }

    Samples = (uint32_t)(Time * Rate + 0.5f);
    if (Samples == 0U)
    {
        return false;
    }

    Signal_Injection.State.Phase = 0.0f;
    Signal_Injection.State.Phase_Step = TWO_PI_F * Freq / Rate;
    Signal_Injection.State.Amp = Amp;
    Signal_Injection.State.Out = 0.0f;
    Signal_Injection.State.Sample_Cnt = 0U;
    Signal_Injection.State.Sample_Max = Samples;
    Signal_Injection.State.Target = Target;
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

    if (Signal_Injection.State.Sample_Cnt >= Signal_Injection.State.Sample_Max)
    {
        Signal_Injection_Stop();
        return 0.0f;
    }

    SinCos(Signal_Injection.State.Phase, &Sin, &Cos);
    Out = Signal_Injection.State.Amp * Sin;
    Signal_Injection.State.Out = Out;

    Signal_Injection.State.Phase += Signal_Injection.State.Phase_Step;
    if (Signal_Injection.State.Phase >= TWO_PI_F)
    {
        Signal_Injection.State.Phase -= TWO_PI_F;
    }

    Signal_Injection.State.Sample_Cnt++;
    return Out;
}
