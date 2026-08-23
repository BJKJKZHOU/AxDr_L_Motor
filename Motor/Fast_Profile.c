/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Fast_Profile.h"

#include <stdint.h>

volatile Fast_Profile_T Fast_Profile = { 0 };

static void Fast_Profile_Stat_Reset(volatile Fast_Profile_Stat_T *Stat)
{
    Stat->Cnt = 0U;
    Stat->Sum = 0U;
    Stat->Min = UINT32_MAX;
    Stat->Max = 0U;
}

static void Fast_Profile_Reset(void)
{
    Fast_Profile.Sample_Cnt = 0U;

    Fast_Profile_Stat_Reset(&Fast_Profile.ADC_Sample);
    Fast_Profile_Stat_Reset(&Fast_Profile.Motor_Fast);
    Fast_Profile_Stat_Reset(&Fast_Profile.Flux_Observer);
    Fast_Profile_Stat_Reset(&Fast_Profile.PLL);
    Fast_Profile_Stat_Reset(&Fast_Profile.IF_Start);
    Fast_Profile_Stat_Reset(&Fast_Profile.Current_Loop);
    Fast_Profile_Stat_Reset(&Fast_Profile.SVPWM);
    Fast_Profile_Stat_Reset(&Fast_Profile.PWM_Update);
    Fast_Profile_Stat_Reset(&Fast_Profile.Plot_Fast);
    Fast_Profile_Stat_Reset(&Fast_Profile.ADC_Run);
}

void Fast_Profile_Request(void)
{
    if (Fast_Profile.Run == 0U)
    {
        Fast_Profile.Request = 1U;
    }
}

void Fast_Profile_Begin_Cycle(void)
{
    if ((Fast_Profile.Request == 0U) || (Fast_Profile.Run != 0U))
    {
        return;
    }

    Fast_Profile_Reset();
    Fast_Profile.Request = 0U;
    Fast_Profile.Ready = 0U;
    Fast_Profile.Run = 1U;
}

void Fast_Profile_End_Cycle(void)
{
    if (Fast_Profile.Run == 0U)
    {
        return;
    }

    Fast_Profile.Sample_Cnt++;

    if (Fast_Profile.Sample_Cnt >= FAST_PROFILE_SAMPLE_NUM)
    {
        Fast_Profile.Run = 0U;
        Fast_Profile.Ready = 1U;
    }
}

void Fast_Profile_Add(volatile Fast_Profile_Stat_T *Stat, uint32_t Cyc)
{
    if (Fast_Profile.Run == 0U)
    {
        return;
    }

    Stat->Cnt++;
    Stat->Sum += Cyc;

    if (Cyc < Stat->Min)
    {
        Stat->Min = Cyc;
    }

    if (Cyc > Stat->Max)
    {
        Stat->Max = Cyc;
    }
}
