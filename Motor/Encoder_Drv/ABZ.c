/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "ABZ.h"

#include "Encoder.h"
#include "Encoder_Port.h"
#include "Math.h"
#include "main.h"

static uint32_t Cpr = 0U;
/* Z is an optional per-turn reference, not a prerequisite for AB feedback. */
static uint16_t Index_Ref_Cnt = 0U;
static volatile int32_t Index_Error_Cnt = 0;
static uint8_t Index_Seen = 0U;

void ABZ_Config(void)
{
    uint32_t Ppr = Encoder_Config.ABZ_PPR;

    if ((Ppr == 0U) || (Ppr > (ENCODER_ABZ_CPR_MAX / 4U)))
    {
        Encoder.Fault = 1U;
        return;
    }

    Cpr = 4U * Ppr;
    Index_Seen = 0U;
    Index_Ref_Cnt = 0U;
    Index_Error_Cnt = 0;

    Encoder_Port_ABZ(Cpr);
}

FAST_CODE void ABZ_Start(void)
{
    uint32_t Cnt = ENCODER_ABZ_TIM->CNT;

    /* A/B provides relative position immediately, even with Z unconnected.
     * The current-session electrical zero is established by phase search. */
    Encoder_Sample_Update(Cnt, (float)Cnt * (TWO_PI_F / (float)Cpr));
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;
}

void ABZ_Index(uint32_t Cnt)
{
    int32_t Delta;

    if (Index_Seen == 0U)
    {
        Index_Ref_Cnt = (uint16_t)Cnt;
        Index_Seen = 1U;
        return;
    }

    /* A Z rising edge repeats once per mechanical revolution.
     * Only observe drift here: IRQ latency is not an exact edge latch,
     * so never modify the counter or the FOC angle from this interrupt. */
    Delta = (int32_t)Cnt - (int32_t)Index_Ref_Cnt;
    if (Delta > (int32_t)(Cpr / 2U))
    {
        Delta -= (int32_t)Cpr;
    }
    else if (Delta < -(int32_t)(Cpr / 2U))
    {
        Delta += (int32_t)Cpr;
    }
    Index_Error_Cnt = Delta;
}
