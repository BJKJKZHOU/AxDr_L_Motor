/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "ABZ.h"

#include "Encoder.h"
#include "Encoder_Port.h"
#include "Math.h"
#include "main.h"

/* 首次 Z 上升沿的计数作为每圈参考。 */
static uint16_t Index_Ref_Cnt = 0U;
static volatile int32_t Index_Error_Cnt = 0;
static uint8_t Index_Seen = 0U;

void ABZ_Config(void)
{
    Index_Seen = 0U;
    Index_Error_Cnt = 0;

    Encoder_Port_ABZ(4U * Encoder_Config.ABZ_PPR);
}

FAST_CODE void ABZ_Start(void)
{
    uint32_t Cnt = ENCODER_ABZ_TIM->CNT;

    /* A/B 计数换算为机械角度，电角零位由寻相确定。 */
    Encoder_Sample_Update(Cnt, (float)Cnt * (TWO_PI_F / (float)(ENCODER_ABZ_TIM->ARR + 1U)));
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;
}

void ABZ_Index(uint32_t Cnt)
{
    uint32_t Cpr = ENCODER_ABZ_TIM->ARR + 1U;
    int32_t Delta;

    if (Index_Seen == 0U)
    {
        Index_Ref_Cnt = (uint16_t)Cnt;
        Index_Seen = 1U;
        return;
    }

    /* 中断读取计数存在延迟，Z 相只用于观察每圈偏差。 */
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
