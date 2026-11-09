/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "ABZ.h"

#include "Enc_Home.h"
#include "Encoder.h"
#include "Encoder_Port.h"
#include "Math.h"
#include "main.h"

volatile int32_t ABZ_Index_Cnt = -1;

/* 首次 Z 上升沿的计数作为每圈参考。 */
static uint16_t Index_Ref_Cnt = 0U;
static int32_t Cnt_Off = 0;
static uint8_t Index_Seen = 0U;

void ABZ_Config(void)
{
    Index_Seen = 0U;
    Cnt_Off = 0;
    ABZ_Index_Cnt = -1;

    Encoder_Port_ABZ(4U * Encoder_Config.ABZ_PPR);
}

FAST_CODE void ABZ_Start(void)
{
    uint32_t Raw = ENCODER_ABZ_TIM->CNT;
    int32_t Cpr = (int32_t)(ENCODER_ABZ_TIM->ARR + 1U);
    int32_t Index_Cnt = ABZ_Index_Cnt;
    int32_t Cnt;

    if (Index_Cnt >= 0)
    {
        /* EXTI3 与采样中断同级，读取和清除期间不会写入新计数。 */
        ABZ_Index_Cnt = -1;
        if (Index_Seen == 0U)
        {
            Index_Ref_Cnt = (uint16_t)Index_Cnt;
            Index_Seen = 1U;
        }

        /* 使用 Z 采样值计算偏差，延后处理期间的转动不计入误差。 */
        Cnt_Off = (int32_t)Index_Ref_Cnt - Index_Cnt;
    }

    Cnt = (int32_t)Raw + Cnt_Off;

    if (Cnt >= Cpr)
    {
        Cnt -= Cpr;
    }
    else if (Cnt < 0)
    {
        Cnt += Cpr;
    }

    /* 原始计数用于观测，角度使用 Z 校正后的计数。 */
    Encoder_Sample_Update(Raw, (float)Cnt * (TWO_PI_F / (float)Cpr));
    if (Index_Cnt >= 0)
    {
        Enc_Home_Trigger((float)Index_Ref_Cnt * (TWO_PI_F / (float)Cpr));
    }
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;
}
