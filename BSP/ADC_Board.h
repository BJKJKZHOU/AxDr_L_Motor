/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ADC_BOARD_H
#define ADC_BOARD_H

#include "stm32g4xx_ll_adc.h"

/* AxDrive-L V1.3：Ia=PA2、Ib=PA1、Ic=PA0。
 * 初始 Ia/Ib 和母线电压采样由 CubeMX 配置。
 * 两个 ADC 均可采集 Ib，以便切换 AB、AC、BC 采样组合。 */
#define ADC1_IA_CH LL_ADC_CHANNEL_3
#define ADC1_IB_CH LL_ADC_CHANNEL_2
#define ADC2_IB_CH LL_ADC_CHANNEL_2
#define ADC2_IC_CH LL_ADC_CHANNEL_1

#endif /* ADC_BOARD_H */
