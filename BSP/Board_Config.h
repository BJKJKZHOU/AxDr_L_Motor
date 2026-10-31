/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"
#include "stm32g4xx_ll_adc.h"

/* AxDrive-L V1.3 电流和母线电压采样。 */
#define ADC_VREF_V     3.3f
#define ADC_FULL_SCALE 4096.0f
#define CUR_SHUNT_OHM  0.001f
#define CUR_AMP_GAIN   20.0f
#define VBUS_R1_OHM    20000.0f
#define VBUS_R2_OHM    1000.0f

/* 三相采样通道：Ia=PA2、Ib=PA1、Ic=PA0。 */
#define ADC1_IA_CH LL_ADC_CHANNEL_3
#define ADC1_IB_CH LL_ADC_CHANNEL_2
#define ADC2_IB_CH LL_ADC_CHANNEL_2
#define ADC2_IC_CH LL_ADC_CHANNEL_1

/* 编码器接口：SPI1 与 TIM3 ABZ 共用 CN47。 */
#define ENC_SPI             SPI1
#define ENC_SPI_RST_REG     RCC->APB2RSTR
#define ENC_SPI_RST         RCC_APB2RSTR_SPI1RST
#define ENC_CS_PORT         SPI1_CSN_GPIO_Port
#define ENC_CS_PIN          SPI1_CSN_Pin
#define ENC_DMA             DMA1
#define ENC_DMA_TX          DMA1_Channel3
#define ENC_DMA_RX          DMA1_Channel5
#define ENC_DMA_TX_CLEAR    DMA_IFCR_CGIF3
#define ENC_DMA_RX_CLEAR    DMA_IFCR_CGIF5
#define ENC_DMA_RX_HT_CLEAR DMA_IFCR_CHTIF5
#define ENC_DMA_RX_TC_CLEAR DMA_IFCR_CTCIF5
#define ENC_DMA_TX_ERR      DMA_ISR_TEIF3
#define ENC_DMA_RX_ERR      DMA_ISR_TEIF5
#define ENC_DMA_RX_HT       DMA_ISR_HTIF5
#define ENC_DMA_RX_TC       DMA_ISR_TCIF5
#define ENCODER_ABZ_TIM     TIM3
#define ENCODER_ABZ_CPR_MAX 65536U

/* AxDrive-L V1.3 产品级运行上限，独立于具体编码器协议。 */
#define DRIVE_I_MAX_A   10.0f
#define DRIVE_WM_MAX_RPM 25000.0f
#define DRIVE_WM_MAX    (DRIVE_WM_MAX_RPM * 6.283185307f / 60.0f)

/* AxDrive-L V1.3 固定母线保护阈值。 */
#define VBUS_UV_FAULT_V          5.0f
#define VBUS_OV_FAULT_V         52.0f
#define VBUS_UV_DEBOUNCE_TICKS  20U /* 2 kHz Motor Thread 下 10 ms。 */
#define VBUS_OV_DEBOUNCE_TICKS   1U /* 2 kHz Motor Thread 下 0.5 ms。 */

/* AxDrive-L V1.3 软件过流保护阈值。 */
#define PROT_CURRENT_OVERLOAD_A       15.0f
#define PROT_CURRENT_OVERLOAD_CYCLES  5000U /* 20 kHz 下 250 ms。 */
#define PROT_CURRENT_FAST_A           25.0f
#define PROT_CURRENT_FAST_CYCLES         5U /* 20 kHz 下 250 us。 */

#endif /* BOARD_CONFIG_H */
