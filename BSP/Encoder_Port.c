/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Encoder_Port.h"

#include "ABZ.h"
#include "main.h"
#include "spi.h"

/* CN47 从 GND 起：GND、A(PB5/CH2)、B(PB4/CH1)、Z(PB3)、空(PD2)、VCC。 */
void Encoder_Port_Release(void)
{
    GPIO_InitTypeDef Gpio = { 0 };

    /* 同时停止接收 DMA 和 MT6835 使用的发送 DMA。 */
    CLEAR_BIT(ENC_DMA_TX->CCR, DMA_CCR_EN);
    CLEAR_BIT(ENC_DMA_RX->CCR,
              DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    ENC_DMA->IFCR = ENC_DMA_TX_CLEAR | ENC_DMA_RX_CLEAR;
    if (hspi1.State != HAL_SPI_STATE_RESET)
    {
        CLEAR_BIT(ENC_SPI->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        ENC_CS_PORT->BSRR = ENC_CS_PIN;
        /* 复位 SPI，清除未完成传输留下的数据。 */
        SET_BIT(ENC_SPI_RST_REG, ENC_SPI_RST);
        CLEAR_BIT(ENC_SPI_RST_REG, ENC_SPI_RST);
        (void)HAL_SPI_DeInit(&hspi1);
    }

    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    ENCODER_ABZ_TIM->CR1 &= ~TIM_CR1_CEN;
    EXTI->IMR1 &= ~GPIO_PIN_3;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);

    /* 先将共用引脚设为输入，避免切换时驱动编码器信号线。 */
    Gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    Gpio.Mode = GPIO_MODE_INPUT;
    Gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &Gpio);
    Gpio.Pin = ENC_CS_PIN;
    Gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_CS_PORT, &Gpio);
}

void Encoder_Port_SPI(void)
{
    GPIO_InitTypeDef Gpio = { 0 };

    ENC_CS_PORT->BSRR = ENC_CS_PIN;
    Gpio.Pin = ENC_CS_PIN;
    Gpio.Mode = GPIO_MODE_OUTPUT_PP;
    Gpio.Pull = GPIO_PULLUP;
    Gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENC_CS_PORT, &Gpio);
    MX_SPI1_Init();
    /* 配置发送 DMA 请求，传输方式由器件驱动设置。 */
    DMAMUX1_Channel2->CCR = DMA_REQUEST_SPI1_TX;
}

void Encoder_Port_ABZ(uint32_t Cpr)
{
    GPIO_InitTypeDef Gpio = { 0 };

    Gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    Gpio.Mode = GPIO_MODE_AF_PP;
    Gpio.Pull = GPIO_NOPULL;
    Gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    Gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &Gpio);

    Gpio.Pin = GPIO_PIN_3;
    Gpio.Mode = GPIO_MODE_IT_RISING;
    Gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &Gpio);

    /* 计数范围设为一机械圈，支持不同编码器线数。 */
    ENCODER_ABZ_TIM->CR1 = 0U;
    ENCODER_ABZ_TIM->PSC = 0U;
    ENCODER_ABZ_TIM->ARR = Cpr - 1U;
    ENCODER_ABZ_TIM->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    ENCODER_ABZ_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    ENCODER_ABZ_TIM->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    ENCODER_ABZ_TIM->CNT = 0U;
    ENCODER_ABZ_TIM->EGR = TIM_EGR_UG;
    ENCODER_ABZ_TIM->SR = 0U;
    ENCODER_ABZ_TIM->CR1 = TIM_CR1_CEN;

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    /* Z 仅用于圈参考观察，不抢占 ADC 快环。 */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* 中断入口放在此处，避免 CubeMX 导出时覆盖。 */
void EXTI3_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        if ((ENCODER_ABZ_TIM->CR1 & TIM_CR1_CEN) != 0U)
        {
            ABZ_Index(ENCODER_ABZ_TIM->CNT);
        }
    }
}
