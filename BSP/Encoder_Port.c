/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Encoder_Port.h"

#include "ABZ.h"
#include "main.h"
#include "spi.h"

/* CN47 from GND: GND, A (PB5/CH2), B (PB4/CH1), Z (PB3), NC (PD2), VCC.
 * Phase search establishes Enc_Dir with this A/B channel assignment. */
void Encoder_Port_Release(void)
{
    GPIO_InitTypeDef Gpio = { 0 };

    /* MT6835 owns TX channel 3 as well as the shared RX channel 5. */
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel5->CCR,
              DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF5;
    if (hspi1.State != HAL_SPI_STATE_RESET)
    {
        CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        /* Flush queued words; the next SPI selection uses CubeMX's setup. */
        SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
        CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
        (void)HAL_SPI_DeInit(&hspi1);
    }

    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    ENCODER_ABZ_TIM->CR1 &= ~TIM_CR1_CEN;
    EXTI->IMR1 &= ~GPIO_PIN_3;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);

    /* ABZ reconfiguration must never drive A/B/Z as SPI outputs. */
    Gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    Gpio.Mode = GPIO_MODE_INPUT;
    Gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &Gpio);
    Gpio.Pin = SPI1_CSN_Pin;
    Gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &Gpio);
}

void Encoder_Port_SPI(void)
{
    GPIO_InitTypeDef Gpio = { 0 };

    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    Gpio.Pin = SPI1_CSN_Pin;
    Gpio.Mode = GPIO_MODE_OUTPUT_PP;
    Gpio.Pull = GPIO_PULLUP;
    Gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &Gpio);
    MX_SPI1_Init();
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

    /* Cpr is validated by ABZ_Config against the board's counter capacity.
     * Modulo one mechanical revolution also supports arbitrary PPR. */
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
    HAL_NVIC_SetPriority(EXTI3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* Keep this vector outside CubeMX-generated stm32g4xx_it.c. */
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
