/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "ABZ.h"

#include "Encoder.h"
#include "Math.h"
#include "main.h"
#include "stm32g4xx_hal.h"

/* CN47 from GND: GND, A (PB5/CH2), B (PB4/CH1), Z (PB3), NC, VCC.
 * Phase search establishes Enc_Dir with this A/B channel assignment.
 * TIM3 is 16-bit, so quadrature x4 CPR cannot exceed 65536.
 */
#define ABZ_PPR_MAX 16384U

static uint32_t Cpr = 0U;
/* Z is an optional per-turn reference, not a prerequisite for AB feedback. */
static uint16_t Index_Ref_Cnt = 0U;
static volatile int32_t Index_Error_Cnt = 0;
static uint8_t Index_Seen = 0U;
static volatile uint8_t Active = 0U;

void ABZ_Deinit(void)
{
    if (Active == 0U)
    {
        return;
    }

    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    TIM3->CR1 &= ~TIM_CR1_CEN;
    EXTI->IMR1 &= ~GPIO_PIN_3;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);

    Index_Seen = 0U;
    Index_Error_Cnt = 0;
    Active = 0U;
}

void ABZ_Config(void)
{
    GPIO_InitTypeDef Gpio = { 0 };
    uint32_t Ppr = Encoder_Config.ABZ_PPR;

    if ((Ppr == 0U) || (Ppr > ABZ_PPR_MAX))
    {
        Encoder.Fault = 1U;
        return;
    }

    Cpr = 4U * Ppr;
    Index_Seen = 0U;
    Index_Ref_Cnt = 0U;
    Index_Error_Cnt = 0;

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

    /* Modulo one mechanical revolution also supports arbitrary PPR. */
    TIM3->CR1 = 0U;
    TIM3->PSC = 0U;
    TIM3->ARR = Cpr - 1U;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->CR1 = TIM_CR1_CEN;

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 1U, 0U);
    Active = 1U;
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

FAST_CODE void ABZ_Start(void)
{
    uint32_t Cnt = TIM3->CNT;

    /* A/B provides relative position immediately, even with Z unconnected.
     * The current-session electrical zero is established by phase search. */
    Encoder_Sample_Update(Cnt, (float)Cnt * (TWO_PI_F / (float)Cpr));
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;
}

/* The EXTI3 vector lives here, not in CubeMX-generated stm32g4xx_it.c. */
void EXTI3_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
        if (Active != 0U)
        {
            int32_t Delta;
            uint32_t Cnt = TIM3->CNT;

            if (Index_Seen == 0U)
            {
                Index_Ref_Cnt = (uint16_t)Cnt;
                Index_Seen = 1U;
                return;
            }

            /* A Z rising edge repeats once per mechanical revolution.
             * Only observe drift here: IRQ latency is not an exact edge latch,
             * so never modify CNT or the FOC angle from this interrupt. */
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
    }
}
