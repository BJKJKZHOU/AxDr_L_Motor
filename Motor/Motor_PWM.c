/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_PWM.h"

#include "control_params.h"
#include "tim.h"

#define PWM_CCER_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)

void PWM_Timing_Update(void)
{
    RCC_ClkInitTypeDef Clk;
    uint32_t Flash_Latency;
    uint32_t Timer_Clk;
    uint32_t Arr;
    uint32_t Adc_Offset;
    uint32_t Enc_Lead;

    HAL_RCC_GetClockConfig(&Clk, &Flash_Latency);
    (void)Flash_Latency;

    Timer_Clk = HAL_RCC_GetPCLK2Freq();
    if (Clk.APB2CLKDivider != RCC_HCLK_DIV1)
    {
        Timer_Clk *= 2U;
    }

    Arr = (uint32_t)(((float)Timer_Clk / (2.0f * PWM_FREQ_HZ_DEFAULT)) + 0.5f);
    Adc_Offset = (uint32_t)(((float)Timer_Clk * ADC_TRIG_CENTER_S) + 0.5f);
    Enc_Lead = (uint32_t)(((float)Timer_Clk * ENC_TRIG_LEAD_S) + 0.5f);

    if ((Arr <= Adc_Offset) || ((Arr - Adc_Offset) <= Enc_Lead))
    {
        return;
    }

    htim1.Init.Period = Arr;
    TIM1->ARR = Arr;
    TIM1->CCR4 = Arr - Adc_Offset - Enc_Lead;
    TIM1->CCR5 = Arr - Adc_Offset;

    /* Load the new ARR/CCR preload values before TIM1 starts. */
    TIM1->EGR = TIM_EGR_UG;
    CLEAR_BIT(TIM1->SR, TIM_SR_UIF);
}

void PWM_Update(float DutyA, float DutyB, float DutyC)
{
    float Arr;

    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        return;
    }

    Arr = (float)TIM1->ARR;

    TIM1->CCR1 = (uint32_t)(DutyA * Arr);
    TIM1->CCR2 = (uint32_t)(DutyB * Arr);
    TIM1->CCR3 = (uint32_t)(DutyC * Arr);
}

void PWM_Enable(void)
{
    uint32_t Half;

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE);

    Half = TIM1->ARR / 2U;
    TIM1->CCR1 = Half;
    TIM1->CCR2 = Half;
    TIM1->CCR3 = Half;

    SET_BIT(TIM1->CCER, PWM_CCER_MASK);

    /* AOE opens all six outputs at the UEV that activates the 50% CCR values. */
    SET_BIT(TIM1->BDTR, TIM_BDTR_AOE);

    while ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
    }

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE);
}

void PWM_Disable(void)
{
    uint32_t Half;

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE);
    CLEAR_BIT(TIM1->CCER, PWM_CCER_MASK);

    Half = TIM1->ARR / 2U;
    TIM1->CCR1 = Half;
    TIM1->CCR2 = Half;
    TIM1->CCR3 = Half;
}
