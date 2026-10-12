/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_PWM.h"

#include "control_params.h"
#include "main.h"
#include "tim.h"

#define PWM_CCER_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define ARR_CCR_LIMIT_TICKS 2125U   /* 12.5 us at TIM1 = 170 MHz, PSC = 0. */
#define ARR_CCR_HIST_TICKS  43U    /* Approx. 0.253 us per histogram bin. */
#define ARR_CCR_WINDOW_N    1000U /* 50 ms of PWM-active control at 20 kHz. */

volatile uint32_t Fast_CCR_Hist[FAST_CCR_HIST_COUNT] = { 0 };

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
    uint32_t CCR_T0;
    uint32_t Cnt;
    uint32_t Dir;
    uint32_t Ticks;
    uint32_t Cyc;
    uint32_t Bin;
    uint32_t Sum;

    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        return;
    }

    Arr = (float)TIM1->ARR;

    TIM1->CCR1 = (uint32_t)(DutyA * Arr);
    TIM1->CCR2 = (uint32_t)(DutyB * Arr);
    TIM1->CCR3 = (uint32_t)(DutyC * Arr);

    /* Capture at the register-write boundary before accounting overhead.
     * TIM1 CNT is the ARR-peak reference; DWT measures ADC IRQ -> CCR. */
    Cnt = TIM1->CNT;
    CCR_T0 = DWT->CYCCNT;
    Dir = TIM1->CR1;

    Cyc = CCR_T0 - Fast_Time.ADC_IRQ_T0;
    Fast_Time.ADC_CCR_Cyc = Cyc;
    if (Cyc > Fast_Time.ADC_CCR_Max)
    {
        Fast_Time.ADC_CCR_Max = Cyc;
    }

    Fast_Time.ARR_CCR_Count++;
    if (((Dir & TIM_CR1_DIR) == 0U) || (Cnt > (uint32_t)Arr) ||
        (Fast_Time.T0 != Fast_Time.ADC_PWM_T0))
    {
        Fast_Time.ARR_CCR_Late++;
        return;
    }

    Ticks = (uint32_t)Arr - Cnt;
    Fast_Time.ARR_CCR_Tick = Ticks;
    if (Ticks < Fast_Time.ARR_CCR_Min)
    {
        Fast_Time.ARR_CCR_Min = Ticks;
    }

    Sum = Fast_Time.ARR_CCR_Sum_Lo + Ticks;
    if (Sum < Fast_Time.ARR_CCR_Sum_Lo)
    {
        Fast_Time.ARR_CCR_Sum_Hi++;
    }
    Fast_Time.ARR_CCR_Sum_Lo = Sum;

    Bin = Ticks / ARR_CCR_HIST_TICKS;
    if (Bin >= FAST_CCR_HIST_COUNT)
    {
        Bin = FAST_CCR_HIST_COUNT - 1U;
    }
    Fast_CCR_Hist[Bin]++;

    if (Ticks > ARR_CCR_LIMIT_TICKS)
    {
        Fast_Time.ARR_CCR_Over++;
        Fast_Time.ARR_CCR_Win_Over = 1U;
    }

    if (Ticks > Fast_Time.ARR_CCR_Tick_Max)
    {
        Fast_Time.ARR_CCR_Tick_Max = Ticks;
        Fast_Time.ARR_CCR_Worst_Seq = Fast_Time.ARR_CCR_Count;
        Fast_Time.ARR_CCR_Worst_ADC = Fast_Time.ADC_CCR_Cyc;
        Fast_Time.ARR_CCR_Worst_ESO = Fast_Time.ESO_Cyc;
        Fast_Time.ARR_CCR_Worst_PI = Fast_Time.PI_Cyc;
        Fast_Time.ARR_CCR_Worst_SVPWM = Fast_Time.SVPWM_Cyc;
    }

    if (Ticks > Fast_Time.ARR_CCR_Win_Max)
    {
        Fast_Time.ARR_CCR_Win_Max = Ticks;
    }
    if (++Fast_Time.ARR_CCR_Win_N == ARR_CCR_WINDOW_N)
    {
        Fast_Time.ARR_CCR_Win_Total++;
        Fast_Time.ARR_CCR_Win_Bad += Fast_Time.ARR_CCR_Win_Over;
        Fast_Time.ARR_CCR_Win_Last = Fast_Time.ARR_CCR_Win_Max;
        Fast_Time.ARR_CCR_Win_N = 0U;
        Fast_Time.ARR_CCR_Win_Max = 0U;
        Fast_Time.ARR_CCR_Win_Over = 0U;
    }
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
