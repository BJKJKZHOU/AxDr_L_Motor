#include "Motor_ADC.h"

#include "adc.h"
#include "main.h"
#include "tim.h"


#define ADC_SAMPLE_NUM      512U
#define ADC_TRIG_CCR        3900U

#define ADC_VREF_V          3.3f
#define ADC_FULL_SCALE      4096.0f

#define CUR_SHUNT_OHM       0.001f
#define CUR_AMP_GAIN        20.0f

#define VBUS_R1_OHM         20000.0f
#define VBUS_R2_OHM         1000.0f

#define CUR_RAW_TO_A (ADC_VREF_V / ADC_FULL_SCALE / CUR_SHUNT_OHM / CUR_AMP_GAIN)

#define VBUS_RAW_TO_V (ADC_VREF_V / ADC_FULL_SCALE * ((VBUS_R1_OHM + VBUS_R2_OHM) / VBUS_R2_OHM))


volatile ADC_T ADC = {0};


static void Iabc_Calib(void);


void ADC_Calib(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    Iabc_Calib();
}


static void Iabc_Calib(void)
{
    uint32_t Ia_Sum = 0U;
    uint32_t Ib_Sum = 0U;
    uint32_t Ic_Sum = 0U;

    /*
     * TIM1 CH4 only provides the ADC injected trigger.
     * No phase PWM output is enabled here.
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ADC_TRIG_CCR);

    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    for (uint32_t n = 0U; n < ADC_SAMPLE_NUM; n++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_ADCEx_InjectedPollForConversion(&hadc2, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        Ia_Sum += ADC1->JDR3;
        Ib_Sum += ADC1->JDR2;
        Ic_Sum += ADC1->JDR1;
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    ADC.Ia_Off =
        (uint16_t)((Ia_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);

    ADC.Ib_Off =
        (uint16_t)((Ib_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);

    ADC.Ic_Off =
        (uint16_t)((Ic_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);
}


void ADC_Sample(void)
{
    ADC.Ia_Raw = (uint16_t)ADC1->JDR3;
    ADC.Ib_Raw = (uint16_t)ADC1->JDR2;
    ADC.Ic_Raw = (uint16_t)ADC1->JDR1;

    ADC.Vbus_Raw = (uint16_t)ADC2->JDR1;

    ADC.Ia_A = ((float)ADC.Ia_Raw - (float)ADC.Ia_Off) * CUR_RAW_TO_A;
    ADC.Ib_A = ((float)ADC.Ib_Raw - (float)ADC.Ib_Off) * CUR_RAW_TO_A;
    ADC.Ic_A = ((float)ADC.Ic_Raw - (float)ADC.Ic_Off) * CUR_RAW_TO_A;
    ADC.Vbus_V = (float)ADC.Vbus_Raw * VBUS_RAW_TO_V;
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    ADC_Sample();
}