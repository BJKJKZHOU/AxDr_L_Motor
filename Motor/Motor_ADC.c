/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_ADC.h"

#include "Plot.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Mechanical_ESO.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Protection.h"
#include "Voltage_Mod.h"
#include "adc.h"
#include "control_params.h"
#include "main.h"
#include "tim.h"

#define ADC_SAMPLE_NUM 512U
#define ADC_VREF_V     3.3f
#define ADC_FULL_SCALE 4096.0f

#define CUR_SHUNT_OHM 0.001f
#define CUR_AMP_GAIN  20.0f

#define VBUS_R1_OHM 20000.0f
#define VBUS_R2_OHM 1000.0f

#define CUR_RAW_TO_A (ADC_VREF_V / ADC_FULL_SCALE / CUR_SHUNT_OHM / CUR_AMP_GAIN)
#define VBUS_RAW_TO_V (ADC_VREF_V / ADC_FULL_SCALE * ((VBUS_R1_OHM + VBUS_R2_OHM) / VBUS_R2_OHM))

volatile ADC_T ADC = { 0 };

volatile float Motor_Plot_Iq_Ref = 0.0f;

static void Iab_Calib(void);

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

    Iab_Calib();
}

static void Iab_Calib(void)
{
    uint32_t Ia_Sum = 0U;
    uint32_t Ib_Sum = 0U;

    /*
     * TIM1 CH5/TRGO2 triggers both injected ADCs.
     * ADC1 JDR1 = Ia; ADC2 JDR1 = Ib; ADC2 JDR2 = Vbus.
     * Offset calibration only uses the simultaneous current Rank1 samples.
     */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5) != HAL_OK)
    {
        Error_Handler();
    }

    for (uint32_t n = 0U; n < ADC_SAMPLE_NUM; n++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        Ia_Sum += ADC1->JDR1;
        Ib_Sum += ADC2->JDR1;
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);

    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    ADC.Ia_Off = (uint16_t)((Ia_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);
    ADC.Ib_Off = (uint16_t)((Ib_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);

    /* Fixed-AB first stage: Ic is reconstructed after Ia/Ib offset correction. */
    ADC.Ic_Off = 0U;
    ADC.Ic_Raw = 0U;
}

void Fast_Loop(void)
{
    Motor_Fast_Mode_e Fast_Mode;
    float Theta_e;
    float Id_Ref;
    float Iq_Ref;
    float Ualpha;
    float Ubeta;
    float DutyA;
    float DutyB;
    float DutyC;
    uint32_t T0;
    uint32_t Cyc;

    T0 = DWT->CYCCNT;

    /*
     * Ia/Ib are Rank1 simultaneous samples. Vbus is ADC2 injected Rank2.
     * The ADC1 Rank1 JEOS path drives the fast loop, so JDR2 may represent
     * the completed value from the previous trigger; <= 50 us latency is
     * negligible for the DC-bus measurement and avoids delaying current FOC.
     */
    ADC.Ia_Raw = (uint16_t)ADC1->JDR1;
    ADC.Ib_Raw = (uint16_t)ADC2->JDR1;
    ADC.Vbus_Raw = (uint16_t)ADC2->JDR2;

    ADC.Ia_A = ((float)ADC.Ia_Off - (float)ADC.Ia_Raw) * CUR_RAW_TO_A;
    ADC.Ib_A = ((float)ADC.Ib_Off - (float)ADC.Ib_Raw) * CUR_RAW_TO_A;
    ADC.Ic_A = -(ADC.Ia_A + ADC.Ib_A);
    ADC.Vbus_V = (float)ADC.Vbus_Raw * VBUS_RAW_TO_V;

    if (!Protection_Current_Fast(ADC.Ia_A, ADC.Ib_A, ADC.Ic_A))
    {
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
        Plot_Fast_Sample();
        goto finish;
    }

    Fast_Mode = Motor_Fast_Run(&Theta_e, &Id_Ref, &Iq_Ref, &Ualpha, &Ubeta);
    Motor_Plot_Iq_Ref = Iq_Ref;

    if (Fast_Mode == FAST_OFF)
    {
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;

        if (Motor_State_Get() == RUN)
        {
            PWM_Disable();
        }

        Plot_Fast_Sample();
        goto finish;
    }

    if (Fast_Mode == FAST_CURRENT)
    {
        Motor_Run.Theta_e = Theta_e;
        Current_Loop(Id_Ref, Iq_Ref, &Ualpha, &Ubeta);

        if (Motor_Mechanical_ESO_Required())
        {
            Mechanical_ESO_Run(Encoder_Position_Get(),
                               Encoder.Valid != 0U,
                               Motor_Run.Iq);
        }
    }

    Motor_Run.Ualpha = Ualpha;
    Motor_Run.Ubeta = Ubeta;

    SVPWM_Calc(Ualpha, Ubeta, ADC.Vbus_V, &DutyA, &DutyB, &DutyC);
    PWM_Update(DutyA, DutyB, DutyC);
    Plot_Fast_Sample();

finish:
    Cyc = DWT->CYCCNT - T0;
    Fast_Time.ADC_Run_Cyc = Cyc;

    if (Cyc > Fast_Time.ADC_Run_Max)
    {
        Fast_Time.ADC_Run_Max = Cyc;
    }
}

float Motor_Iq_Ref_Get(void)
{
    return Motor_Plot_Iq_Ref;
}
