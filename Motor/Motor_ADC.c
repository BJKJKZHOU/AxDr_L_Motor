/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_ADC.h"

#include "Plot.h"
#include "Current_Loop.h"
#include "Fast_Profile.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Voltage_Mod.h"
#include "adc.h"
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

    /* TIM1 CH5/TRGO2 triggers ADC injected without enabling phase PWM. */
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

        if (HAL_ADCEx_InjectedPollForConversion(&hadc2, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        Ia_Sum += ADC1->JDR3;
        Ib_Sum += ADC1->JDR2;
        Ic_Sum += ADC1->JDR1;
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);

    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    ADC.Ia_Off = (uint16_t)((Ia_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);
    ADC.Ib_Off = (uint16_t)((Ib_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);
    ADC.Ic_Off = (uint16_t)((Ic_Sum + (ADC_SAMPLE_NUM / 2U)) / ADC_SAMPLE_NUM);
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
    uint32_t Profile_T0 = 0U;
    uint32_t Segment_T0 = 0U;

    T0 = DWT->CYCCNT;
    Fast_Profile_Begin_Cycle();

    if (Fast_Profile.Run != 0U)
    {
        Profile_T0 = DWT->CYCCNT;
        Segment_T0 = Profile_T0;
    }

    ADC.Ia_Raw = (uint16_t)ADC1->JDR3;
    ADC.Ib_Raw = (uint16_t)ADC1->JDR2;
    ADC.Ic_Raw = (uint16_t)ADC1->JDR1;
    ADC.Vbus_Raw = (uint16_t)ADC2->JDR1;

    ADC.Ia_A = ((float)ADC.Ia_Off - (float)ADC.Ia_Raw) * CUR_RAW_TO_A;
    ADC.Ib_A = ((float)ADC.Ib_Off - (float)ADC.Ib_Raw) * CUR_RAW_TO_A;
    ADC.Ic_A = ((float)ADC.Ic_Off - (float)ADC.Ic_Raw) * CUR_RAW_TO_A;
    ADC.Vbus_V = (float)ADC.Vbus_Raw * VBUS_RAW_TO_V;

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.ADC_Sample, DWT->CYCCNT - Segment_T0);
        Segment_T0 = DWT->CYCCNT;
    }

    Fast_Mode = Motor_Fast_Run(&Theta_e, &Id_Ref, &Iq_Ref, &Ualpha, &Ubeta);

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.Motor_Fast, DWT->CYCCNT - Segment_T0);
    }

    if (Fast_Mode == FAST_OFF)
    {
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;

        if (Motor_State_Get() == RUN)
        {
            PWM_Disable();
        }

        if (Fast_Profile.Run != 0U)
        {
            Segment_T0 = DWT->CYCCNT;
        }

        Plot_Fast_Sample();

        if (Fast_Profile.Run != 0U)
        {
            Fast_Profile_Add(&Fast_Profile.Plot_Fast, DWT->CYCCNT - Segment_T0);
        }

        goto finish;
    }

    if (Fast_Mode == FAST_CURRENT)
    {
        Motor_Run.Theta_e = Theta_e;

        if (Fast_Profile.Run != 0U)
        {
            Segment_T0 = DWT->CYCCNT;
        }

        Current_Loop(Id_Ref, Iq_Ref, &Ualpha, &Ubeta);

        if (Fast_Profile.Run != 0U)
        {
            Fast_Profile_Add(&Fast_Profile.Current_Loop, DWT->CYCCNT - Segment_T0);
        }
    }

    Motor_Run.Ualpha = Ualpha;
    Motor_Run.Ubeta = Ubeta;

    if (Fast_Profile.Run != 0U)
    {
        Segment_T0 = DWT->CYCCNT;
    }

    SVPWM_Calc(Ualpha, Ubeta, ADC.Vbus_V, &DutyA, &DutyB, &DutyC);

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.SVPWM, DWT->CYCCNT - Segment_T0);
        Segment_T0 = DWT->CYCCNT;
    }

    PWM_Update(DutyA, DutyB, DutyC);

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.PWM_Update, DWT->CYCCNT - Segment_T0);
        Segment_T0 = DWT->CYCCNT;
    }

    Plot_Fast_Sample();

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.Plot_Fast, DWT->CYCCNT - Segment_T0);
    }

finish:
    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.ADC_Run, DWT->CYCCNT - Profile_T0);
        Fast_Profile_End_Cycle();
    }

    Cyc = DWT->CYCCNT - T0;
    Fast_Time.ADC_Run_Cyc = Cyc;

    if (Cyc > Fast_Time.ADC_Run_Max)
    {
        Fast_Time.ADC_Run_Max = Cyc;
    }
}
