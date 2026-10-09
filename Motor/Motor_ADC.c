/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_ADC.h"

#include <stdbool.h>

#include "Plot.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Mechanical_ESO.h"
#include "Math.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_PWM.h"
#include "Protection.h"
#include "Sin_LUT.h"
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
    bool Current_OK;
    bool Encoder_Park_Valid;
    float Theta_e;
    float Ialpha;
    float Ibeta;
    float Sin;
    float Cos;
    float Iq_Encoder;
    float Ud;
    float Uq;
    float U_Lim;
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

    Ialpha = ADC.Ia_A;
    Ibeta = (ADC.Ia_A + 2.0f * ADC.Ib_A) * INV_SQRT3_F;
    Encoder_Park_Valid = false;
    Iq_Encoder = 0.0f;

    /* Check current protection before observer work; trip disables PWM here. */
    Current_OK = Protection_Current_Fast(ADC.Ia_A, ADC.Ib_A, ADC.Ic_A);

    /* Feedback remains live with PWM off and in every control mode. */
    if ((Encoder.Ready != 0U) && (Encoder.Fault == 0U) &&
        (Mechanical_ESO.Para.Valid != 0U))
    {
        float Theta_m = Encoder.Theta_m;

        if (Motor_Cal.Valid != 0U)
        {
            /* ESO uses the encoder frame, including coast current. */
            SinCos(Angle_Wrap((float)Motor_Para.Pp * Theta_m + Motor_Cal.Theta_Off), &Sin, &Cos);
            Iq_Encoder = -Ialpha * Sin + Ibeta * Cos;
            Encoder_Park_Valid = true;
        }

        Mechanical_ESO_Run(Theta_m, Encoder.Valid != 0U, Iq_Encoder);
    }

    if (!Current_OK)
    {
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
        goto finish;
    }

    Fast_Mode = Motor_Fast_Run(&Theta_e, &Id_Ref, &Iq_Ref, &Ualpha, &Ubeta);
    Motor_Plot_Iq_Ref = Iq_Ref;

    if (Fast_Mode == FAST_OFF)
    {
        Motor_Run.Ud = 0.0f;
        Motor_Run.Uq = 0.0f;
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
        /* End an active flow before the slow loop can leave a spinning motor shorted. */
        if (Motor_State_Get() == RUN)
        {
            PWM_Disable();
        }

        goto finish;
    }

    if (Fast_Mode == FAST_CURRENT)
    {
        /* Motor_Fast_Run may use the previous-cycle dq values (IF/ident).
         * Publish this sample only after it has selected the FOC angle. */
        /* Only the three sensored servo modes use the encoder FOC frame. */
        if (!Encoder_Park_Valid || (Motor_Mode > POSITION))
        {
            SinCos(Theta_e, &Sin, &Cos);
            Motor_Run.Iq = -Ialpha * Sin + Ibeta * Cos;
        }
        else
        {
            Motor_Run.Iq = Iq_Encoder;
        }
        Motor_Run.Id = Ialpha * Cos + Ibeta * Sin;
        Motor_Run.Theta_e = Theta_e;

        U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
        Current_Loop(Id_Ref, Iq_Ref, U_Lim, &Ud, &Uq);

        /* Encoder servo dq decoupling and back-EMF feedforward. */
        if ((Current_FF_Enable != 0U) && (Motor_Mode <= POSITION) && Encoder_Park_Valid)
        {
            float We = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;

            Ud -= We * Motor_Para.Lq * Motor_Run.Iq;
            Uq += We * (Motor_Para.Ld * Motor_Run.Id + Motor_Para.Flux);
        }

        Vector2_Limit(&Ud, &Uq, U_Lim);

        Motor_Run.Ud = Ud;
        Motor_Run.Uq = Uq;
        Ualpha = Ud * Cos - Uq * Sin;
        Ubeta = Ud * Sin + Uq * Cos;
    }

    Motor_Run.Ualpha = Ualpha;
    Motor_Run.Ubeta = Ubeta;

    SVPWM_Calc(Ualpha, Ubeta, ADC.Vbus_V, &DutyA, &DutyB, &DutyC);
    PWM_Update(DutyA, DutyB, DutyC);

finish:
    Plot_Fast_Sample();
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
