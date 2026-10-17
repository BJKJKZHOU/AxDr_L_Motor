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
#include "Signal_Injection.h"
#include "Voltage_Mod.h"
#include "adc.h"
#include "control_params.h"
#include "main.h"
#include "stm32g4xx_ll_adc.h"
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

typedef enum
{
    ADC_SAMPLE_AB = 0,
    ADC_SAMPLE_AC,
    ADC_SAMPLE_BC,
} ADC_Sample_e;

static ADC_Sample_e ADC_Sample = ADC_SAMPLE_AB;
static uint32_t ADC1_JSQR_A;
static uint32_t ADC1_JSQR_B;
static uint32_t ADC2_JSQR_B;
static uint32_t ADC2_JSQR_C;

static FAST_CODE bool ADC_Phase_Set(ADC_Sample_e Phase);
static void Iabc_Calib(void);

void ADC_Calib(void)
{
    uint32_t JSQR1;
    uint32_t JSQR2;

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    JSQR1 = ADC1->JSQR;
    JSQR2 = ADC2->JSQR;

    /* Both ADCs must be independent, with the same TIM1 TRGO2 trigger.
     * ADC1 uses one current rank; ADC2 keeps current + Vbus (rank 2).
     * Generated ADC configuration is authoritative for this initial context. */
    if ((LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1)) != LL_ADC_MULTI_INDEPENDENT) ||
        ((JSQR1 & ADC_JSQR_JEXTEN) == 0U) ||
        (((JSQR1 ^ JSQR2) & (ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN)) != 0U) ||
        ((JSQR1 & ADC_JSQR_JL) != 0U) ||
        ((JSQR2 & ADC_JSQR_JL) != ADC_JSQR_JL_0))
    {
        Error_Handler();
    }

    /* Alternate Rank1 channels use the same acquisition time. */
    MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP2,
               ((ADC1->SMPR1 & ADC_SMPR1_SMP3) >> ADC_SMPR1_SMP3_Pos) << ADC_SMPR1_SMP2_Pos);
    MODIFY_REG(ADC2->SMPR1, ADC_SMPR1_SMP1,
               ((ADC2->SMPR1 & ADC_SMPR1_SMP2) >> ADC_SMPR1_SMP2_Pos) << ADC_SMPR1_SMP1_Pos);

    /* Following the MCSDK approach, precompute the full queue contexts.
     * JQM=0 repeats the last context: only the channels change, never
     * the trigger or length (STM32G474 ES0430 workaround). */
    ADC1_JSQR_A = JSQR1;
    ADC1_JSQR_B = (JSQR1 & ~ADC_JSQR_JSQ1) |
                  ADC_JSQR_RK(ADC_CHANNEL_2, ADC_INJECTED_RANK_1);
    ADC2_JSQR_B = JSQR2;
    ADC2_JSQR_C = (JSQR2 & ~ADC_JSQR_JSQ1) |
                  ADC_JSQR_RK(ADC_CHANNEL_1, ADC_INJECTED_RANK_1);

    /* Setting the queue mode clears JSQR. Re-seed the initial AB pair. */
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_2CONTEXTS_LAST_ACTIVE);
    ADC2->JSQR = ADC2_JSQR_B;
    ADC1->JSQR = ADC1_JSQR_A;
    ADC_Sample = ADC_SAMPLE_AB;

    Iabc_Calib();
}

/* TIM1 CH5/TRGO2 triggers both independent ADCs on the same falling edge.
 * The queue accepts one complete JSQR context per write. Keep JEXTSEL,
 * JEXTEN, JL and ADC2 Vbus Rank2 unchanged while switching Rank1. */
static FAST_CODE bool ADC_Phase_Set(ADC_Sample_e Phase)
{
    if (Phase == ADC_Sample)
    {
        return true;
    }

    if ((Phase == ADC_SAMPLE_BC) != (ADC_Sample == ADC_SAMPLE_BC))
    {
        ADC1->JSQR = (Phase == ADC_SAMPLE_BC) ? ADC1_JSQR_B : ADC1_JSQR_A;
    }

    if ((Phase == ADC_SAMPLE_AB) != (ADC_Sample == ADC_SAMPLE_AB))
    {
        ADC2->JSQR = (Phase == ADC_SAMPLE_AB) ? ADC2_JSQR_B : ADC2_JSQR_C;
    }

    if (((ADC1->ISR | ADC2->ISR) & ADC_ISR_JQOVF) != 0U)
    {
        PWM_Disable();
        Protection_Fault_Set(PROT_ADC);
        return false;
    }

    ADC_Sample = Phase;
    return true;
}

static void Iabc_Calib(void)
{
    uint32_t Ia_Sum = 0U;
    uint32_t Ib_Sum = 0U;
    uint32_t Ib1_Sum = 0U;
    uint32_t Ic_Sum = 0U;

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

    /* AB: Ia on ADC1, Ib on ADC2. */
    for (uint32_t n = 0U; n < ADC_SAMPLE_NUM; n++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        Ia_Sum += ADC1->JDR1;
        Ib_Sum += ADC2->JDR1;
    }

    if (!ADC_Phase_Set(ADC_SAMPLE_BC))
    {
        Error_Handler();
    }

    /* BC: Ib on ADC1, Ic on ADC2. Discard two transition triggers. */
    for (uint32_t n = 0U; n < (ADC_SAMPLE_NUM + 2U); n++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100U) != HAL_OK)
        {
            Error_Handler();
        }

        if (n >= 2U)
        {
            Ib1_Sum += ADC1->JDR1;
            Ic_Sum += ADC2->JDR1;
        }
    }

    /* Restore AB before normal fast-loop sampling starts. */
    if (!ADC_Phase_Set(ADC_SAMPLE_AB))
    {
        Error_Handler();
    }

    for (uint32_t n = 0U; n < 2U; n++)
    {
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100U) != HAL_OK)
        {
            Error_Handler();
        }
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    ADC.Ia_Off = (uint16_t)((Ia_Sum + ADC_SAMPLE_NUM / 2U) / ADC_SAMPLE_NUM);
    ADC.Ib_Off = (uint16_t)((Ib_Sum + ADC_SAMPLE_NUM / 2U) / ADC_SAMPLE_NUM);
    ADC.Ib1_Off = (uint16_t)((Ib1_Sum + ADC_SAMPLE_NUM / 2U) / ADC_SAMPLE_NUM);
    ADC.Ic_Off = (uint16_t)((Ic_Sum + ADC_SAMPLE_NUM / 2U) / ADC_SAMPLE_NUM);
}

void Fast_Loop(void)
{
    Motor_Fast_Mode_e Fast_Mode;
    bool Current_OK;
    bool Encoder_Park_Valid;
    float Theta_e;
    float Encoder_Theta_e;
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
    uint16_t Raw1;
    uint16_t Raw2;
    ADC_Sample_e Next_Sample;
    uint32_t T0;
    uint32_t Cyc;

    T0 = DWT->CYCCNT;

    /*
     * Both current samples use injected Rank1. ADC2 Rank2 Vbus may be one
     * trigger old, as in the previous fixed-AB implementation.
     */
    Raw1 = (uint16_t)ADC1->JDR1;
    Raw2 = (uint16_t)ADC2->JDR1;
    ADC.Vbus_Raw = (uint16_t)ADC2->JDR2;

    ADC.Ia_Raw = 0U;
    ADC.Ib_Raw = 0U;
    ADC.Ic_Raw = 0U;

    switch (ADC_Sample)
    {
        case ADC_SAMPLE_BC:
            ADC.Ib_Raw = Raw1;
            ADC.Ic_Raw = Raw2;
            ADC.Ib_A = ((float)ADC.Ib1_Off - (float)Raw1) * CUR_RAW_TO_A;
            ADC.Ic_A = ((float)ADC.Ic_Off - (float)Raw2) * CUR_RAW_TO_A;
            ADC.Ia_A = -(ADC.Ib_A + ADC.Ic_A);
            break;

        case ADC_SAMPLE_AC:
            ADC.Ia_Raw = Raw1;
            ADC.Ic_Raw = Raw2;
            ADC.Ia_A = ((float)ADC.Ia_Off - (float)Raw1) * CUR_RAW_TO_A;
            ADC.Ic_A = ((float)ADC.Ic_Off - (float)Raw2) * CUR_RAW_TO_A;
            ADC.Ib_A = -(ADC.Ia_A + ADC.Ic_A);
            break;

        case ADC_SAMPLE_AB:
        default:
            ADC.Ia_Raw = Raw1;
            ADC.Ib_Raw = Raw2;
            ADC.Ia_A = ((float)ADC.Ia_Off - (float)Raw1) * CUR_RAW_TO_A;
            ADC.Ib_A = ((float)ADC.Ib_Off - (float)Raw2) * CUR_RAW_TO_A;
            ADC.Ic_A = -(ADC.Ia_A + ADC.Ib_A);
            break;
    }

    ADC.Vbus_V = (float)ADC.Vbus_Raw * VBUS_RAW_TO_V;

    Ialpha = ADC.Ia_A;
    Ibeta = (ADC.Ia_A + 2.0f * ADC.Ib_A) * INV_SQRT3_F;
    Encoder_Park_Valid = false;
    Encoder_Theta_e = 0.0f;
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
            Encoder_Theta_e = Angle_Wrap((float)Motor_Para.Pp * Theta_m + Motor_Cal.Theta_Off);
            SinCos(Encoder_Theta_e, &Sin, &Cos);
            Iq_Encoder = -Ialpha * Sin + Ibeta * Cos;
            Encoder_Park_Valid = true;
        }

        Mechanical_ESO_Run(Theta_m, Encoder.Valid != 0U, Iq_Encoder);
    }

    if (!Current_OK)
    {
        Signal_Injection_Stop();
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
        goto finish;
    }

    Fast_Mode = Motor_Fast_Run(Encoder_Theta_e, Encoder_Park_Valid,
                               &Theta_e, &Id_Ref, &Iq_Ref, &Ualpha, &Ubeta);
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

        /* Current FRF is injected into the actual PI reference before limits. */
        if ((Signal_Injection.State.Active != 0U) &&
            (Signal_Injection.State.Target != SIGNAL_SPEED))
        {
            if ((Motor_Mode == TORQUE) && (Motor_State_Get() == ENABLED))
            {
                if (Signal_Injection.State.Target == SIGNAL_ID)
                {
                    Id_Ref += Signal_Injection_Run();
                }
                else
                {
                    Iq_Ref += Signal_Injection_Run();
                }
            }
            else
            {
                Signal_Injection_Stop();
            }
        }

        /* One command-current limit for every current-controlled mode, including
         * phase search, I/F and the final injected d-axis reference. */
        Vector2_Limit(&Id_Ref, &Iq_Ref, Motor_I_Limit_Effective_Get());
        Motor_Plot_Iq_Ref = Iq_Ref;

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

    /* Skip the phase with the shortest low-side conduction window. */
    if ((DutyA >= DutyB) && (DutyA >= DutyC))
    {
        Next_Sample = ADC_SAMPLE_BC;
    }
    else if (DutyB >= DutyC)
    {
        Next_Sample = ADC_SAMPLE_AC;
    }
    else
    {
        Next_Sample = ADC_SAMPLE_AB;
    }

    if (!ADC_Phase_Set(Next_Sample))
    {
        goto finish;
    }

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
