/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_ADC.h"

#include <stdbool.h>

#include "Plot.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Enc_Cal.h"
#include "Enc_Control.h"
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
    bool Sensored;
    bool Cal_Handoff;
    float Theta_e;
    float Encoder_Theta_e;
    float Ialpha;
    float Ibeta;
    float Sin;
    float Cos;
    float Iq_Encoder;
    float Ud;
    float Uq;
    float Ud_FF;
    float Uq_FF;
    float We;
    uint8_t Controller;
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
        float Theta_m = Motor_Run.Theta_m;

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

    /* A latched encoder fault must also turn off an ENABLED servo hold.
     * The 2 kHz protection task performs the final Disable transition. */
    if ((Encoder.Fault != 0U) && (Motor_State_Get() != DISABLED) &&
        Motor_Encoder_Required())
    {
        PWM_Disable();
        Motor_Run.Ud = 0.0f;
        Motor_Run.Uq = 0.0f;
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
        Motor_Plot_Iq_Ref = 0.0f;
        goto finish;
    }

    /* The slow speed-loop handoff and the first FOC cycle both need fresh
     * encoder-frame currents, while VF still owns the output voltage. */
    Cal_Handoff = (Motor_State_Get() == RUN) && (Enc_Step == ENC_CAL_RUN) &&
                  ((Enc_Cal.Step == CAL_HANDOFF) ||
                   ((Enc_Cal.Step == CAL_DECEL) && Enc_Cal_Handoff_Ready));
    if (Cal_Handoff)
    {
        if (!Encoder_Park_Valid)
        {
            Encoder_Theta_e = Angle_Wrap((float)Motor_Para.Pp * Motor_Run.Theta_m + Motor_Cal.Theta_Off);
            SinCos(Encoder_Theta_e, &Sin, &Cos);
            Iq_Encoder = -Ialpha * Sin + Ibeta * Cos;
        }
        Motor_Run.Id = Ialpha * Cos + Ibeta * Sin;
        Motor_Run.Iq = Iq_Encoder;
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
        Sensored = ((Enc_Step == ENC_CAL_RUN) &&
                    ((Enc_Cal.Step == CAL_SPEED) || (Enc_Cal.Step == CAL_DECEL))) ||
                   (!Enc_Control_Active() && (Motor_Mode <= POSITION));
        Controller = Sensored ? Current_Controller : CURRENT_CTRL_PI;
        if ((Controller == CURRENT_CTRL_DPCC) && Enc_Control_Active())
        {
            Controller = CURRENT_CTRL_PI;
        }

        /* Motor_Fast_Run may use the previous-cycle dq values (IF/ident).
         * Publish this sample only after it has selected the FOC angle. */
        if (!Cal_Handoff)
        {
            if (!Encoder_Park_Valid || !Sensored)
            {
                SinCos(Theta_e, &Sin, &Cos);
                Motor_Run.Iq = -Ialpha * Sin + Ibeta * Cos;
            }
            else
            {
                Motor_Run.Iq = Iq_Encoder;
            }
            Motor_Run.Id = Ialpha * Cos + Ibeta * Sin;
        }
        Motor_Run.Theta_e = Theta_e;

        /* Motor_Signal_Start checks TORQUE/ENABLED before starting current FRF.
         * Stop, Disable and Run cancel it; only the per-sample target remains here. */
        if ((Signal_Injection.State.Active != 0U) &&
            (Signal_Injection.State.Target != SIGNAL_SPEED))
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

        /* One command-current limit for every current-controlled mode, including
         * phase search, I/F and the final injected d-axis reference. */
        Vector2_Limit(&Id_Ref, &Iq_Ref, Motor_I_Limit_Effective_Get());
        Motor_Plot_Iq_Ref = Iq_Ref;

        U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
        We = 0.0f;
        Ud_FF = 0.0f;
        Uq_FF = 0.0f;
        if (Sensored && Encoder_Park_Valid)
        {
            We = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;
            if (Current_FF_Enable != 0U)
            {
                Ud_FF = -We * Motor_Para.Lq * Motor_Run.Iq;
                Uq_FF = We * (Motor_Para.Ld * Motor_Run.Id + Motor_Para.Flux);
            }
        }

        if (Cal_Handoff)
        {
            /* Track this cycle's rotating VF vector with the selected PI/LADRC.
             * Park and model feedforward above are shared with normal FOC. */
            Motor_Run.Ud = Ualpha * Cos + Ubeta * Sin;
            Motor_Run.Uq = -Ualpha * Sin + Ubeta * Cos;
            Current_Loop_State_Reset();
            Current_Loop_Track(Controller, Motor_Run.Id, Motor_Run.Iq,
                               Motor_Run.Ud - Ud_FF, Motor_Run.Uq - Uq_FF, Ud_FF, Uq_FF);
            Enc_Cal_Handoff_Ready = false;
        }

        Current_Loop(Id_Ref, Iq_Ref, U_Lim, Controller, We, Sin, Cos,
                     Ud_FF, Uq_FF, &Ud, &Uq);
        /* One shared model feedforward and final voltage-vector limit. */
        Ud += Ud_FF;
        Uq += Uq_FF;

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

