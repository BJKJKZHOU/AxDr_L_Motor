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

#ifdef MATH_TIME
#include "Math_Time.h"

volatile Math_Time_T Math_Time[MATH_COUNT] = { 0 };
volatile uint32_t Math_Run = 0U;
#endif

#ifdef DUAL_LIMIT_TEST
/* 仅供空载小电流实验：0.5 s 基线、0.1~10 s 双限幅、0.5 s 恢复。
 * Run 由主机在位置环 RUN 后置 1；2 为结束，3 为异常，均锁住 PWM。
 * 重新置 0/清空统计前，主机必须先禁能。测试不写入 NVS。 */
typedef struct
{
    uint32_t Seq;
    uint32_t Phase;
    uint32_t Flags;             /* bit 0/1: I/U 缩放；bit 2/3: d/q 积分回退。 */
    uint32_t Tick;
    float Id_In, Iq_In, Id_Ref, Iq_Ref;
    float Ud_In, Uq_In, Ud, Uq;
    float Id, Iq, Wm, Vbus, I_Lim, U_Lim;
    float Id_Int_Pre, Iq_Int_Pre, Id_Int, Iq_Int;
    float Ud_PI, Uq_PI, Ud_FF, Uq_FF;
} Limit_Sample_T;

typedef struct
{
    uint32_t Count, I, U, Both, Aw_D, Aw_Q, Both_FF;
    uint32_t Late, Min, Max, Over, Sum_Lo, Sum_Hi;
    uint32_t Hist[64];          /* 只统计本阶段同周期双缩放；43 个 TIM1 tick/桶。 */
    float I2_Max;              /* 全阶段实际 dq 电流矢量平方峰值，单位 A²。 */
} Limit_Stat_T;

volatile uint32_t Limit_Run = 0U;
volatile uint32_t Limit_N = 0U;
volatile uint32_t Limit_Excite_N = 200000U;
volatile uint32_t Limit_Trace_N = 0U;
volatile Limit_Stat_T Limit_Stat[3] = { 0 };
volatile Limit_Sample_T Limit_Trace[256] = { 0 };
/* 保留关断原因及此前 16 个真实周期，避免低速抽样漏掉触发瞬间。 */
volatile uint32_t Limit_Trip_Code = 0U;
volatile float Limit_Trip_Fbk[5] = { 0 };
volatile Limit_Sample_T Limit_Tail[16] = { 0 };
#endif

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
    uint32_t T0;
    uint32_t Cyc;
    uint32_t Stage_T0;
#ifdef DUAL_LIMIT_TEST
    bool Limit_Active = false;
    Limit_Sample_T Sample;
#endif
#ifdef MATH_TIME
    uint32_t Math_T0;
    float Math_Theta;
    float Math_X;
    float Math_Y;
    float Math_Lim;

    for (uint32_t n = 0U; n < MATH_COUNT; n++)
    {
        Math_Time[n].Cyc = 0U;
        Math_Time[n].Alt = 0U;
    }
    Math_T0 = Math_Clock();
    Math_Time[MATH_EMPTY].Cyc = Math_Clock() - Math_T0;
#endif

    T0 = DWT->CYCCNT;
    Fast_Time.ESO_Cyc = 0U;
    Fast_Time.PI_Cyc = 0U;
    Fast_Time.SVPWM_Cyc = 0U;
    Fast_Time.ADC_CCR_Cyc = 0U;
    Fast_Time.ARR_CCR_Tick = UINT32_MAX;

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
#ifdef MATH_TIME
            Math_Theta = (float)Motor_Para.Pp * Theta_m + Motor_Cal.Theta_Off;
            Math_T0 = Math_Clock();
            Encoder_Theta_e = Angle_Wrap(Math_Theta);
            Math_Time[MATH_WRAP_E].Cyc = Math_Clock() - Math_T0;
            Math_Time[MATH_WRAP_E].Alt = __builtin_isfinite(Math_Theta) &&
                ((Math_Theta >= 2.0f * TWO_PI_F) || (Math_Theta < -TWO_PI_F));
            Math_T0 = Math_Clock();
#else
            Encoder_Theta_e = Angle_Wrap((float)Motor_Para.Pp * Theta_m + Motor_Cal.Theta_Off);
#endif
            SinCos(Encoder_Theta_e, &Sin, &Cos);
#ifdef MATH_TIME
            Math_Time[MATH_SINCOS].Cyc = Math_Clock() - Math_T0;
#endif
            Iq_Encoder = -Ialpha * Sin + Ibeta * Cos;
            Encoder_Park_Valid = true;
        }

        Stage_T0 = DWT->CYCCNT;
        Mechanical_ESO_Run(Theta_m, Encoder.Valid != 0U, Iq_Encoder);
        Cyc = DWT->CYCCNT - Stage_T0;
        Fast_Time.ESO_Cyc = Cyc;
#ifdef MATH_TIME
        Math_Time[MATH_ESO].Cyc = Cyc;
#endif
        if (Cyc > Fast_Time.ESO_Max)
        {
            Fast_Time.ESO_Max = Cyc;
        }
    }

#ifdef DUAL_LIMIT_TEST
    if (Limit_Run != 0U)
    {
        /* 使用实际反馈，检查每一个快环；不依赖 USB 主机退出的延迟。
         * 0.6 A 是反馈矢量关断阈值，参考仍受原有 <=0.3 A 限幅约束。 */
        if ((Limit_Run == 1U) &&
            (!Current_OK || !Encoder_Park_Valid ||
             (Motor_Mode_Get() != POSITION) || (Motor_State_Get() != RUN) ||
             (Current_FF_Enable == 0U) ||
             !(ADC.Vbus_V >= 20.0f && ADC.Vbus_V <= 27.0f) ||
             !(Ialpha * Ialpha + Ibeta * Ibeta <= 0.36f) ||
             !(Mechanical_ESO.State.Wm >= -5.0f && Mechanical_ESO.State.Wm <= 5.0f) ||
             !(Motor_I_Limit_Effective_Get() > 0.0f && Motor_I_Limit_Effective_Get() <= 0.3f) ||
             (Limit_Excite_N < 2000U) || (Limit_Excite_N > 200000U) ||
             (Id_Ctrl.Para.Kd != 0.0f) || (Iq_Ctrl.Para.Kd != 0.0f)))
        {
            PWM_Disable();
            Limit_Trip_Fbk[0] = Ialpha;
            Limit_Trip_Fbk[1] = Ibeta;
            Limit_Trip_Fbk[2] = Mechanical_ESO.State.Wm;
            Limit_Trip_Fbk[3] = ADC.Vbus_V;
            Limit_Trip_Fbk[4] = Motor_I_Limit_Effective_Get();
            /* bit 0..10 对应上述检查；重建只在已经关断后执行。 */
            Limit_Trip_Code = (uint32_t)!Current_OK |
                ((uint32_t)!Encoder_Park_Valid << 1) |
                ((uint32_t)(Motor_Mode_Get() != POSITION) << 2) |
                ((uint32_t)(Motor_State_Get() != RUN) << 3) |
                ((uint32_t)(Current_FF_Enable == 0U) << 4) |
                ((uint32_t)!(ADC.Vbus_V >= 20.0f && ADC.Vbus_V <= 27.0f) << 5) |
                ((uint32_t)!(Ialpha * Ialpha + Ibeta * Ibeta <= 0.36f) << 6) |
                ((uint32_t)!(Mechanical_ESO.State.Wm >= -5.0f && Mechanical_ESO.State.Wm <= 5.0f) << 7) |
                ((uint32_t)!(Motor_I_Limit_Effective_Get() > 0.0f && Motor_I_Limit_Effective_Get() <= 0.3f) << 8) |
                ((uint32_t)((Limit_Excite_N < 2000U) || (Limit_Excite_N > 200000U)) << 9) |
                ((uint32_t)((Id_Ctrl.Para.Kd != 0.0f) || (Iq_Ctrl.Para.Kd != 0.0f)) << 10);
            Limit_Run = 3U;
        }
        if ((Limit_Run == 1U) && (Limit_N >= Limit_Excite_N + 20000U))
        {
            Limit_Run = 2U;
        }
        if (Limit_Run != 1U)
        {
            PWM_Disable();
            goto finish;
        }
        Limit_Active = true;
        Sample.Seq = Limit_N;
        Sample.Phase = (Limit_N < 10000U) ? 0U : ((Limit_N < Limit_Excite_N + 10000U) ? 1U : 2U);
    }
#endif

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
#ifdef DUAL_LIMIT_TEST
        if (Limit_Active)
        {
            /* 1 kHz 对称 d 轴方波；缩放发生在真实 PI 参考入口。 */
            if (Sample.Phase == 1U)
            {
                Id_Ref += ((Sample.Seq % 20U) < 10U) ? 0.36f : -0.36f;
            }
            Sample.Id_In = Id_Ref;
            Sample.Iq_In = Iq_Ref;
            Sample.I_Lim = Motor_I_Limit_Effective_Get();
        }
#endif
#ifdef MATH_TIME
        Math_X = Id_Ref;
        Math_Y = Iq_Ref;
        Math_Lim = Motor_I_Limit_Effective_Get();
        Math_T0 = Math_Clock();
        Vector2_Limit(&Id_Ref, &Iq_Ref, Math_Lim);
        Math_Time[MATH_LIM_I].Cyc = Math_Clock() - Math_T0;
        Math_Time[MATH_LIM_I].Alt = (Math_Lim > 0.0f) &&
            ((Id_Ref != Math_X) || (Iq_Ref != Math_Y));
#else
        Vector2_Limit(&Id_Ref, &Iq_Ref, Motor_I_Limit_Effective_Get());
#endif
        Motor_Plot_Iq_Ref = Iq_Ref;

        U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
#ifdef DUAL_LIMIT_TEST
        if (Limit_Active)
        {
            /* 仅降低实验电压上限，不放大 PI 增益或修改电流反馈。 */
            if ((Sample.Phase == 1U) && (U_Lim > 0.15f))
            {
                U_Lim = 0.15f;
            }
            Sample.Id_Ref = Id_Ref;
            Sample.Iq_Ref = Iq_Ref;
            Sample.Id = Motor_Run.Id;
            Sample.Iq = Motor_Run.Iq;
            Sample.Wm = Mechanical_ESO.State.Wm;
            Sample.Vbus = ADC.Vbus_V;
            Sample.U_Lim = U_Lim;
            Sample.Id_Int_Pre = Id_Ctrl.State.Int;
            Sample.Iq_Int_Pre = Iq_Ctrl.State.Int;
        }
#endif
        Stage_T0 = DWT->CYCCNT;
        Current_Loop(Id_Ref, Iq_Ref, U_Lim, &Ud, &Uq);
        Cyc = DWT->CYCCNT - Stage_T0;
        Fast_Time.PI_Cyc = Cyc;
        if (Cyc > Fast_Time.PI_Max)
        {
            Fast_Time.PI_Max = Cyc;
        }

        /* Encoder servo dq decoupling and back-EMF feedforward. */
#ifdef DUAL_LIMIT_TEST
        if (Limit_Active)
        {
            Sample.Ud_PI = Ud;
            Sample.Uq_PI = Uq;
        }
#endif
        if ((Current_FF_Enable != 0U) && (Motor_Mode <= POSITION) && Encoder_Park_Valid)
        {
            float We = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;

            Ud -= We * Motor_Para.Lq * Motor_Run.Iq;
            Uq += We * (Motor_Para.Ld * Motor_Run.Id + Motor_Para.Flux);
        }

#ifdef MATH_TIME
        Math_X = Ud;
        Math_Y = Uq;
        Math_T0 = Math_Clock();
#endif
#ifdef DUAL_LIMIT_TEST
        if (Limit_Active)
        {
            Sample.Ud_In = Ud;
            Sample.Uq_In = Uq;
        }
#endif
        Vector2_Limit(&Ud, &Uq, U_Lim);
#ifdef MATH_TIME
        Math_Time[MATH_LIM_U].Cyc = Math_Clock() - Math_T0;
        Math_Time[MATH_LIM_U].Alt = (U_Lim > 0.0f) &&
            ((Ud != Math_X) || (Uq != Math_Y));
#endif

        Motor_Run.Ud = Ud;
        Motor_Run.Uq = Uq;
        Ualpha = Ud * Cos - Uq * Sin;
        Ubeta = Ud * Sin + Uq * Cos;
    }

    Motor_Run.Ualpha = Ualpha;
    Motor_Run.Ubeta = Ubeta;

    Stage_T0 = DWT->CYCCNT;
    SVPWM_Calc(Ualpha, Ubeta, ADC.Vbus_V, &DutyA, &DutyB, &DutyC);
    Cyc = DWT->CYCCNT - Stage_T0;
    Fast_Time.SVPWM_Cyc = Cyc;
#ifdef MATH_TIME
    Math_Time[MATH_SVPWM].Cyc = Cyc;
#endif
    if (Cyc > Fast_Time.SVPWM_Max)
    {
        Fast_Time.SVPWM_Max = Cyc;
    }

    PWM_Update(DutyA, DutyB, DutyC);

#ifdef DUAL_LIMIT_TEST
    if (Limit_Active)
    {
        /* 所有比较、回退条件重建及统计均在 CCR 写入之后。
         * 前面的现场保存仍有开销，结果属于带采集的完整快环。 */
        volatile Limit_Stat_T *Stat = &Limit_Stat[Sample.Phase];
        float Int_D = Sample.Id_Int_Pre + Id_Ctrl.Para.Ki * Id_Ctrl.Sig.Err * CUR_TS;
        float Int_Q = Sample.Iq_Int_Pre + Iq_Ctrl.Para.Ki * Iq_Ctrl.Sig.Err * CUR_TS;
        float Out_D;
        float Out_Q;
        float I2 = Sample.Id * Sample.Id + Sample.Iq * Sample.Iq;
        bool I_Scale;
        bool U_Scale;
        bool Aw_D;
        bool Aw_Q;

        Limit_Value(&Int_D, -Sample.U_Lim, Sample.U_Lim);
        Limit_Value(&Int_Q, -Sample.U_Lim, Sample.U_Lim);
        Out_D = Id_Ctrl.Para.Kp * Id_Ctrl.Sig.Err + Int_D;
        Out_Q = Iq_Ctrl.Para.Kp * Iq_Ctrl.Sig.Err + Int_Q;
        Sample.Id_Int = Id_Ctrl.State.Int;
        Sample.Iq_Int = Iq_Ctrl.State.Int;
        Sample.Ud = Ud;
        Sample.Uq = Uq;
        Sample.Ud_FF = Sample.Ud_In - Sample.Ud_PI;
        Sample.Uq_FF = Sample.Uq_In - Sample.Uq_PI;
        Sample.Tick = Fast_Time.ARR_CCR_Tick;
        I_Scale = (Sample.Id_In != Sample.Id_Ref) || (Sample.Iq_In != Sample.Iq_Ref);
        U_Scale = (Sample.Ud_In != Ud) || (Sample.Uq_In != Uq);
        Aw_D = (((Out_D > Sample.U_Lim) && (Int_D > Sample.Id_Int_Pre)) ||
                ((Out_D < -Sample.U_Lim) && (Int_D < Sample.Id_Int_Pre))) &&
               (Sample.Id_Int == Sample.Id_Int_Pre);
        Aw_Q = (((Out_Q > Sample.U_Lim) && (Int_Q > Sample.Iq_Int_Pre)) ||
                ((Out_Q < -Sample.U_Lim) && (Int_Q < Sample.Iq_Int_Pre))) &&
               (Sample.Iq_Int == Sample.Iq_Int_Pre);
        Sample.Flags = (uint32_t)I_Scale | ((uint32_t)U_Scale << 1) |
                       ((uint32_t)Aw_D << 2) | ((uint32_t)Aw_Q << 3);
        Stat->Count++;
        if (I2 > Stat->I2_Max)
        {
            Stat->I2_Max = I2;
        }
        Stat->I += I_Scale;
        Stat->U += U_Scale;
        Stat->Aw_D += Aw_D;
        Stat->Aw_Q += Aw_Q;
        if (I_Scale && U_Scale)
        {
            Stat->Both++;
            Stat->Both_FF += (Sample.Ud_FF != 0.0f) && (Sample.Uq_FF != 0.0f);
            if (Sample.Tick == UINT32_MAX)
            {
                Stat->Late++;
            }
            else
            {
                uint32_t Bin = Sample.Tick / 43U;
                uint32_t Sum = Stat->Sum_Lo + Sample.Tick;
                Stat->Hist[(Bin < 64U) ? Bin : 63U]++;
                Stat->Over += Sample.Tick > 2125U;
                if ((Stat->Min == 0U) || (Sample.Tick < Stat->Min))
                {
                    Stat->Min = Sample.Tick;
                }
                if (Sample.Tick > Stat->Max)
                {
                    Stat->Max = Sample.Tick;
                }
                if (Sum < Stat->Sum_Lo)
                {
                    Stat->Sum_Hi++;
                }
                Stat->Sum_Lo = Sum;
            }
        }
        /* 1009 与 20 点激励周期互素，避免抽样固定在方波同一相位。 */
        if ((Limit_Trace_N < 256U) &&
            ((Sample.Seq % 1009U == 0U) || (Sample.Seq == 10000U) ||
             (Sample.Seq == Limit_Excite_N + 10000U) || (I_Scale && U_Scale && Stat->Both == 1U)))
        {
            Limit_Trace[Limit_Trace_N++] = Sample;
        }
        Limit_Tail[Sample.Seq % 16U] = Sample;
        Limit_N++;
    }
#endif

finish:
#ifdef MATH_TIME
    /* Aggregate after CCR writes. Cyc is zero for paths not executed this ISR.
     * The timestamps and nested ESO/PID probes still perturb total latency. */
    if (Math_Run != 0U)
    {
        for (uint32_t n = 0U; n < MATH_COUNT; n++)
        {
            uint32_t Sample = Math_Time[n].Cyc;
            if (Sample != 0U)
            {
                uint32_t Sum = Math_Time[n].Sum_Lo + Sample;
                if (Sum < Math_Time[n].Sum_Lo)
                {
                    Math_Time[n].Sum_Hi++;
                }
                Math_Time[n].Sum_Lo = Sum;
                if (Sample < Math_Time[n].Min)
                {
                    Math_Time[n].Min = Sample;
                }
                if (Sample > Math_Time[n].Max)
                {
                    Math_Time[n].Max = Sample;
                }
                Math_Time[n].Count++;
                Math_Time[n].Alt_Count += Math_Time[n].Alt;
            }
        }
    }
#endif
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
