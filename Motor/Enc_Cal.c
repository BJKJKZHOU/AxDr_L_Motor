/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Enc_Cal.h"

#include "Encoder.h"
#include "Current_Loop.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Cal.h"
#include "Motor_Control.h"
#include "motor_work.h"
#include "Motor_Para.h"
#include "Motion_Type.h"
#include "Servo_Phase.h"
#include "Sin_LUT.h"
#include "VF.h"
#include "control_params.h"
#include "main.h"

#define ENC_CAL_VF_SETTLE_CNT  ((uint32_t)(2.0f / SPD_TS + 0.5f))
#define ENC_CAL_SPEED_HOLD_CNT ((uint32_t)(0.5f / SPD_TS + 0.5f))
#define ENC_CAL_SAMPLE_DIV     10U
#define ENC_CAL_SAMPLE_TS      (CUR_TS * (float)ENC_CAL_SAMPLE_DIV)
#define ENC_CAL_SAMPLES        ((uint32_t)(6.0f / ENC_CAL_SAMPLE_TS + 0.5f))
#define ENC_CAL_COEF_MAX       0.05f
#define ENC_CAL_STOP_WAIT_CNT  ((uint32_t)(0.3f / SPD_TS + 0.5f))
#define ENC_CAL_STOP_WM        (10.0f * TWO_PI_F / 60.0f)
/* Mechanical angle RMS after removing offset, speed and 1x/2x.
 * The accepted VF records were 0.06..0.08 degrees; allow 0.25 degrees. */
#define ENC_CAL_RMS_MAX        (0.25f * PI_F / 180.0f)
#define ENC_CAL_FIT_N          6U

volatile Enc_Cal_T Enc_Cal = { 0 };
static volatile uint32_t Sample_Cnt = 0U;
static uint32_t Tick = 0U;
static uint32_t Hold_Cnt = 0U;
static volatile bool VF_Ready = false;
volatile bool Enc_Cal_Handoff_Ready = false;
static VF_T Cal_VF = { 0 };

static uint8_t Div = 0U;
static float Theta_Pre = 0.0f;
static float Theta_Err = 0.0f;
static float A[ENC_CAL_FIT_N][ENC_CAL_FIT_N] = { 0 };
static float B[ENC_CAL_FIT_N] = { 0 };
static float Y2 = 0.0f;

static FAST_CODE void Enc_Cal_Sample(float Theta_Native);

static void Enc_Cal_Finish(bool Passed)
{
    if (!Passed && Servo_Phase_Active())
    {
        Servo_Phase_Abort();
    }
    if (Motor_State_Get() != DISABLED)
    {
        Motor_Disable();
    }

    Enc_Cal.Step = Passed ? CAL_DONE : CAL_FAILED;
}

bool Enc_Cal_Start(void)
{
    if (((Enc_Cal.Step >= CAL_SPEED) && (Enc_Cal.Step <= CAL_PHASE)) ||
        (Motor_State_Get() != DISABLED) ||
        (Motor_Wm_Limit_Effective_Get() < ENC_CAL_WM) ||
        (Motion_Config.Wm_Acc <= 0.0f) || (Motion_Config.Wm_Dec <= 0.0f) ||
        (Motor_Para.Flux <= 0.0f))
    {
        return false;
    }

    Tick = 0U;
    Hold_Cnt = 0U;
    VF_Ready = false;
    Enc_Cal_Handoff_Ready = false;
    Cal_VF = (VF_T){ 0 };
    Div = 0U;
    Sample_Cnt = 0U;
    Theta_Pre = 0.0f;
    Theta_Err = 0.0f;
    Y2 = 0.0f;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] = 0.0f;
        for (uint32_t j = 0U; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] = 0.0f;
        }
    }

    Enc_Cal.Step = CAL_SPEED;
    Motor_Enable();
    if (Motor_State_Get() == ENABLED)
    {
        Motor_Start();
    }

    if (Motor_State_Get() != RUN)
    {
        Enc_Cal_Finish(false);
        return false;
    }

    return true;
}

Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                   float *Id_Ref,
                                   float *Iq_Ref,
                                   float *Ualpha,
                                   float *Ubeta)
{
    if (Enc_Cal.Step == CAL_PHASE)
    {
        return Servo_Phase_Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);
    }

    if ((Enc_Cal.Step != CAL_VF) && (Enc_Cal.Step != CAL_SAMPLE) &&
        (Enc_Cal.Step != CAL_HANDOFF) && (Enc_Cal.Step != CAL_DECEL))
    {
        return FAST_OFF;
    }

    if (!VF_Ready)
    {
        /* The stored voltage was applied last cycle. Advance its phase once
         * before VF_Run emits this cycle, preserving the rotating vector. */
        float Ua = Motor_Run.Ualpha;
        float Ub = Motor_Run.Ubeta;

        Cal_VF.State.U = __builtin_sqrtf(Ua * Ua + Ub * Ub);
        Cal_VF.State.We = (float)Motor_Para.Pp * ENC_CAL_WM;
        Cal_VF.State.Theta_e = Angle_Wrap(__builtin_atan2f(-Ua, Ub) +
                                         Cal_VF.State.We * CUR_TS);
        Cal_VF.Para.U_Slew_V_S = 30.0f;
        Cal_VF.Para.We_Acc = (float)Motor_Para.Pp * Motion_Config.Wm_Acc;
        VF_Ready = true;
    }

    /* Constant-speed calibration holds the measured operating voltage. Encoder
     * feedback never adjusts VF angle or amplitude after the transfer. */
    Cal_VF.Para.U_Max_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    *Theta_e = Cal_VF.State.Theta_e;
    VF_Run(&Cal_VF, (float)Motor_Para.Pp * ENC_CAL_WM,
           Cal_VF.State.U, Ualpha, Ubeta, CUR_TS);

    if ((Enc_Cal.Step == CAL_HANDOFF) || (Enc_Cal.Step == CAL_DECEL))
    {
        float Sin;
        float Cos;
        float Ialpha = ADC.Ia_A;
        float Ibeta = (ADC.Ia_A + 2.0f * ADC.Ib_A) * INV_SQRT3_F;
        float Theta = Angle_Wrap((float)Motor_Para.Pp * Encoder.Theta_m +
                                 Motor_Cal.Theta_Off);
        float Id;
        float Iq;

        SinCos(Theta, &Sin, &Cos);
        Id = Ialpha * Cos + Ibeta * Sin;
        Iq = -Ialpha * Sin + Ibeta * Cos;
        Motor_Run.Id = Id;
        Motor_Run.Iq = Iq;

        if (Enc_Cal.Step == CAL_HANDOFF)
        {
            /* Keep rotating while the 2 kHz owner prepares the speed ramp. */
            Enc_Cal_Handoff_Ready = true;
            return FAST_VOLTAGE;
        }

        /* Seed FOC in the first DECEL ADC cycle, using this cycle's VF voltage
         * and measured current rather than a snapshot held by the slow task. */
        float We = (float)Motor_Para.Pp * Mechanical_ESO_Wm_Get();
        float Ud = *Ualpha * Cos + *Ubeta * Sin;
        float Uq = -*Ualpha * Sin + *Ubeta * Cos;
        float Ud_FF = 0.0f;
        float Uq_FF = 0.0f;
        if (Current_FF_Enable != 0U)
        {
            Ud_FF = -We * Motor_Para.Lq * Iq;
            Uq_FF = We * (Motor_Para.Ld * Id + Motor_Para.Flux);
        }

        Motor_Run.Ud = Ud;
        Motor_Run.Uq = Uq;
        Current_Loop_State_Reset();
        Current_Loop_Track(Id, Iq, Ud - Ud_FF, Uq - Uq_FF, Ud_FF, Uq_FF);
        *Theta_e = Theta;
        *Id_Ref = Id;
        *Iq_Ref = Iq;
        Enc_Cal_Handoff_Ready = false;
        return FAST_CURRENT;
    }

    Enc_Cal_Sample(Encoder.Theta_Native);
    /* An incomplete/invalid capture cannot become a successful fit. */
    return (Sample_Cnt > ENC_CAL_SAMPLES) ? FAST_OFF : FAST_VOLTAGE;
}

static FAST_CODE void Enc_Cal_Sample(float Theta_Native)
{
    float X[ENC_CAL_FIT_N];
    float Delta;
    float Time;
    float Y;
    float Sin1;
    float Cos1;
    float Sin2;
    float Cos2;

    if ((Enc_Cal.Step != CAL_SAMPLE) || (Sample_Cnt >= ENC_CAL_SAMPLES))
    {
        return;
    }

    if ((Encoder.Valid == 0U) || (Encoder.Fault != 0U) ||
        !__builtin_isfinite(Theta_Native))
    {
        /* Preserve rejection even if the encoder recovers before the next
         * slow tick. A count above the requested length is never fitted. */
        Sample_Cnt = ENC_CAL_SAMPLES + 1U;
        return;
    }

    if (++Div < ENC_CAL_SAMPLE_DIV)
    {
        return;
    }
    Div = 0U;

    if (Sample_Cnt == 0U)
    {
        Theta_Pre = Theta_Native;
        Theta_Err = 0.0f;
    }
    else
    {
        Delta = Theta_Native - Theta_Pre;
        if (Delta > PI_F)
        {
            Delta -= TWO_PI_F;
        }
        else if (Delta < -PI_F)
        {
            Delta += TWO_PI_F;
        }
        /* Accumulate the small deviation, not ~600 rad of single-precision
         * angle followed by cancellation against the nominal trajectory. */
        Theta_Err += Delta - (float)Motor_Cal.Enc_Dir * ENC_CAL_WM * ENC_CAL_SAMPLE_TS;
        Theta_Pre = Theta_Native;
    }

    Time = (float)Sample_Cnt * ENC_CAL_SAMPLE_TS;
    Y = Theta_Err;
    SinCos(Theta_Native, &Sin1, &Cos1);
    SinCos(Angle_Wrap(2.0f * Theta_Native), &Sin2, &Cos2);

    X[0] = 1.0f;
    X[1] = Time - 3.0f;
    X[2] = Cos1;
    X[3] = Sin1;
    X[4] = Cos2;
    X[5] = Sin2;

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] += X[i] * Y;
        for (uint32_t j = i; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] += X[i] * X[j];
        }
    }
    Y2 += Y * Y;
    Sample_Cnt++;
}

/* Small, offline 6x6 least-squares solve; never called from the fast ISR. */
static bool Enc_Cal_Fit(float C[4])
{
    float M[ENC_CAL_FIT_N][ENC_CAL_FIT_N + 1U];
    float Beta[ENC_CAL_FIT_N];

    if (Sample_Cnt != ENC_CAL_SAMPLES)
    {
        return false;
    }

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        for (uint32_t j = 0U; j < ENC_CAL_FIT_N; j++)
        {
            M[i][j] = (j >= i) ? A[i][j] : A[j][i];
        }
        M[i][ENC_CAL_FIT_N] = B[i];
    }

    for (uint32_t k = 0U; k < ENC_CAL_FIT_N; k++)
    {
        uint32_t Pivot = k;
        float Max = __builtin_fabsf(M[k][k]);
        for (uint32_t i = k + 1U; i < ENC_CAL_FIT_N; i++)
        {
            float V = __builtin_fabsf(M[i][k]);
            if (V > Max)
            {
                Max = V;
                Pivot = i;
            }
        }
        if (!__builtin_isfinite(Max) || (Max < 0.0001f * (float)ENC_CAL_SAMPLES))
        {
            return false;
        }
        if (Pivot != k)
        {
            for (uint32_t j = k; j <= ENC_CAL_FIT_N; j++)
            {
                float Tmp = M[k][j];
                M[k][j] = M[Pivot][j];
                M[Pivot][j] = Tmp;
            }
        }

        for (uint32_t i = k + 1U; i < ENC_CAL_FIT_N; i++)
        {
            float Scale = M[i][k] / M[k][k];
            for (uint32_t j = k + 1U; j <= ENC_CAL_FIT_N; j++)
            {
                M[i][j] -= Scale * M[k][j];
            }
        }
    }

    for (int32_t i = (int32_t)ENC_CAL_FIT_N - 1; i >= 0; i--)
    {
        float V = M[i][ENC_CAL_FIT_N];
        for (uint32_t j = (uint32_t)i + 1U; j < ENC_CAL_FIT_N; j++)
        {
            V -= M[i][j] * Beta[j];
        }
        Beta[i] = V / M[i][i];
        if (!__builtin_isfinite(Beta[i]))
        {
            return false;
        }
    }

    for (uint32_t i = 0U; i < 4U; i++)
    {
        C[i] = Beta[i + 2U];
        if (!__builtin_isfinite(C[i]) ||
            (__builtin_fabsf(C[i]) > ENC_CAL_COEF_MAX))
        {
            return false;
        }
    }

    /* Residual energy of the joint offset/slope/harmonic fit. Evaluate the
     * cancellation in double here, outside the fast ISR. */
    double Rss = (double)Y2;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        Rss -= 2.0 * (double)Beta[i] * (double)B[i];
        for (uint32_t j = 0U; j < ENC_CAL_FIT_N; j++)
        {
            float Aij = (j >= i) ? A[i][j] : A[j][i];
            Rss += (double)Beta[i] * (double)Aij * (double)Beta[j];
        }
    }
    double Rss_Max = (double)ENC_CAL_SAMPLES * ENC_CAL_RMS_MAX * ENC_CAL_RMS_MAX;
    if (!__builtin_isfinite(Rss) || (Rss > Rss_Max) || (Rss < -Rss_Max) ||
        (__builtin_fabsf(Beta[1]) > 0.1f * ENC_CAL_WM))
    {
        return false;
    }

    return true;
}

/* The low-priority Motor Work thread calls this only with PWM off.
 * Fit owns its coefficients; the encoder consumes the published correction. */
bool Enc_Cal_Fit_Apply(void)
{
    float Coeff[4];
    uint32_t Primask;

    if (!Enc_Cal_Fit(Coeff))
    {
        return false;
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    if ((Enc_Cal.Step != CAL_FIT_WAIT) || (Motor_State_Get() != RUN) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
    {
        __set_PRIMASK(Primask);
        return false;
    }
    Motor_Cal.Valid = 0U;
    Encoder_Harmonic.C1 = Coeff[0];
    Encoder_Harmonic.S1 = Coeff[1];
    Encoder_Harmonic.C2 = Coeff[2];
    Encoder_Harmonic.S2 = Coeff[3];
    __set_PRIMASK(Primask);
    return true;
}

void Enc_Cal_Run(void)
{
    float Wm;
 
    if ((Enc_Cal.Step == CAL_IDLE) || (Enc_Cal.Step >= CAL_DONE))
    {
        return;
    }

    /* A calibration stays RUN until its final disable, including zero-speed phasing. */
    if (Motor_State_Get() != RUN)
    {
        Enc_Cal_Finish(false);
        return;
    }

    if (Enc_Cal.Step == CAL_PHASE)
    {
        if (!Servo_Phase_Active())
        {
            /* Servo_Phase_Apply() deliberately writes only with PWM disabled. */
            Motor_Disable();
            bool Passed = Servo_Phase_Result_Valid() && Servo_Phase_Apply();
            if (!Passed)
            {
                Motor_Cal.Valid = 0U;
            }
            Enc_Cal_Finish(Passed);
        }
        return;
    }

    if (Enc_Cal.Step == CAL_SPEED)
    {
        Wm = Mechanical_ESO_Wm_Get();
        Tick++;
        if (__builtin_fabsf(Wm - ENC_CAL_WM) < 0.1f * ENC_CAL_WM)
        {
            Hold_Cnt++;
        }
        else
        {
            Hold_Cnt = 0U;
        }

        if (Hold_Cnt >= ENC_CAL_SPEED_HOLD_CNT)
        {
            Tick = 0U;
            Hold_Cnt = 0U;
            Enc_Cal.Step = CAL_VF;
        }
        else if (Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Acc + 4.0f) / SPD_TS))
        {
            Enc_Cal_Finish(false);
        }
        return;
    }

    if (Enc_Cal.Step == CAL_VF)
    {
        if (!VF_Ready)
        {
            return;
        }
        Wm = Mechanical_ESO_Wm_Get();
        if (__builtin_isfinite(Wm) &&
            (__builtin_fabsf(Wm - ENC_CAL_WM) < 0.1f * ENC_CAL_WM))
        {
            Hold_Cnt++;
        }
        else
        {
            Hold_Cnt = 0U;
        }
        if (++Tick < ENC_CAL_VF_SETTLE_CNT)
        {
            return;
        }

        if (Hold_Cnt < ENC_CAL_SPEED_HOLD_CNT)
        {
            Enc_Cal_Finish(false);
            return;
        }

        Div = 0U;
        Enc_Cal.Step = CAL_SAMPLE;
        return;
    }

    if (Enc_Cal.Step == CAL_SAMPLE)
    {
        Wm = Mechanical_ESO_Wm_Get();
        if ((Sample_Cnt > ENC_CAL_SAMPLES) || !__builtin_isfinite(Wm) ||
            (__builtin_fabsf(Wm - ENC_CAL_WM) >= 0.1f * ENC_CAL_WM))
        {
            Enc_Cal_Finish(false);
        }
        else if (Sample_Cnt == ENC_CAL_SAMPLES)
        {
            Enc_Cal.Step = CAL_HANDOFF;
        }
        return;
    }

    if (Enc_Cal.Step == CAL_HANDOFF)
    {
        if (Enc_Cal_Handoff_Ready)
        {
            Motor_Cal_Decel_Begin();
            Hold_Cnt = 0U;
            Tick = 0U;
            Enc_Cal.Step = CAL_DECEL;
        }
        return;
    }

    if (Enc_Cal.Step == CAL_DECEL)
    {
        Wm = Mechanical_ESO_Wm_Get();
        if (++Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Dec + 4.0f) / SPD_TS))
        {
            Enc_Cal_Finish(false);
            return;
        }
        if ((Motor_Wm_Ref_Get() != 0.0f) || !__builtin_isfinite(Wm) ||
            (__builtin_fabsf(Wm) > ENC_CAL_STOP_WM))
        {
            Hold_Cnt = 0U;
            return;
        }

        if (++Hold_Cnt < ENC_CAL_STOP_WAIT_CNT)
        {
            return;
        }

        Enc_Cal.Step = CAL_FIT;
        return;
    }

    if (Enc_Cal.Step == CAL_FIT)
    {
        if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
        {
            if (!Motor_Work_Request(MOTOR_WORK_FIT))
            {
                Enc_Cal_Finish(false);
                return;
            }
            Enc_Cal.Step = CAL_FIT_WAIT;
        }
        return;
    }

    if (Enc_Cal.Step == CAL_FIT_WAIT)
    {
        if (Motor_Work_State < MOTOR_WORK_DONE)
        {
            return;
        }
        bool Passed = (Motor_Work_State == MOTOR_WORK_DONE);
        Motor_Work_Ack();
        if (!Passed)
        {
            Enc_Cal_Finish(false);
            return;
        }

        Current_Loop_State_Reset();
        if (!Servo_Phase_Start())
        {
            Enc_Cal_Finish(false);
            return;
        }

        Enc_Cal.Step = CAL_PHASE;
    }
}
