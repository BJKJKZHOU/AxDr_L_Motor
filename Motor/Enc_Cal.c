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
#include "Motor_Para.h"
#include "Motor_PWM.h"
#include "Motion_Type.h"
#include "NVS_Storage.h"
#include "Parameter.h"
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
#define ENC_CAL_FIT_N          6U

volatile Enc_Cal_T Enc_Cal = { 0 };
static volatile uint32_t Sample_Cnt = 0U;
static uint8_t Mode_Pre = TORQUE;
static uint32_t Tick = 0U;
static uint32_t Hold_Cnt = 0U;
static volatile bool VF_Ready = false;
static VF_T Cal_VF = { 0 };

static uint8_t Div = 0U;
static float Theta_Pre = 0.0f;
static float Theta_Sum = 0.0f;
static float A[ENC_CAL_FIT_N][ENC_CAL_FIT_N] = { 0 };
static float B[ENC_CAL_FIT_N] = { 0 };

static FAST_CODE void Enc_Cal_Sample(float Theta_Native);

static void Enc_Cal_Finish(bool Passed)
{
    if (Motor_State_Get() != DISABLED)
    {
        Motor_Disable();
    }

    Motor_Mode = Mode_Pre;
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

    Mode_Pre = Motor_Mode;
    Tick = 0U;
    Hold_Cnt = 0U;
    VF_Ready = false;
    Cal_VF = (VF_T){ 0 };
    Div = 0U;
    Sample_Cnt = 0U;
    Theta_Pre = 0.0f;
    Theta_Sum = 0.0f;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] = 0.0f;
        for (uint32_t j = 0U; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] = 0.0f;
        }
    }

    Enc_Cal.Step = CAL_SPEED;
    Motor_Mode = ENC_CAL;
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
    float U_Target;

    if (Enc_Cal.Step == CAL_PHASE)
    {
        return Servo_Phase_Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);
    }

    if ((Enc_Cal.Step != CAL_VF) && (Enc_Cal.Step != CAL_SAMPLE))
    {
        return FAST_OFF;
    }

    if (!VF_Ready)
    {
        /* Carry forward the last applied alpha/beta voltage, not the encoder
         * d-axis angle. VF generates voltage on its signed q-axis. */
        float Ua = Motor_Run.Ualpha;
        float Ub = Motor_Run.Ubeta;
        float Dir = (ENC_CAL_WM < 0.0f) ? -1.0f : 1.0f;

        Cal_VF.State.U = __builtin_sqrtf(Ua * Ua + Ub * Ub);
        Cal_VF.State.Theta_e = Angle_Wrap(__builtin_atan2f(-Dir * Ua, Dir * Ub));
        Cal_VF.State.We = (float)Motor_Para.Pp * ENC_CAL_WM;
        Cal_VF.Para.U_Slew_V_S = 30.0f;
        Cal_VF.Para.We_Acc = (float)Motor_Para.Pp * Motion_Config.Wm_Acc;
        VF_Ready = true;
    }

    Cal_VF.Para.U_Max_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    U_Target = __builtin_fabsf(Cal_VF.State.We) * Motor_Para.Flux +
               Motor_Para.Rs * OPEN_IQ_A;

    *Theta_e = Cal_VF.State.Theta_e;
    VF_Run(&Cal_VF, (float)Motor_Para.Pp * ENC_CAL_WM,
           U_Target, Ualpha, Ubeta, CUR_TS);
    Enc_Cal_Sample(Encoder.Theta_Native);
    return FAST_VOLTAGE;
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

    if (++Div < ENC_CAL_SAMPLE_DIV)
    {
        return;
    }
    Div = 0U;

    if (Sample_Cnt == 0U)
    {
        Theta_Pre = Theta_Native;
        Theta_Sum = 0.0f;
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
        Theta_Sum += Delta;
        Theta_Pre = Theta_Native;
    }

    Time = (float)Sample_Cnt * ENC_CAL_SAMPLE_TS;
    Y = Theta_Sum - (float)Motor_Cal.Enc_Dir * ENC_CAL_WM * Time;
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
    Sample_Cnt++;
}

/* Small, offline 6x6 least-squares solve; never called from the fast ISR. */
static bool Enc_Cal_Fit(float C[4])
{
    float M[ENC_CAL_FIT_N][ENC_CAL_FIT_N + 1U];
    float Beta[ENC_CAL_FIT_N];

    if ((Sample_Cnt != ENC_CAL_SAMPLES) ||
        (__builtin_fabsf(Theta_Sum) < 10.0f * TWO_PI_F))
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

    return true;
}

void Enc_Cal_Run(void)
{
    float Wm;
    float Coeff[4];
    uint32_t Primask;
    const uint16_t Valid_Id = PARAM_CAL_VALID;

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
        if (++Tick < ENC_CAL_VF_SETTLE_CNT)
        {
            return;
        }

        Wm = Mechanical_ESO_Wm_Get();
        if (__builtin_fabsf(Wm - ENC_CAL_WM) >= 0.1f * ENC_CAL_WM)
        {
            Enc_Cal_Finish(false);
            return;
        }

        Div = 0U;
        Enc_Cal.Step = CAL_SAMPLE;
        return;
    }

    if ((Enc_Cal.Step == CAL_SAMPLE) && (Sample_Cnt == ENC_CAL_SAMPLES))
    {
        /* Re-enter sensored PI at the current VF operating point. */
        Motor_Cal_Decel_Begin();
        Hold_Cnt = 0U;
        Enc_Cal.Step = CAL_DECEL;
        return;
    }

    if (Enc_Cal.Step == CAL_DECEL)
    {
        if (Motor_Wm_Ref_Get() != 0.0f)
        {
            Hold_Cnt = 0U;
            return;
        }

        if (++Hold_Cnt < ENC_CAL_STOP_WAIT_CNT)
        {
            return;
        }

        /* The speed trajectory has reached zero and its dwell has elapsed.
         * Release PWM only for fitting and subsequent stationary phasing. */
        PWM_Disable();

        if (!Enc_Cal_Fit(Coeff))
        {
            Enc_Cal_Finish(false);
            return;
        }

        /* Invalidate persistent electrical zero before replacing harmonics. */
        Motor_Cal.Valid = 0U;
        if (NVS_Storage_Save_Ids(&Valid_Id, 1U) != 0)
        {
            Enc_Cal_Finish(false);
            return;
        }

        Primask = __get_PRIMASK();
        __disable_irq();
        Encoder_Harmonic.C1 = Coeff[0];
        Encoder_Harmonic.S1 = Coeff[1];
        Encoder_Harmonic.C2 = Coeff[2];
        Encoder_Harmonic.S2 = Coeff[3];
        __set_PRIMASK(Primask);

        Current_Loop_State_Reset();
        if (!Servo_Phase_Start())
        {
            Enc_Cal_Finish(false);
            return;
        }

        Enc_Cal.Step = CAL_PHASE;
        PWM_Enable();
    }
}
