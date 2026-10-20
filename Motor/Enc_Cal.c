/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Enc_Cal.h"

#include "Encoder.h"
#include "Math.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Cal.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motion_Type.h"
#include "NVS_Storage.h"
#include "Parameter.h"
#include "Servo_Phase.h"
#include "Sin_LUT.h"
#include "VF.h"
#include "control_params.h"
#include "main.h"

#define ENC_CAL_WM             (1000.0f * TWO_PI_F / 60.0f)
#define ENC_CAL_VF_SETTLE_CNT  ((uint32_t)(2.0f / SPD_TS + 0.5f))
#define ENC_CAL_SPEED_HOLD_CNT ((uint32_t)(0.5f / SPD_TS + 0.5f))
#define ENC_CAL_SAMPLE_DIV     10U
#define ENC_CAL_SAMPLE_TS      (CUR_TS * (float)ENC_CAL_SAMPLE_DIV)
#define ENC_CAL_SAMPLES        ((uint32_t)(6.0f / ENC_CAL_SAMPLE_TS + 0.5f))
#define ENC_CAL_COEF_MAX       0.05f
#define ENC_CAL_COAST_WM       2.0f
#define ENC_CAL_COAST_HOLD_CNT ((uint32_t)(0.3f / SPD_TS + 0.5f))
#define ENC_CAL_COAST_MAX_CNT  ((uint32_t)(30.0f / SPD_TS + 0.5f))
#define ENC_CAL_FIT_N          6U

typedef enum
{
    CAL_IDLE = 0,
    CAL_SPEED,
    CAL_VF,
    CAL_SAMPLE,
    CAL_COAST,
    CAL_PHASE,

} Enc_Cal_Step_e;

static volatile Enc_Cal_Step_e Step = CAL_IDLE;
static volatile uint32_t Sample_Cnt = 0U;
static uint8_t Mode_Pre = TORQUE;
static bool Success = false;
static uint32_t Tick = 0U;
static uint32_t Hold_Cnt = 0U;
static volatile bool VF_Ready = false;
static VF_T Cal_VF = { 0 };

static uint8_t Div = 0U;
static float Theta_Pre = 0.0f;
static float Theta_Sum = 0.0f;
static float A[ENC_CAL_FIT_N][ENC_CAL_FIT_N] = { 0 };
static float B[ENC_CAL_FIT_N] = { 0 };
static float Y2 = 0.0f;

static void Enc_Cal_Finish(bool Passed)
{
    if (Motor_State_Get() != DISABLED)
    {
        Motor_Disable();
    }

    Motor_Mode = Mode_Pre;
    Success = Passed;
    Step = CAL_IDLE;
}

bool Enc_Cal_Start(void)
{
    if ((Step != CAL_IDLE) || (Motor_State_Get() != DISABLED) ||
        (Motor_Wm_Limit_Effective_Get() < ENC_CAL_WM) ||
        (Motion_Config.Wm_Acc <= 0.0f) ||
        (Motor_Para.Flux <= 0.0f))
    {
        return false;
    }

    Mode_Pre = Motor_Mode;
    Success = false;
    Tick = 0U;
    Hold_Cnt = 0U;
    VF_Ready = false;
    Cal_VF = (VF_T){ 0 };
    Div = 0U;
    Sample_Cnt = 0U;
    Theta_Pre = 0.0f;
    Theta_Sum = 0.0f;
    Y2 = 0.0f;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] = 0.0f;
        for (uint32_t j = 0U; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] = 0.0f;
        }
    }

    Step = CAL_SPEED;
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

bool Enc_Cal_Active(void)
{
    return Step != CAL_IDLE;
}

bool Enc_Cal_Succeeded(void)
{
    return Success;
}

bool Enc_Cal_Sensored(void)
{
    return Step == CAL_SPEED;
}

float Enc_Cal_Wm_Target(void)
{
    return ENC_CAL_WM;
}

Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                   float *Ualpha,
                                   float *Ubeta)
{
    float U_Target;

    if ((Step != CAL_VF) && (Step != CAL_SAMPLE))
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
    return FAST_VOLTAGE;
}

void Enc_Cal_Sample(float Theta_Native)
{
    float X[ENC_CAL_FIT_N];
    float Delta;
    float Time;
    float Y;
    float Sin1;
    float Cos1;
    float Sin2;
    float Cos2;

    if ((Step != CAL_SAMPLE) || (Sample_Cnt >= ENC_CAL_SAMPLES))
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
    Y2 += Y * Y;
    Sample_Cnt++;
}

/* Small, offline 6x6 least-squares solve; never called from the fast ISR. */
static bool Enc_Cal_Fit(float C[4])
{
    float M[ENC_CAL_FIT_N][ENC_CAL_FIT_N + 1U];
    float Beta[ENC_CAL_FIT_N];
    float SSE = Y2;

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

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        SSE -= Beta[i] * B[i];
    }
    if (!__builtin_isfinite(SSE) || (SSE < -0.01f * (float)ENC_CAL_SAMPLES))
    {
        return false;
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

    if (Step == CAL_IDLE)
    {
        return;
    }

    if (Step == CAL_PHASE)
    {
        if (Motor_State_Get() == DISABLED)
        {
            bool Passed = Servo_Phase_Result_Valid() && (Motor_Cal.Valid != 0U);
            if (!Passed)
            {
                Motor_Cal.Valid = 0U;
            }
            Enc_Cal_Finish(Passed);
        }
        return;
    }

    if ((Step != CAL_COAST) && (Motor_State_Get() != RUN))
    {
        Enc_Cal_Finish(false);
        return;
    }

    if (Step == CAL_SPEED)
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
            Step = CAL_VF;
        }
        else if (Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Acc + 4.0f) / SPD_TS))
        {
            Enc_Cal_Finish(false);
        }
        return;
    }

    if (Step == CAL_VF)
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
        Step = CAL_SAMPLE;
        return;
    }

    if ((Step == CAL_SAMPLE) && (Sample_Cnt == ENC_CAL_SAMPLES))
    {
        /* Stop VF before stationary alignment; the rotor may still be at 1000 RPM. */
        Motor_Disable();
        Tick = 0U;
        Hold_Cnt = 0U;
        Step = CAL_COAST;
        return;
    }

    if (Step == CAL_COAST)
    {
        if (++Tick >= ENC_CAL_COAST_MAX_CNT)
        {
            Enc_Cal_Finish(false);
            return;
        }

        if (__builtin_fabsf(Mechanical_ESO_Wm_Get()) < ENC_CAL_COAST_WM)
        {
            Hold_Cnt++;
        }
        else
        {
            Hold_Cnt = 0U;
        }
        if (Hold_Cnt < ENC_CAL_COAST_HOLD_CNT)
        {
            return;
        }

        if (!Enc_Cal_Fit(Coeff))
        {
            Enc_Cal_Finish(false);
            return;
        }

        /* Power loss must not recover a new harmonic model with the old
         * electrical zero. Servo_Phase saves the complete set after alignment. */
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

        Motor_Mode = PHASE_SEARCH;
        Motor_Enable();
        if (Motor_State_Get() == ENABLED)
        {
            Motor_Start();
        }
        if (Motor_State_Get() != RUN)
        {
            Enc_Cal_Finish(false);
            return;
        }
        Step = CAL_PHASE;
    }
}
