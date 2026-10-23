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
#include "motor_work.h"
#include "Motor_Para.h"
#include "Motion_Type.h"
#include "NVS_Storage.h"
#include "Parameter.h"
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

bool Enc_Cal_Start(void)
{
    if ((Motor_Wm_Limit_Effective_Get() < ENC_CAL_WM) ||
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
    Y2 = 0.0f;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] = 0.0f;
        for (uint32_t j = i; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] = 0.0f;
        }
    }

    Enc_Cal.Step = CAL_SPEED;
    return true;
}

/* Publish a calibration phase and its 20 kHz execution entry together.
 * Do not mask interrupts while running the rest of the 2 kHz procedure. */
static void Enc_Cal_Step_Set(Enc_Cal_Step_e Step)
{
    uint32_t Primask = __get_PRIMASK();

    __disable_irq();
    Enc_Cal.Step = Step;
    Motor_Fast_Select();
    __set_PRIMASK(Primask);
}

void Enc_Cal_Abort(void)
{
    if ((Enc_Cal.Step >= CAL_SPEED) && (Enc_Cal.Step < CAL_DONE))
    {
        /* Prevent a late worker result from applying after cancellation. */
        Enc_Cal_Step_Set(CAL_FAILED);
    }
}

Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                   float *Ualpha,
                                   float *Ubeta)
{
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

    if (Enc_Cal.Step == CAL_HANDOFF)
    {
        /* Keep rotating while the 2 kHz owner prepares the speed ramp. */
        Enc_Cal_Handoff_Ready = true;
    }

    Enc_Cal_Sample(Encoder.Theta_Native);
    /* An incomplete/invalid capture cannot become a successful fit. */
    return (Sample_Cnt > ENC_CAL_SAMPLES) ? FAST_OFF : FAST_VOLTAGE;
}

static FAST_CODE void Enc_Cal_Sample(float Theta_Native)
{
    float X[ENC_CAL_FIT_N];
    float Delta;

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
    }
    Theta_Pre = Theta_Native;

    /* Offset, speed error, then 1x/2x encoder harmonics. */
    X[0] = 1.0f;
    X[1] = (float)Sample_Cnt * ENC_CAL_SAMPLE_TS - 3.0f;
    SinCos(Theta_Native, &X[3], &X[2]);
    SinCos(Angle_Wrap(2.0f * Theta_Native), &X[5], &X[4]);

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        B[i] += X[i] * Theta_Err;
        for (uint32_t j = i; j < ENC_CAL_FIT_N; j++)
        {
            A[i][j] += X[i] * X[j];
        }
    }
    Y2 += Theta_Err * Theta_Err;
    Sample_Cnt++;
}

/* A = X'X is positive definite for a complete rotating capture. LDL' solves
 * A*Beta = B without pivoting or square roots. Double precision is confined to this offline
 * solve so RSS = Y2 - Beta'B does not amplify float solve error. */
static bool Enc_Cal_Fit(float C[4])
{
    double LD[ENC_CAL_FIT_N][ENC_CAL_FIT_N]; /* Unit L below diagonal, D on diagonal. */
    double Beta[ENC_CAL_FIT_N];

    if (Sample_Cnt != ENC_CAL_SAMPLES)
    {
        return false;
    }

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        for (uint32_t j = 0U; j <= i; j++)
        {
            double V = A[j][i];
            for (uint32_t k = 0U; k < j; k++)
            {
                V -= LD[i][k] * LD[j][k] * LD[k][k];
            }
            if (i == j)
            {
                if (!__builtin_isfinite(V) || (V < 0.0001f * (float)ENC_CAL_SAMPLES))
                {
                    return false;
                }
                LD[i][j] = V;
            }
            else
            {
                LD[i][j] = V / LD[j][j];
            }
        }

        /* Forward substitution L*z = B; Beta temporarily holds z. */
        double V = B[i];
        for (uint32_t j = 0U; j < i; j++)
        {
            V -= LD[i][j] * Beta[j];
        }
        Beta[i] = V;
    }

    for (int32_t i = (int32_t)ENC_CAL_FIT_N - 1; i >= 0; i--)
    {
        double V = Beta[i] / LD[i][i];
        for (uint32_t j = (uint32_t)i + 1U; j < ENC_CAL_FIT_N; j++)
        {
            V -= LD[j][i] * Beta[j];
        }
        Beta[i] = V;
        if (!__builtin_isfinite(Beta[i]))
        {
            return false;
        }
    }

    for (uint32_t i = 0U; i < 4U; i++)
    {
        if (__builtin_fabs(Beta[i + 2U]) > ENC_CAL_COEF_MAX)
        {
            return false;
        }
        C[i] = (float)Beta[i + 2U];
    }

    double Rss = (double)Y2;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        Rss -= Beta[i] * (double)B[i];
    }
    double Rss_Max = (double)ENC_CAL_SAMPLES * ENC_CAL_RMS_MAX * ENC_CAL_RMS_MAX;
    if (!__builtin_isfinite(Rss) || (Rss > Rss_Max) || (Rss < -Rss_Max) ||
        (__builtin_fabs(Beta[1]) > 0.1f * ENC_CAL_WM))
    {
        return false;
    }

    return true;
}

/* The low-priority Motor Work thread calls this only while DISABLED.
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
    if ((Enc_Cal.Step != CAL_FIT_WAIT) || (Motor_State_Get() != DISABLED) ||
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

bool Enc_Cal_Save(void)
{
    const uint16_t Ids[] = {
        /* Publish the invalid phase marker before the changed correction. */
        PARAM_CAL_VALID,
        PARAM_ENCODER_HARMONIC_C1, PARAM_ENCODER_HARMONIC_S1,
        PARAM_ENCODER_HARMONIC_C2, PARAM_ENCODER_HARMONIC_S2,
    };

    return NVS_Storage_Save_Ids(Ids, sizeof(Ids) / sizeof(Ids[0])) == 0;
}

void Enc_Cal_Run(void)
{
    float Wm;
 
    if ((Enc_Cal.Step == CAL_IDLE) || (Enc_Cal.Step >= CAL_DONE))
    {
        return;
    }

    /* Fitting and saving follow the actual transition to DISABLED. */
    if (Motor_State_Get() != ((Enc_Cal.Step < CAL_FIT) ? RUN : DISABLED))
    {
        Enc_Cal_Step_Set(CAL_FAILED);
        return;
    }

    if (Enc_Cal.Step == CAL_SPEED)
    {
        Wm = Mechanical_ESO.State.Wm;
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
            Enc_Cal_Step_Set(CAL_VF);
        }
        else if (Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Acc + 4.0f) / SPD_TS))
        {
            Enc_Cal_Step_Set(CAL_FAILED);
        }
        return;
    }

    if (Enc_Cal.Step == CAL_VF)
    {
        if (!VF_Ready)
        {
            return;
        }
        Wm = Mechanical_ESO.State.Wm;
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
            Enc_Cal_Step_Set(CAL_FAILED);
            return;
        }

        Enc_Cal_Step_Set(CAL_SAMPLE);
        return;
    }

    if (Enc_Cal.Step == CAL_SAMPLE)
    {
        Wm = Mechanical_ESO.State.Wm;
        if ((Sample_Cnt > ENC_CAL_SAMPLES) || !__builtin_isfinite(Wm) ||
            (__builtin_fabsf(Wm - ENC_CAL_WM) >= 0.1f * ENC_CAL_WM))
        {
            Enc_Cal_Step_Set(CAL_FAILED);
        }
        else if (Sample_Cnt == ENC_CAL_SAMPLES)
        {
            Enc_Cal_Step_Set(CAL_HANDOFF);
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
            Enc_Cal_Step_Set(CAL_DECEL);
        }
        return;
    }

    if (Enc_Cal.Step == CAL_DECEL)
    {
        Wm = Mechanical_ESO.State.Wm;
        if (++Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Dec + 4.0f) / SPD_TS))
        {
            Enc_Cal_Step_Set(CAL_FAILED);
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

        Enc_Cal_Step_Set(CAL_FIT);
        return;
    }

    if (Enc_Cal.Step == CAL_FIT)
    {
        if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
        {
            if (!Motor_Work_Request(MOTOR_WORK_FIT))
            {
                Enc_Cal_Step_Set(CAL_FAILED);
                return;
            }
            Enc_Cal_Step_Set(CAL_FIT_WAIT);
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
            Enc_Cal_Step_Set(CAL_FAILED);
            return;
        }

        if (!Motor_Work_Request(MOTOR_WORK_SAVE_ENC))
        {
            Enc_Cal_Step_Set(CAL_FAILED);
            return;
        }

        Enc_Cal_Step_Set(CAL_SAVE_WAIT);
        return;
    }

    if ((Enc_Cal.Step == CAL_SAVE_WAIT) && (Motor_Work_State >= MOTOR_WORK_DONE))
    {
        Enc_Cal_Step_Set((Motor_Work_State == MOTOR_WORK_DONE) ? CAL_DONE : CAL_FAILED);
        Motor_Work_Ack();
    }
}
