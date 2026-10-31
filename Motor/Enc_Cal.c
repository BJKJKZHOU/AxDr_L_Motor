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
/* 拟合后的机械角度残差 RMS：实测 0.06～0.08°，合格上限取 0.25°。 */
#define ENC_CAL_RMS_MAX        (0.25f * PI_F / 180.0f)
#define ENC_CAL_FIT_N          6U

volatile Enc_Cal_T Enc_Cal = { 0 };
volatile bool Enc_Cal_Handoff_Ready = false;

typedef struct
{
    uint32_t Tick;
    uint32_t Hold_Cnt;
    volatile uint32_t Sample_Cnt;
    volatile bool VF_Ready;
    VF_T VF;
    uint8_t Div;
    float Theta_Pre;
    float Theta_Err;
    float A[ENC_CAL_FIT_N][ENC_CAL_FIT_N];
    float B[ENC_CAL_FIT_N];
    float Y2;

} Enc_Cal_State_T;

static Enc_Cal_State_T Cal = { 0 };

static FAST_CODE void Enc_Cal_Sample(float Theta_Native);

bool Enc_Cal_Start(void)
{
    if ((Motor_Wm_Limit_Effective_Get() < ENC_CAL_WM) ||
        (Motion_Config.Wm_Acc <= 0.0f) || (Motion_Config.Wm_Dec <= 0.0f) ||
        (Motor_Para.Flux <= 0.0f))
    {
        return false;
    }

    Cal = (Enc_Cal_State_T){ 0 };
    Enc_Cal_Handoff_Ready = false;

    Enc_Cal.Step = CAL_SPEED;
    return true;
}

void Enc_Cal_Abort(void)
{
    if ((Enc_Cal.Step >= CAL_SPEED) && (Enc_Cal.Step < CAL_DONE))
    {
        /* 标记失败，使后台拟合结果在取消后失效。 */
        Enc_Cal.Step = CAL_FAILED;
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

    if (!Cal.VF_Ready)
    {
        /* 保存的是上周期电压，先推进一个周期的相位以保持连续。 */
        float Ua = Motor_Run.Ualpha;
        float Ub = Motor_Run.Ubeta;

        Cal.VF.State.U = __builtin_sqrtf(Ua * Ua + Ub * Ub);
        Cal.VF.State.We = (float)Motor_Para.Pp * ENC_CAL_WM;
        Cal.VF.State.Theta_e = Angle_Wrap(__builtin_atan2f(-Ua, Ub) +
                                         Cal.VF.State.We * CUR_TS);
        Cal.VF.Para.U_Slew_V_S = 30.0f;
        Cal.VF.Para.We_Acc = (float)Motor_Para.Pp * Motion_Config.Wm_Acc;
        Cal.VF_Ready = true;
    }

    /* 切入 VF 后保持电压幅值和转速，开环采集编码器误差。 */
    Cal.VF.Para.U_Max_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;
    *Theta_e = Cal.VF.State.Theta_e;
    VF_Run(&Cal.VF, (float)Motor_Para.Pp * ENC_CAL_WM,
           Cal.VF.State.U, Ualpha, Ubeta, CUR_TS);

    if (Enc_Cal.Step == CAL_HANDOFF)
    {
        /* 等待速度环准备减速时，继续 VF 转动。 */
        Enc_Cal_Handoff_Ready = true;
    }

    Enc_Cal_Sample(Encoder.Theta_Native);
    /* 采样失败时关闭输出。 */
    return (Cal.Sample_Cnt > ENC_CAL_SAMPLES) ? FAST_OFF : FAST_VOLTAGE;
}

static FAST_CODE void Enc_Cal_Sample(float Theta_Native)
{
    float X[ENC_CAL_FIT_N];
    float Delta;

    if ((Enc_Cal.Step != CAL_SAMPLE) || (Cal.Sample_Cnt >= ENC_CAL_SAMPLES))
    {
        return;
    }

    if ((Encoder.Valid == 0U) || (Encoder.Fault != 0U) ||
        !__builtin_isfinite(Theta_Native))
    {
        /* 用超出目标数量的计数记录采样失败，供低频任务检查。 */
        Cal.Sample_Cnt = ENC_CAL_SAMPLES + 1U;
        return;
    }

    if (++Cal.Div < ENC_CAL_SAMPLE_DIV)
    {
        return;
    }
    Cal.Div = 0U;

    if (Cal.Sample_Cnt == 0U)
    {
        Cal.Theta_Err = 0.0f;
    }
    else
    {
        Delta = Theta_Native - Cal.Theta_Pre;
        if (Delta > PI_F)
        {
            Delta -= TWO_PI_F;
        }
        else if (Delta < -PI_F)
        {
            Delta += TWO_PI_F;
        }
        /* 逐次累加角度误差，避免大角度相减造成浮点精度损失。 */
        Cal.Theta_Err += Delta - (float)Motor_Cal.Enc_Dir * ENC_CAL_WM * ENC_CAL_SAMPLE_TS;
    }
    Cal.Theta_Pre = Theta_Native;

    /* 拟合项：偏置、速度误差、一阶和二阶谐波。 */
    X[0] = 1.0f;
    X[1] = (float)Cal.Sample_Cnt * ENC_CAL_SAMPLE_TS - 3.0f;
    SinCos(Theta_Native, &X[3], &X[2]);
    SinCos(Angle_Wrap(2.0f * Theta_Native), &X[5], &X[4]);

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        Cal.B[i] += X[i] * Cal.Theta_Err;
        for (uint32_t j = i; j < ENC_CAL_FIT_N; j++)
        {
            Cal.A[i][j] += X[i] * X[j];
        }
    }
    Cal.Y2 += Cal.Theta_Err * Cal.Theta_Err;
    Cal.Sample_Cnt++;
}

/* 用 LDL' 分解求解 A*Beta=B；采用双精度以减小残差计算误差。 */
static bool Enc_Cal_Fit(float C[4])
{
    double LD[ENC_CAL_FIT_N][ENC_CAL_FIT_N]; /* 下三角保存 L，对角线保存 D。 */
    double Beta[ENC_CAL_FIT_N];

    if (Cal.Sample_Cnt != ENC_CAL_SAMPLES)
    {
        return false;
    }

    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        for (uint32_t j = 0U; j <= i; j++)
        {
            double V = Cal.A[j][i];
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

        /* 求解 L*z=B，Beta 暂存 z。 */
        double V = Cal.B[i];
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

    double Rss = (double)Cal.Y2;
    for (uint32_t i = 0U; i < ENC_CAL_FIT_N; i++)
    {
        Rss -= Beta[i] * (double)Cal.B[i];
    }
    double Rss_Max = (double)ENC_CAL_SAMPLES * ENC_CAL_RMS_MAX * ENC_CAL_RMS_MAX;
    if (!__builtin_isfinite(Rss) || (Rss > Rss_Max) || (Rss < -Rss_Max) ||
        (__builtin_fabs(Beta[1]) > 0.1f * ENC_CAL_WM))
    {
        return false;
    }

    return true;
}

/* 禁能后由后台线程拟合并应用系数。 */
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
        /* 先保存无效标志，再保存新系数。 */
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

    /* 拟合和保存阶段要求电机已禁能。 */
    if (Motor_State_Get() != ((Enc_Cal.Step < CAL_FIT) ? RUN : DISABLED))
    {
        Enc_Cal.Step = CAL_FAILED;
        return;
    }

    if (Enc_Cal.Step == CAL_SPEED)
    {
        Wm = Mechanical_ESO.State.Wm;
        Cal.Tick++;
        if (__builtin_fabsf(Wm - ENC_CAL_WM) < 0.1f * ENC_CAL_WM)
        {
            Cal.Hold_Cnt++;
        }
        else
        {
            Cal.Hold_Cnt = 0U;
        }

        if (Cal.Hold_Cnt >= ENC_CAL_SPEED_HOLD_CNT)
        {
            Cal.Tick = 0U;
            Cal.Hold_Cnt = 0U;
            Enc_Cal.Step = CAL_VF;
        }
        else if (Cal.Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Acc + 4.0f) / SPD_TS))
        {
            Enc_Cal.Step = CAL_FAILED;
        }
        return;
    }

    if (Enc_Cal.Step == CAL_VF)
    {
        if (!Cal.VF_Ready)
        {
            return;
        }
        Wm = Mechanical_ESO.State.Wm;
        if (__builtin_isfinite(Wm) &&
            (__builtin_fabsf(Wm - ENC_CAL_WM) < 0.1f * ENC_CAL_WM))
        {
            Cal.Hold_Cnt++;
        }
        else
        {
            Cal.Hold_Cnt = 0U;
        }
        if (++Cal.Tick < ENC_CAL_VF_SETTLE_CNT)
        {
            return;
        }

        if (Cal.Hold_Cnt < ENC_CAL_SPEED_HOLD_CNT)
        {
            Enc_Cal.Step = CAL_FAILED;
            return;
        }

        Enc_Cal.Step = CAL_SAMPLE;
        return;
    }

    if (Enc_Cal.Step == CAL_SAMPLE)
    {
        Wm = Mechanical_ESO.State.Wm;
        if ((Cal.Sample_Cnt > ENC_CAL_SAMPLES) || !__builtin_isfinite(Wm) ||
            (__builtin_fabsf(Wm - ENC_CAL_WM) >= 0.1f * ENC_CAL_WM))
        {
            Enc_Cal.Step = CAL_FAILED;
        }
        else if (Cal.Sample_Cnt == ENC_CAL_SAMPLES)
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
            Cal.Hold_Cnt = 0U;
            Cal.Tick = 0U;
            Enc_Cal.Step = CAL_DECEL;
        }
        return;
    }

    if (Enc_Cal.Step == CAL_DECEL)
    {
        Wm = Mechanical_ESO.State.Wm;
        if (++Cal.Tick > (uint32_t)((ENC_CAL_WM / Motion_Config.Wm_Dec + 4.0f) / SPD_TS))
        {
            Enc_Cal.Step = CAL_FAILED;
            return;
        }
        if ((Motor_Wm_Ref_Get() != 0.0f) || !__builtin_isfinite(Wm) ||
            (__builtin_fabsf(Wm) > ENC_CAL_STOP_WM))
        {
            Cal.Hold_Cnt = 0U;
            return;
        }

        if (++Cal.Hold_Cnt < ENC_CAL_STOP_WAIT_CNT)
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
                Enc_Cal.Step = CAL_FAILED;
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
            Enc_Cal.Step = CAL_FAILED;
            return;
        }

        if (!Motor_Work_Request(MOTOR_WORK_SAVE_ENC))
        {
            Enc_Cal.Step = CAL_FAILED;
            return;
        }

        Enc_Cal.Step = CAL_SAVE_WAIT;
        return;
    }

    if ((Enc_Cal.Step == CAL_SAVE_WAIT) && (Motor_Work_State >= MOTOR_WORK_DONE))
    {
        Enc_Cal.Step = (Motor_Work_State == MOTOR_WORK_DONE) ? CAL_DONE : CAL_FAILED;
        Motor_Work_Ack();
    }
}
