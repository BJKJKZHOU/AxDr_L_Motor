/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Servo_Phase.h"

#include "Encoder.h"
#include "Math.h"
#include "control_params.h"
#include "motor_params.h"

/* 电角度置零对齐后，施加正 Iq 判断编码器方向。
 * Iq 归零等待 500 ms，再次对齐计算零位；禁能后应用结果。 */
#define PHASE_SEARCH_I_DEFAULT_A  0.5f
#define PHASE_DIR_CONFIRM_E_RAD   0.05f
#define PHASE_STABLE_ERR_E_RAD    0.05f
#define PHASE_STABLE_WINDOW_S     0.10f
#define PHASE_STABLE_CNT         ((uint32_t)(PHASE_STABLE_WINDOW_S / CUR_TS + 0.5f))
#define PHASE_ZERO_IQ_WAIT_S     0.50f
#define PHASE_ZERO_IQ_WAIT_CNT   ((uint32_t)(PHASE_ZERO_IQ_WAIT_S / CUR_TS + 0.5f))

typedef struct
{
    uint32_t Stable_Cnt;
    uint32_t Zero_Iq_Cnt;
    float Stable_Theta;
    float Stable_Sum;
    float Theta_Native_Align;
    float Theta_Native_Pre;
    float Native_Delta;
    bool Use_Harmonic;

} Servo_Phase_State_T;

Servo_Phase_T Servo_Phase =
{
    .I_Search_A = PHASE_SEARCH_I_DEFAULT_A,
};

static Servo_Phase_State_T Phase = { 0 };

static bool Angle_Stable(float Theta, float *Theta_Mean)
{
    float Delta = Angle_Diff(Theta, Phase.Stable_Theta);

    if (__builtin_fabsf(Delta) > PHASE_STABLE_ERR_E_RAD / (float)Motor_Para.Pp)
    {
        Phase.Stable_Theta = Theta;
        Phase.Stable_Sum = 0.0f;
        Phase.Stable_Cnt = 0U;
        return false;
    }

    Phase.Stable_Sum += Delta;
    Phase.Stable_Cnt++;
    if (Phase.Stable_Cnt < PHASE_STABLE_CNT)
    {
        return false;
    }

    *Theta_Mean = Angle_Wrap(Phase.Stable_Theta + Phase.Stable_Sum / (float)Phase.Stable_Cnt);
    return true;
}

bool Servo_Phase_Start(bool Use_Harmonic)
{
    if ((Encoder.Ready == 0U) || (Encoder.Fault != 0U))
    {
        return false;
    }

    Phase = (Servo_Phase_State_T){ 0 };
    Servo_Phase.Theta_Off = 0.0f;
    Servo_Phase.Theta_Off_Error = 0.0f;

    Phase.Stable_Theta = Encoder.Theta_Native;
    Servo_Phase.Enc_Dir = 1; /* 暂用于判断方向，结束后再应用到 Motor_Cal。 */
    Phase.Use_Harmonic = Use_Harmonic;
    Servo_Phase.Step = PHASE_ALIGN;

    return true;
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref)
{
    float Delta;
    float Theta_Native_Stable;
    float Offset_Initial;
    float Offset_Final;
    float Offset_Error;

    if ((Encoder.Valid == 0U) || (Encoder.Fault != 0U))
    {
        Servo_Phase.Step = PHASE_FAILED;
        return FAST_OFF;
    }

    switch (Servo_Phase.Step)
    {
        case PHASE_ALIGN:
            *Id_Ref = Servo_Phase.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native, &Theta_Native_Stable))
            {
                /* 编码器校准后，寻相使用新系数计算零位。 */
                Phase.Theta_Native_Align = Encoder_Angle_Correct(
                    Theta_Native_Stable, Phase.Use_Harmonic);
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Servo_Phase.Step = PHASE_RUN;
            }
            return FAST_CURRENT;

        case PHASE_RUN:
        case PHASE_ZERO_IQ:
            Delta = Angle_Diff(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Native_Delta += Delta;

            if (Servo_Phase.Step == PHASE_RUN)
            {
                *Iq_Ref = Servo_Phase.I_Search_A;

                if (__builtin_fabsf((float)Motor_Para.Pp * Phase.Native_Delta) >=
                    PHASE_DIR_CONFIRM_E_RAD)
                {
                    Servo_Phase.Enc_Dir = (Phase.Native_Delta > 0.0f) ? 1 : -1;
                    Servo_Phase.Step = PHASE_ZERO_IQ;
                    *Iq_Ref = 0.0f;
                }
            }
            else if (++Phase.Zero_Iq_Cnt >= PHASE_ZERO_IQ_WAIT_CNT)
            {
                Phase.Stable_Theta = Encoder.Theta_Native;
                Phase.Stable_Sum = 0.0f;
                Phase.Stable_Cnt = 0U;
                Servo_Phase.Step = PHASE_ALIGN_FINAL;
            }

            *Theta_e = Angle_Wrap((float)Motor_Para.Pp *
                                  (float)Servo_Phase.Enc_Dir * Phase.Native_Delta);
            return FAST_CURRENT;

        case PHASE_ALIGN_FINAL:
            *Id_Ref = Servo_Phase.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native, &Theta_Native_Stable))
            {
                Theta_Native_Stable = Encoder_Angle_Correct(
                    Theta_Native_Stable, Phase.Use_Harmonic);
                Offset_Initial = Angle_Wrap(-(float)Motor_Para.Pp *
                                            (float)Servo_Phase.Enc_Dir * Phase.Theta_Native_Align);
                Offset_Final = Angle_Wrap(-(float)Motor_Para.Pp *
                                          (float)Servo_Phase.Enc_Dir * Theta_Native_Stable);
                Offset_Error = Angle_Diff(Offset_Final, Offset_Initial);

                /* 两次对齐位置应相差整数个电周期，用零位之差检查偏差。 */
                Servo_Phase.Theta_Off_Error = Offset_Error;
                Servo_Phase.Theta_Off = Angle_Wrap(Offset_Initial + 0.5f * Offset_Error);
                Servo_Phase.Step = PHASE_DONE;
                return FAST_OFF;
            }
            return FAST_CURRENT;

        case PHASE_DONE:
        case PHASE_FAILED:
        case PHASE_IDLE:
        default:
            return FAST_OFF;
    }
}
