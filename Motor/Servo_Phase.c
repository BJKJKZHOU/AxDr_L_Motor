/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Servo_Phase.h"

#include "Encoder.h"
#include "Enc_Cal.h"
#include "Math.h"
#include "Motor_Cal.h"
#include "NVS_Storage.h"
#include "Parameter.h"
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

typedef enum
{
    PHASE_IDLE = 0,
    PHASE_ALIGN,
    PHASE_RUN,
    PHASE_ZERO_IQ,
    PHASE_ALIGN_FINAL,
    PHASE_DONE,
    PHASE_FAILED,

} Servo_Phase_State_e;

typedef struct
{
    Servo_Phase_State_e State;
    uint32_t Stable_Cnt;
    uint32_t Zero_Iq_Cnt;
    float Stable_Theta;
    float Stable_Sum;
    float Theta_Native_Align;
    float Theta_Native_Pre;
    float Native_Delta;
    int8_t Enc_Dir;

} Servo_Phase_T;

Servo_Phase_Config_T Servo_Phase_Config =
{
    .I_Search_A = PHASE_SEARCH_I_DEFAULT_A,
};

typedef struct
{
    int8_t Enc_Dir;
    float Theta_Off;
    float Theta_Off_Error;

} Servo_Phase_Result_T;

static Servo_Phase_T Phase = { 0 };
static Servo_Phase_Result_T Last_Result = { 0 };

static float Angle_Delta(float Theta, float Theta_Pre)
{
    float Delta = Theta - Theta_Pre;

    if (Delta > PI_F)
    {
        Delta -= TWO_PI_F;
    }
    else if (Delta < -PI_F)
    {
        Delta += TWO_PI_F;
    }

    return Delta;
}

static bool Angle_Stable(float Theta, float *Theta_Mean)
{
    float Delta = Angle_Delta(Theta, Phase.Stable_Theta);

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

static void Fail(void)
{
    Phase.State = PHASE_FAILED;
    Motor_Cal.Valid = 0U;
}

bool Servo_Phase_Start(void)
{
    if ((Encoder.Ready == 0U) || (Encoder.Fault != 0U) || (Motor_Para.Pp == 0U))
    {
        return false;
    }

    Phase = (Servo_Phase_T){ 0 };
    Last_Result = (Servo_Phase_Result_T){ 0 };
    /* 重复寻相保留已有的编码器校准，失败或中止时再清零。 */
    if (Motor_Cal.Valid != 2U)
    {
        Motor_Cal.Valid = 0U;
    }

    Phase.Stable_Theta = Encoder.Theta_Native;
    Phase.Enc_Dir = 1; /* 暂用于判断方向，结束后再应用到 Motor_Cal。 */
    Phase.State = PHASE_ALIGN;

    return true;
}

void Servo_Phase_Abort(void)
{
    Phase.State = PHASE_IDLE;
    Motor_Cal.Valid = 0U;
}

bool Servo_Phase_Active(void)
{
    return (Phase.State >= PHASE_ALIGN) && (Phase.State <= PHASE_ALIGN_FINAL);
}

bool Servo_Phase_Apply(void)
{
    if (Phase.State != PHASE_DONE)
    {
        Phase.State = PHASE_IDLE;
        return false;
    }

    if (!Motor_Cal_Set(Last_Result.Enc_Dir, Last_Result.Theta_Off))
    {
        Fail();
        Phase.State = PHASE_IDLE;
        return false;
    }

    Phase.State = PHASE_IDLE;
    return true;
}

bool Servo_Phase_Save(void)
{
    const uint16_t Ids[] = {
        PARAM_CAL_ENC_DIR, PARAM_CAL_THETA_OFF, PARAM_CAL_VALID,
    };

    return (Motor_Cal.Valid != 0U) &&
           (NVS_Storage_Save_Ids(Ids, sizeof(Ids) / sizeof(Ids[0])) == 0);
}

float Servo_Phase_Theta_Off_Error_Get(void)
{
    return Last_Result.Theta_Off_Error;
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta)
{
    float Delta;
    float Theta_Native_Stable;
    float Offset_Initial;
    float Offset_Final;
    float Offset_Error;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha = 0.0f;
    *Ubeta = 0.0f;

    if ((Encoder.Valid == 0U) || (Encoder.Fault != 0U))
    {
        Fail();
        return FAST_OFF;
    }

    switch (Phase.State)
    {
        case PHASE_ALIGN:
            *Theta_e = 0.0f;
            *Id_Ref = Servo_Phase_Config.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native, &Theta_Native_Stable))
            {
                /* 编码器校准后，寻相使用新系数计算零位。 */
                Phase.Theta_Native_Align = Encoder_Angle_Correct(
                    Theta_Native_Stable, (Motor_Cal.Valid == 2U) || (Enc_Cal.Step == CAL_DONE));
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.Native_Delta = 0.0f;
                Phase.State = PHASE_RUN;
            }
            return FAST_CURRENT;

        case PHASE_RUN:
        case PHASE_ZERO_IQ:
            Delta = Angle_Delta(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Native_Delta += Delta;

            if (Phase.State == PHASE_RUN)
            {
                *Iq_Ref = Servo_Phase_Config.I_Search_A;

                if (__builtin_fabsf((float)Motor_Para.Pp * Phase.Native_Delta) >=
                    PHASE_DIR_CONFIRM_E_RAD)
                {
                    Phase.Enc_Dir = (Phase.Native_Delta > 0.0f) ? 1 : -1;
                    Phase.Zero_Iq_Cnt = 0U;
                    Phase.State = PHASE_ZERO_IQ;
                    *Iq_Ref = 0.0f;
                }
            }
            else if (++Phase.Zero_Iq_Cnt >= PHASE_ZERO_IQ_WAIT_CNT)
            {
                Phase.Stable_Theta = Encoder.Theta_Native;
                Phase.Stable_Sum = 0.0f;
                Phase.Stable_Cnt = 0U;
                Phase.State = PHASE_ALIGN_FINAL;
            }

            *Theta_e = Angle_Wrap((float)Motor_Para.Pp *
                                  (float)Phase.Enc_Dir * Phase.Native_Delta);
            return FAST_CURRENT;

        case PHASE_ALIGN_FINAL:
            *Theta_e = 0.0f;
            *Id_Ref = Servo_Phase_Config.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native, &Theta_Native_Stable))
            {
                Theta_Native_Stable = Encoder_Angle_Correct(
                    Theta_Native_Stable, (Motor_Cal.Valid == 2U) || (Enc_Cal.Step == CAL_DONE));
                Offset_Initial = Angle_Wrap(-(float)Motor_Para.Pp *
                                            (float)Phase.Enc_Dir * Phase.Theta_Native_Align);
                Offset_Final = Angle_Wrap(-(float)Motor_Para.Pp *
                                          (float)Phase.Enc_Dir * Theta_Native_Stable);
                Offset_Error = Angle_Delta(Offset_Final, Offset_Initial);

                /* 两次对齐位置应相差整数个电周期，用零位之差检查偏差。 */
                Last_Result.Theta_Off_Error = Offset_Error;
                Last_Result.Theta_Off = Angle_Wrap(Offset_Initial + 0.5f * Offset_Error);
                Phase.State = PHASE_DONE;
                Last_Result.Enc_Dir = Phase.Enc_Dir;
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
