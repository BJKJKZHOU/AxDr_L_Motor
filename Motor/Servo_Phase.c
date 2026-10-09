/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Servo_Phase.h"

#include "Encoder.h"
#include "Math.h"
#include "Motor_Cal.h"
#include "NVS_Storage.h"
#include "Parameter.h"
#include "control_params.h"
#include "motor_params.h"

/*
 * Servo phase search establishes the encoder direction and electrical zero:
 *   1. align rotor with a fixed electrical field,
 *   2. turn that field one positive electrical revolution,
 *   3. hold the field until the encoder angle is stable.
 * No speed loop, sensorless observer or positive-Iq verification is required.
 */
#define PHASE_SEARCH_I_DEFAULT_A  0.5f
#define PHASE_SEARCH_WE_RAD_S     TWO_PI_F
#define PHASE_SEARCH_TRAVEL_RAD   TWO_PI_F
#define PHASE_SEARCH_MOVE_RATIO  0.25f
#define PHASE_STABLE_DELTA_RAD   0.005f
#define PHASE_STABLE_WINDOW_S    0.10f
#define PHASE_WAIT_TIMEOUT_S     3.0f
#define PHASE_STABLE_CNT         ((uint32_t)(PHASE_STABLE_WINDOW_S / CUR_TS + 0.5f))
#define PHASE_WAIT_TIMEOUT_CNT   ((uint32_t)(PHASE_WAIT_TIMEOUT_S / CUR_TS + 0.5f))

typedef enum
{
    PHASE_IDLE = 0,
    PHASE_ALIGN,
    PHASE_SCAN,
    PHASE_HOLD,
    PHASE_DONE,
    PHASE_FAILED,

} Servo_Phase_State_e;

typedef struct
{
    Servo_Phase_State_e State;
    uint32_t Wait_Cnt;
    uint32_t Stable_Cnt;
    float Theta_Cmd;
    float Theta_Native_Pre;
    float Stable_Theta;
    float Native_Delta;
    float I_Search_A;
    int8_t Enc_Dir;
    float Theta_Off;

} Servo_Phase_T;

Servo_Phase_Config_T Servo_Phase_Config =
{
    .I_Search_A = PHASE_SEARCH_I_DEFAULT_A,
};

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

static float Native_To_Internal(float Theta_Native)
{
    if (Phase.Enc_Dir < 0)
    {
        return Angle_Wrap(-Theta_Native);
    }

    return Theta_Native;
}

static bool Angle_Stable(float Theta)
{
    if (__builtin_fabsf(Angle_Delta(Theta, Phase.Stable_Theta)) > PHASE_STABLE_DELTA_RAD)
    {
        Phase.Stable_Theta = Theta;
        Phase.Stable_Cnt = 0U;
    }
    else if (Phase.Stable_Cnt < PHASE_STABLE_CNT)
    {
        Phase.Stable_Cnt++;
    }

    return Phase.Stable_Cnt >= PHASE_STABLE_CNT;
}

static void Result_Snapshot(Servo_Phase_Result_State_e State, Servo_Phase_Fail_e Fail)
{
    Last_Result.State = State;
    Last_Result.Fail = Fail;
    Last_Result.Enc_Dir = Phase.Enc_Dir;
    Last_Result.Theta_Off = Phase.Theta_Off;
    Last_Result.I_Search_A = Phase.I_Search_A;
}

static void Fail(Servo_Phase_Fail_e Reason)
{
    Phase.State = PHASE_FAILED;
    Result_Snapshot(SERVO_PHASE_RESULT_FAIL, Reason);
}

static bool Calibration_Save(void)
{
    const uint16_t Valid_Id = PARAM_CAL_VALID;
    const uint16_t Ids[] = {
        PARAM_MOTOR_PP, PARAM_ENCODER_PROTOCOL, PARAM_ENCODER_SPI_TYPE,
        PARAM_CAL_ENC_DIR, PARAM_CAL_THETA_OFF, PARAM_CAL_VALID,
    };

    Motor_Cal.Valid = 0U;
    if (NVS_Storage_Save(&Valid_Id, 1U, NVS_INCLUDE) != 0)
    {
        Motor_Cal.Valid = 1U;
        return false;
    }

    Motor_Cal.Valid = 1U;

    if (NVS_Storage_Save(Ids, sizeof(Ids) / sizeof(Ids[0]), NVS_INCLUDE) != 0)
    {
        return false;
    }

    return true;
}

bool Servo_Phase_Start(void)
{
    if ((Encoder.Ready == 0U) || (Encoder.Fault != 0U) || (Motor_Para.Pp == 0U))
    {
        return false;
    }

    Phase = (Servo_Phase_T){ 0 };
    Last_Result = (Servo_Phase_Result_T){ 0 };
    Last_Result.State = SERVO_PHASE_RESULT_RUNNING;

    Phase.I_Search_A = Servo_Phase_Config.I_Search_A;
    Phase.Stable_Theta = Encoder.Theta_Native;
    Last_Result.I_Search_A = Phase.I_Search_A;
    Phase.State = PHASE_ALIGN;

    return true;
}

void Servo_Phase_Abort(void)
{
    if (Servo_Phase_Active())
    {
        Result_Snapshot(SERVO_PHASE_RESULT_FAIL, SERVO_PHASE_FAIL_ABORTED);
    }

    Phase.State = PHASE_IDLE;
}

bool Servo_Phase_Active(void)
{
    return (Phase.State >= PHASE_ALIGN) && (Phase.State <= PHASE_HOLD);
}

bool Servo_Phase_Apply(void)
{
    if ((Phase.State != PHASE_DONE) ||
        (Last_Result.State != SERVO_PHASE_RESULT_PASS))
    {
        Phase.State = PHASE_IDLE;
        return false;
    }

    if (!Motor_Cal_Set(Last_Result.Enc_Dir, Last_Result.Theta_Off))
    {
        Fail(SERVO_PHASE_FAIL_APPLY);
        Phase.State = PHASE_IDLE;
        return false;
    }

    if (!Calibration_Save())
    {
        Fail(SERVO_PHASE_FAIL_APPLY);
        Phase.State = PHASE_IDLE;
        return false;
    }

    Phase.State = PHASE_IDLE;
    return true;
}

const Servo_Phase_Result_T *Servo_Phase_Last_Result_Get(void)
{
    return &Last_Result;
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta)
{
    float Delta;
    float Move_Min;
    float Theta_m_Align;

    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha = 0.0f;
    *Ubeta = 0.0f;

    if ((Encoder.Valid == 0U) || (Encoder.Fault != 0U))
    {
        Fail(SERVO_PHASE_FAIL_ENCODER);
        return FAST_OFF;
    }

    switch (Phase.State)
    {
        case PHASE_ALIGN:
            *Theta_e = 0.0f;
            *Id_Ref = Phase.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native))
            {
                Phase.Theta_Cmd = 0.0f;
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.Native_Delta = 0.0f;
                Phase.State = PHASE_SCAN;
            }
            else if (++Phase.Wait_Cnt >= PHASE_WAIT_TIMEOUT_CNT)
            {
                Fail(SERVO_PHASE_FAIL_ALIGN_TIMEOUT);
                return FAST_OFF;
            }
            return FAST_CURRENT;

        case PHASE_SCAN:
            /* Keep the commanded travel unwrapped: completion is angular,
             * not a fixed elapsed-time success condition. */
            Phase.Theta_Cmd += PHASE_SEARCH_WE_RAD_S * CUR_TS;
            if (Phase.Theta_Cmd > PHASE_SEARCH_TRAVEL_RAD)
            {
                Phase.Theta_Cmd = PHASE_SEARCH_TRAVEL_RAD;
            }
            *Theta_e = Angle_Wrap(Phase.Theta_Cmd);
            *Id_Ref = Phase.I_Search_A;

            Delta = Angle_Delta(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Native_Delta += Delta;

            if (Phase.Theta_Cmd >= PHASE_SEARCH_TRAVEL_RAD)
            {
                Move_Min = PHASE_SEARCH_MOVE_RATIO * PHASE_SEARCH_TRAVEL_RAD / (float)Motor_Para.Pp;
                if (__builtin_fabsf(Phase.Native_Delta) < Move_Min)
                {
                    Last_Result.Pos_Move = Phase.Native_Delta;
                    Fail(SERVO_PHASE_FAIL_NO_POS_MOVE);
                    return FAST_OFF;
                }

                Phase.Enc_Dir = (Phase.Native_Delta > 0.0f) ? 1 : -1;
                Last_Result.Enc_Dir = Phase.Enc_Dir;
                Last_Result.Pos_Move = (float)Phase.Enc_Dir * Phase.Native_Delta;
                Phase.Stable_Theta = Encoder.Theta_Native;
                Phase.Stable_Cnt = 0U;
                Phase.Wait_Cnt = 0U;
                Phase.State = PHASE_HOLD;
            }
            return FAST_CURRENT;

        case PHASE_HOLD:
            *Theta_e = 0.0f;
            *Id_Ref = Phase.I_Search_A;

            if (Angle_Stable(Encoder.Theta_Native))
            {
                Theta_m_Align = Native_To_Internal(Encoder.Theta_Native);
                Phase.Theta_Off = Angle_Wrap(-(float)Motor_Para.Pp * Theta_m_Align);
                Phase.State = PHASE_DONE;
                Result_Snapshot(SERVO_PHASE_RESULT_PASS, SERVO_PHASE_FAIL_NONE);
                return FAST_OFF;
            }
            if (++Phase.Wait_Cnt >= PHASE_WAIT_TIMEOUT_CNT)
            {
                Fail(SERVO_PHASE_FAIL_HOLD_TIMEOUT);
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
