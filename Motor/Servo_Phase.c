/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Servo_Phase.h"

#include "Encoder.h"
#include "Math.h"
#include "Protection.h"
#include "Sin_LUT.h"
#include "control_params.h"
#include "motor_params.h"

/*
 * First-version commissioning limits.
 *
 * I_Search_A is the single user-facing excitation parameter. One search
 * snapshots an effective current bounded by the firmware phase-search ceiling
 * and Motor/User current limits. Align/search/settle use 100% of that value;
 * final direction verification uses an internal fraction of the same envelope.
 */
#define PHASE_SEARCH_I_DEFAULT_A   0.5f
#define PHASE_VERIFY_I_RATIO       0.6f
#define PHASE_ALIGN_TIME_S         0.5f
#define PHASE_ALIGN_CNT            ((uint32_t)(PHASE_ALIGN_TIME_S / CUR_TS + 0.5f))
#define PHASE_SEARCH_WE_RAD_S      TWO_PI_F
#define PHASE_SEARCH_TRAVEL_RAD    TWO_PI_F
#define PHASE_SEARCH_TIME_S        (PHASE_SEARCH_TRAVEL_RAD / PHASE_SEARCH_WE_RAD_S)
#define PHASE_SEARCH_CNT           ((uint32_t)(PHASE_SEARCH_TIME_S / CUR_TS + 0.5f))
#define PHASE_SEARCH_MOVE_RATIO    0.25f
#define PHASE_OFFSET_ERR_MAX_RAD   0.25f
#define PHASE_SETTLE_TIME_S        0.25f
#define PHASE_SETTLE_CNT           ((uint32_t)(PHASE_SETTLE_TIME_S / CUR_TS + 0.5f))
#define PHASE_VERIFY_TIME_S        0.25f
#define PHASE_VERIFY_CNT           ((uint32_t)(PHASE_VERIFY_TIME_S / CUR_TS + 0.5f))
#define PHASE_VERIFY_MOVE_RAD      0.005f

typedef enum
{
    PHASE_IDLE = 0,
    PHASE_ALIGN,
    PHASE_SEARCH_POS,
    PHASE_SEARCH_NEG,
    PHASE_SETTLE,
    PHASE_VERIFY,
    PHASE_DONE,
    PHASE_FAILED,

} Servo_Phase_State_e;

typedef struct
{
    Servo_Phase_State_e State;
    uint32_t Cnt;
    float Theta_Cmd;
    float Theta_Native_Pre;
    float Native_Delta;
    float Verify_Delta;
    float I_Search_A;

    float Pos_Sin_PosDir;
    float Pos_Cos_PosDir;
    float Pos_Sin_NegDir;
    float Pos_Cos_NegDir;
    uint32_t Pos_Off_Cnt;

    float Neg_Off_Sin;
    float Neg_Off_Cos;
    uint32_t Neg_Off_Cnt;

    int8_t Enc_Dir;
    float Theta_Off_Pos;
    float Theta_Off_Neg;
    float Theta_Off_Error;
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

static void Offset_Accumulate(float Theta_m, float *Sin_Sum, float *Cos_Sum)
{
    float Theta_Off;
    float Sin;
    float Cos;

    Theta_Off = Angle_Wrap(Phase.Theta_Cmd - (float)Motor_Para.Pp * Theta_m);
    SinCos(Theta_Off, &Sin, &Cos);

    *Sin_Sum += Sin;
    *Cos_Sum += Cos;
}

static void Positive_Offset_Sample(void)
{
    Offset_Accumulate(Encoder.Theta_Native, &Phase.Pos_Sin_PosDir, &Phase.Pos_Cos_PosDir);
    Offset_Accumulate(Angle_Wrap(-Encoder.Theta_Native), &Phase.Pos_Sin_NegDir, &Phase.Pos_Cos_NegDir);
    Phase.Pos_Off_Cnt++;
}

static void Negative_Offset_Sample(void)
{
    Offset_Accumulate(Native_To_Internal(Encoder.Theta_Native), &Phase.Neg_Off_Sin, &Phase.Neg_Off_Cos);
    Phase.Neg_Off_Cnt++;
}

static void Result_Snapshot(Servo_Phase_Result_State_e State, Servo_Phase_Fail_e Fail)
{
    Last_Result.State = State;
    Last_Result.Fail = Fail;
    Last_Result.Enc_Dir = Phase.Enc_Dir;
    Last_Result.Theta_Off_Pos = Phase.Theta_Off_Pos;
    Last_Result.Theta_Off_Neg = Phase.Theta_Off_Neg;
    Last_Result.Theta_Off_Error = Phase.Theta_Off_Error;
    Last_Result.Theta_Off = Phase.Theta_Off;
    Last_Result.I_Search_A = Phase.I_Search_A;
}

static void Fail(Servo_Phase_Fail_e Reason)
{
    Phase.State = PHASE_FAILED;
    Result_Snapshot(SERVO_PHASE_RESULT_FAIL, Reason);
    Protection_Report_Set(PROT_PHASE_SEARCH_FAILED);
}

bool Servo_Phase_Start(void)
{
    float I_Search;

    if ((Encoder.Ready == 0U) || (Encoder.Fault != 0U) || (Motor_Para.Pp == 0U))
    {
        return false;
    }

    I_Search = Servo_Phase_Config.I_Search_A;

    if (!__builtin_isfinite(I_Search) || (I_Search <= 0.0f))
    {
        return false;
    }

    if (I_Search > SERVO_PHASE_I_MAX_A)
    {
        I_Search = SERVO_PHASE_I_MAX_A;
    }

    if (I_Search > Motor_Lim.I_Max)
    {
        I_Search = Motor_Lim.I_Max;
    }

    if (I_Search > User_Lim.I_Max)
    {
        I_Search = User_Lim.I_Max;
    }

    if (I_Search <= 0.0f)
    {
        return false;
    }

    Protection_Report_Clear(PROT_PHASE_SEARCH_FAILED);

    Phase = (Servo_Phase_T){ 0 };
    Last_Result = (Servo_Phase_Result_T){ 0 };
    Last_Result.State = SERVO_PHASE_RESULT_RUNNING;

    Phase.I_Search_A = I_Search;
    Last_Result.I_Search_A = I_Search;
    Phase.State = PHASE_ALIGN;
    Phase.Theta_Native_Pre = Encoder.Theta_Native;

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
    return (Phase.State >= PHASE_ALIGN) && (Phase.State <= PHASE_VERIFY);
}

bool Servo_Phase_Result_Get(int8_t *Enc_Dir, float *Theta_Off)
{
    if (Last_Result.State != SERVO_PHASE_RESULT_PASS)
    {
        return false;
    }

    *Enc_Dir = Last_Result.Enc_Dir;
    *Theta_Off = Last_Result.Theta_Off;
    return true;
}

const Servo_Phase_Result_T *Servo_Phase_Last_Result_Get(void)
{
    return &Last_Result;
}

void Servo_Phase_Clear(void)
{
    Phase = (Servo_Phase_T){ 0 };
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta)
{
    float Delta;
    float Move_Min;

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

            if (++Phase.Cnt >= PHASE_ALIGN_CNT)
            {
                Phase.Cnt = 0U;
                Phase.Theta_Cmd = 0.0f;
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.Native_Delta = 0.0f;
                Phase.State = PHASE_SEARCH_POS;
            }
            return FAST_CURRENT;

        case PHASE_SEARCH_POS:
            Phase.Theta_Cmd = Angle_Wrap(Phase.Theta_Cmd + PHASE_SEARCH_WE_RAD_S * CUR_TS);
            *Theta_e = Phase.Theta_Cmd;
            *Id_Ref = Phase.I_Search_A;

            Delta = Angle_Delta(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Native_Delta += Delta;
            Positive_Offset_Sample();

            if (++Phase.Cnt >= PHASE_SEARCH_CNT)
            {
                Move_Min = PHASE_SEARCH_MOVE_RATIO * PHASE_SEARCH_TRAVEL_RAD / (float)Motor_Para.Pp;

                if ((__builtin_fabsf(Phase.Native_Delta) < Move_Min) || (Phase.Pos_Off_Cnt == 0U))
                {
                    Last_Result.Pos_Move = Phase.Native_Delta;
                    Fail(SERVO_PHASE_FAIL_NO_POS_MOVE);
                    return FAST_OFF;
                }

                Phase.Enc_Dir = (Phase.Native_Delta > 0.0f) ? 1 : -1;
                Last_Result.Pos_Move = (float)Phase.Enc_Dir * Phase.Native_Delta;

                if (Phase.Enc_Dir > 0)
                {
                    Phase.Theta_Off_Pos =
                        Angle_Wrap(__builtin_atan2f(Phase.Pos_Sin_PosDir, Phase.Pos_Cos_PosDir));
                }
                else
                {
                    Phase.Theta_Off_Pos =
                        Angle_Wrap(__builtin_atan2f(Phase.Pos_Sin_NegDir, Phase.Pos_Cos_NegDir));
                }

                Last_Result.Enc_Dir = Phase.Enc_Dir;
                Last_Result.Theta_Off_Pos = Phase.Theta_Off_Pos;

                Phase.Cnt = 0U;
                Phase.Native_Delta = 0.0f;
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.State = PHASE_SEARCH_NEG;
            }
            return FAST_CURRENT;

        case PHASE_SEARCH_NEG:
            Phase.Theta_Cmd = Angle_Wrap(Phase.Theta_Cmd - PHASE_SEARCH_WE_RAD_S * CUR_TS);
            *Theta_e = Phase.Theta_Cmd;
            *Id_Ref = Phase.I_Search_A;

            Delta = Angle_Delta(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Native_Delta += (float)Phase.Enc_Dir * Delta;
            Negative_Offset_Sample();

            if (++Phase.Cnt >= PHASE_SEARCH_CNT)
            {
                Move_Min = PHASE_SEARCH_MOVE_RATIO * PHASE_SEARCH_TRAVEL_RAD / (float)Motor_Para.Pp;
                Last_Result.Neg_Move = Phase.Native_Delta;

                if ((Phase.Native_Delta > -Move_Min) || (Phase.Neg_Off_Cnt == 0U))
                {
                    Fail(SERVO_PHASE_FAIL_NO_NEG_MOVE);
                    return FAST_OFF;
                }

                Phase.Theta_Off_Neg = Angle_Wrap(__builtin_atan2f(Phase.Neg_Off_Sin, Phase.Neg_Off_Cos));
                Phase.Theta_Off_Error = Angle_Delta(Phase.Theta_Off_Pos, Phase.Theta_Off_Neg);

                Last_Result.Theta_Off_Neg = Phase.Theta_Off_Neg;
                Last_Result.Theta_Off_Error = Phase.Theta_Off_Error;

                if (__builtin_fabsf(Phase.Theta_Off_Error) > PHASE_OFFSET_ERR_MAX_RAD)
                {
                    Fail(SERVO_PHASE_FAIL_OFFSET_MISMATCH);
                    return FAST_OFF;
                }

                Phase.Theta_Off = Angle_Wrap(Phase.Theta_Off_Neg + 0.5f * Phase.Theta_Off_Error);
                Last_Result.Theta_Off = Phase.Theta_Off;

                Phase.Cnt = 0U;
                Phase.Native_Delta = 0.0f;
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.State = PHASE_SETTLE;
            }
            return FAST_CURRENT;

        case PHASE_SETTLE:
            *Theta_e = Phase.Theta_Cmd;
            *Id_Ref = Phase.I_Search_A;

            if (++Phase.Cnt >= PHASE_SETTLE_CNT)
            {
                Phase.Cnt = 0U;
                Phase.Verify_Delta = 0.0f;
                Phase.Theta_Native_Pre = Encoder.Theta_Native;
                Phase.State = PHASE_VERIFY;
            }
            return FAST_CURRENT;

        case PHASE_VERIFY:
            *Theta_e = Angle_Wrap((float)Motor_Para.Pp * Native_To_Internal(Encoder.Theta_Native) + Phase.Theta_Off);
            *Iq_Ref = PHASE_VERIFY_I_RATIO * Phase.I_Search_A;

            Delta = Angle_Delta(Encoder.Theta_Native, Phase.Theta_Native_Pre);
            Phase.Theta_Native_Pre = Encoder.Theta_Native;
            Phase.Verify_Delta += (float)Phase.Enc_Dir * Delta;

            if (++Phase.Cnt >= PHASE_VERIFY_CNT)
            {
                Last_Result.Verify_Move = Phase.Verify_Delta;

                if (Phase.Verify_Delta <= PHASE_VERIFY_MOVE_RAD)
                {
                    Fail(SERVO_PHASE_FAIL_VERIFY_DIR);
                    return FAST_OFF;
                }

                Phase.State = PHASE_DONE;
                Result_Snapshot(SERVO_PHASE_RESULT_PASS, SERVO_PHASE_FAIL_NONE);
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
