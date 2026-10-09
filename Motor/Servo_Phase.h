/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SERVO_PHASE_H
#define SERVO_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef enum
{
    SERVO_PHASE_RESULT_NONE = 0,
    SERVO_PHASE_RESULT_RUNNING,
    SERVO_PHASE_RESULT_PASS,
    SERVO_PHASE_RESULT_FAIL,

} Servo_Phase_Result_State_e;

typedef enum
{
    SERVO_PHASE_FAIL_NONE = 0,
    SERVO_PHASE_FAIL_ENCODER,
    SERVO_PHASE_FAIL_NO_POS_MOVE,
    SERVO_PHASE_FAIL_NO_NEG_MOVE,
    SERVO_PHASE_FAIL_VERIFY_DIR,
    SERVO_PHASE_FAIL_ABORTED,
    SERVO_PHASE_FAIL_APPLY,
    SERVO_PHASE_FAIL_ALIGN_TIMEOUT,
    SERVO_PHASE_FAIL_HOLD_TIMEOUT,

} Servo_Phase_Fail_e;

typedef struct
{
    float I_Search_A;

} Servo_Phase_Config_T;

typedef struct
{
    Servo_Phase_Result_State_e State;
    Servo_Phase_Fail_e Fail;

    int8_t Enc_Dir;
    float Theta_Off;

    float Pos_Move;
    float Neg_Move;
    float Verify_Move;

    float I_Search_A;

} Servo_Phase_Result_T;

extern Servo_Phase_Config_T Servo_Phase_Config;

bool Servo_Phase_Start(void);
void Servo_Phase_Abort(void);
bool Servo_Phase_Active(void);
bool Servo_Phase_Apply(void);
const Servo_Phase_Result_T *Servo_Phase_Last_Result_Get(void);

static inline bool Servo_Phase_Result_Valid(void)
{
    return Servo_Phase_Last_Result_Get()->State == SERVO_PHASE_RESULT_PASS;
}

/* Legacy host diagnostic retained until the generated parameter dictionary is
 * regenerated; dynamic scan offset mismatch is no longer part of calibration. */
static inline float Servo_Phase_Theta_Off_Error_Get(void)
{
    return 0.0f;
}

static inline float Servo_Phase_Pos_Move_Get(void)
{
    return Servo_Phase_Last_Result_Get()->Pos_Move;
}

static inline float Servo_Phase_Neg_Move_Get(void)
{
    return Servo_Phase_Last_Result_Get()->Neg_Move;
}

static inline float Servo_Phase_Verify_Move_Get(void)
{
    return Servo_Phase_Last_Result_Get()->Verify_Move;
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta);

#endif /* SERVO_PHASE_H */
