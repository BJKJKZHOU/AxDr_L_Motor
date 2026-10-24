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
    SERVO_PHASE_FAIL_NONE = 0,
    SERVO_PHASE_FAIL_ENCODER,
    SERVO_PHASE_FAIL_ABORTED,
    SERVO_PHASE_FAIL_APPLY,

} Servo_Phase_Fail_e;

typedef struct
{
    float I_Search_A;

} Servo_Phase_Config_T;

typedef struct
{
    Servo_Phase_Fail_e Fail;

    int8_t Enc_Dir;
    float Theta_Off;
    float Theta_Off_Error;

} Servo_Phase_Result_T;

extern Servo_Phase_Config_T Servo_Phase_Config;

bool Servo_Phase_Start(void);
void Servo_Phase_Abort(void);
bool Servo_Phase_Active(void);
bool Servo_Phase_Apply(void);
bool Servo_Phase_Save(void);
const Servo_Phase_Result_T *Servo_Phase_Last_Result_Get(void);

/* Difference between static zero measurements at the two ALIGN positions. */
static inline float Servo_Phase_Theta_Off_Error_Get(void)
{
    return Servo_Phase_Last_Result_Get()->Theta_Off_Error;
}

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta);

#endif /* SERVO_PHASE_H */
