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
    float I_Search_A;
    volatile Servo_Phase_State_e Step;
    int8_t Enc_Dir;
    float Theta_Off;
    float Theta_Off_Error;

} Servo_Phase_T;

/* 算法推进寻相步骤并生成结果，Enc_Control 负责取消和应用。 */
extern Servo_Phase_T Servo_Phase;

bool Servo_Phase_Start(bool Use_Harmonic);

Motor_Fast_Mode_e Servo_Phase_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref);

#endif /* SERVO_PHASE_H */
