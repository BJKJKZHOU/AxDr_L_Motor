/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_PARA_H
#define MOTOR_PARA_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef enum
{
    MOTOR_PARA_RL = 1U << 0,
    MOTOR_PARA_FLUX = 1U << 1,
    MOTOR_PARA_JB = 1U << 2,
    MOTOR_PARA_PP = 1U << 3,

} Motor_Para_Change_e;

typedef struct
{
    float Iq_Start_A;
    float Iq_Max_A;
    float We_Base;
    float Acc;
    float U_Budget_V;
    bool Valid;

} Motor_IF_Para_T;

void Motor_Para_Changed(uint32_t Changed);
bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para);

#endif /* MOTOR_PARA_H */
