/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_PARA_H
#define MOTOR_PARA_H

#include <stdint.h>

#include "Motor_Type.h"

typedef enum
{
    MOTOR_PARA_RL = 1U << 0,
    MOTOR_PARA_FLUX = 1U << 1,
    MOTOR_PARA_JB = 1U << 2,
    MOTOR_PARA_PP = 1U << 3,

} Motor_Para_Change_e;

void Motor_Para_Changed(uint32_t Changed);

#endif /* MOTOR_PARA_H */
