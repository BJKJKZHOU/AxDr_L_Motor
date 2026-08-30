/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_CAL_H
#define MOTOR_CAL_H

#include <stdbool.h>
#include <stdint.h>

bool Motor_Cal_Set(int8_t Enc_Dir, float Theta_Off);

#endif /* MOTOR_CAL_H */
