/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

typedef struct
{
    int8_t Dir; /* +1: user positive = internal positive, -1: reversed */
    float Align_Current_A; /* A, rotor alignment current used before I/F startup */
    float IF_Current_A; /* A, q-axis current used by standard I/F startup */
    float RL_I_Peak_A; /* A, RL injection target: DC bias plus AC amplitude */

} Motor_Config_T;

extern Motor_Config_T Motor_Config;

float Motor_User_To_Internal(float Value);
float Motor_Internal_To_User(float Value);
void Motor_Position_User_To_Internal(int32_t Turn_User,
                                     float Theta_User,
                                     int32_t *Turn_Int,
                                     float *Theta_Int);
void Motor_Position_Internal_To_User(int32_t Turn_Int,
                                     float Theta_Int,
                                     int32_t *Turn_User,
                                     float *Theta_User);

#endif /* MOTOR_CONFIG_H */
