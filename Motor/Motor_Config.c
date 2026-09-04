/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Config.h"

#include "Math.h"

Motor_Config_T Motor_Config =
{
    .Dir = 1,
};

static void Position_Reverse(int32_t Turn,
                             float Theta,
                             int32_t *Turn_Out,
                             float *Theta_Out)
{
    if (Theta == 0.0f)
    {
        *Turn_Out = -Turn;
        *Theta_Out = 0.0f;
        return;
    }

    *Turn_Out = -Turn - 1;
    *Theta_Out = TWO_PI_F - Theta;
}

float Motor_User_To_Internal(float Value)
{
    return (float)Motor_Config.Dir * Value;
}

float Motor_Internal_To_User(float Value)
{
    return (float)Motor_Config.Dir * Value;
}

void Motor_Position_User_To_Internal(int32_t Turn_User,
                                     float Theta_User,
                                     int32_t *Turn_Int,
                                     float *Theta_Int)
{
    if (Motor_Config.Dir > 0)
    {
        *Turn_Int = Turn_User;
        *Theta_Int = Theta_User;
        return;
    }

    Position_Reverse(Turn_User, Theta_User, Turn_Int, Theta_Int);
}

void Motor_Position_Internal_To_User(int32_t Turn_Int,
                                     float Theta_Int,
                                     int32_t *Turn_User,
                                     float *Theta_User)
{
    if (Motor_Config.Dir > 0)
    {
        *Turn_User = Turn_Int;
        *Theta_User = Theta_Int;
        return;
    }

    Position_Reverse(Turn_Int, Theta_Int, Turn_User, Theta_User);
}
