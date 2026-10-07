/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTION_TYPE_H
#define MOTION_TYPE_H

#include <stdint.h>

typedef enum
{
    MOTION_TRAPEZOID = 0,
    MOTION_S_TIME,
    MOTION_S_PEAK,

} Motion_Profile_e;

typedef struct
{
    float Wm_Acc; /* rad/s^2, positive magnitude */
    float Wm_Dec; /* rad/s^2, positive magnitude */
    float Te_Rate; /* N*m/s, TORQUE only; zero bypasses the ramp */
    uint8_t Profile; /* SPEED/POSITION; selectable only outside RUN */

} Motion_Config_T;

typedef struct
{
    int32_t Turn;
    float Theta;
    float Wm;
    float Am;

} Motion_Ref_T;

extern Motion_Config_T Motion_Config;

#endif /* MOTION_TYPE_H */
