/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTION_TYPE_H
#define MOTION_TYPE_H

#include <stdint.h>

typedef struct
{
    float Wm_Max; /* rad/s, positive magnitude */
    float Wm_Acc; /* rad/s^2, positive magnitude */
    float Wm_Dec; /* rad/s^2, positive magnitude */

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
