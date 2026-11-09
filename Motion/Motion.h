/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef enum
{
    MOTION_TRAPEZOID = 0,
    MOTION_S_TIME,
    MOTION_S_PEAK,

} Motion_Profile_e;

typedef struct
{
    float Wm_Acc; // 加速度幅值，rad/s²。
    float Wm_Dec; // 减速度幅值，rad/s²。
    float Te_Rate; // 转矩变化率，N·m/s；0 表示直接跟随。
    uint8_t Profile; // 速度和位置轨迹类型，只能在非 RUN 状态修改。

} Motion_Config_T;

typedef struct
{
    int32_t Turn;
    float Theta;
    float Wm;
    float Am;

} Motion_Ref_T;

extern Motion_Config_T Motion_Config;

void Ramp_Run(Motion_Ref_T *Ref, float Wm_Target, float Acc, float Dec, float Ts);

#endif /* MOTION_H */
