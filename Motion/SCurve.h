/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SCURVE_H
#define SCURVE_H

#include <stdbool.h>

#include "Motion.h"

/* 五次速度多项式，B 为相对起始速度的 Bernstein 控制点。 */
typedef struct
{
    float Wm;
    float B[6];
    float T;

} SCurve_Seg_T;

typedef struct
{
    SCurve_Seg_T Seg[4]; // 加速度回零、加速、匀速、减速。
    float Time;
    float Time_Rem;
    float Pos_Rem; // 保留小于单圈角度分辨率的位置增量。
    int32_t Turn; // 上次规划的位置目标圈数，与 Theta 配对。
    float Theta;
    float Wm; // 速度目标，或位置模式的速度上限。
    float Acc;
    float Dec;
    uint8_t Profile;
    uint8_t Count;
    bool Position;
    bool To_Goal;

} SCurve_T;

/* 使能时清零。改目标时继承规划位置、速度和加速度；Ref.Am 为瞬时加速度。
 * 速度规划也累计位置，用于位置模式 Stop；Count 和速度都为零才表示停止。 */
void SCurve_Position(SCurve_T *S, Motion_Ref_T *Ref,
                     int32_t Turn, float Theta, float Wm_Max,
                     float Acc, float Dec, uint8_t Profile, float Ts);
void SCurve_Speed(SCurve_T *S, Motion_Ref_T *Ref, float Wm,
                  float Acc, float Dec, uint8_t Profile, float Ts);

#endif /* SCURVE_H */
