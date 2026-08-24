/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTION_LOOP_H
#define MOTION_LOOP_H

#include <stdint.h>

#include "PID.h"

extern PID_T Pos_Ctrl;
extern PID_T Speed_Ctrl;

float Speed_Profile(float Wm_Target, float Wm_Ref, float Acc, float Dec);
float Position_Loop(int32_t Turn_Ref, float Theta_Ref, float Wm_Min, float Wm_Max);
float Speed_Loop(float We_Ref, float We_Fbk, float Iq_Min, float Iq_Max);

#endif /* MOTION_LOOP_H */
