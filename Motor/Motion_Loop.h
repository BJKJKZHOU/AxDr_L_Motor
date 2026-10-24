/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTION_LOOP_H
#define MOTION_LOOP_H

#include <stdint.h>

#include "PID.h"

typedef enum
{
    SPEED_CTRL_PI = 0,
    SPEED_CTRL_LADRC,

} Speed_Controller_e;

extern PID_T Pos_Ctrl;
/* Sig reports the active speed controller; Para/State remain PI-owned. */
extern PID_T Speed_Ctrl;
extern uint8_t Speed_Controller;
extern float Control_Speed_LADRC_Bw_Hz;

void Speed_Loop_Track(float We_Ref, float We_Fbk, float Iq, float Iq_Min, float Iq_Max);
float Position_Loop(int32_t Turn_Ref, float Theta_Ref, float Wm_Min, float Wm_Max);
float Speed_Loop(float We_Ref, float We_Fbk, float Iq_Min, float Iq_Max);
/* Normal sensored servo only; uses the live mechanical ESO without resetting it. */
float Speed_LADRC(float We_Ref, float Iq_Min, float Iq_Max);

#endif /* MOTION_LOOP_H */
