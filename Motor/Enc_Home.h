/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENC_HOME_H
#define ENC_HOME_H

#include <stdbool.h>
#include "Motor_Type.h"

typedef struct
{
    float Wm; /* 寻零速度，rad/s，符号按用户方向。 */
    float Wm_Save; /* 回零前的速度指令。 */
    Motor_Position_T Position_Save; /* 回零前的位置目标。 */
    volatile Motor_Position_T Zero; /* 触发点的内部机械位置。 */
    uint8_t Valid; /* 机械零点仅在本次上电有效。 */
} Enc_Home_T;

extern Enc_Home_T Enc_Home;

bool Enc_Home_Start(void);
/* BSP 传入触发时的原生机械角度，单位 rad。 */
void Enc_Home_Trigger(float Theta_Native);
void Enc_Home_Run(void);

#endif /* ENC_HOME_H */
