/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_PARA_H
#define MOTOR_PARA_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef struct
{
    float Iq_Start_A;
    float Iq_Max_A;
    float We_Base;
    float Acc;
    float U_Budget_V;
    bool Valid;

} Motor_IF_Para_T;

/* Host tuning uses physical design bandwidths; active gains are derived from Motor_Para. */
extern float Control_Current_Bw_Hz;
extern float Control_Speed_Bw_Hz;

/* Rebuild all control/model values derived from the current Motor_Para and tuning inputs. */
void Motor_Para_Update(void);

/* Pp also changes the mechanical-to-electrical coordinate mapping. */
void Motor_Pp_Changed(void);

bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para);

#endif /* MOTOR_PARA_H */
