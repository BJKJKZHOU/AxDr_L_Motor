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

typedef enum
{
    CTRL_TUNE_BANDWIDTH = 0,
    CTRL_TUNE_MANUAL,

} Control_Tune_Source_e;

extern float Control_Current_Bw_Hz;
extern float Control_Speed_Bw_Hz;
extern uint8_t Control_Current_Tune_Source;
extern uint8_t Control_Speed_Tune_Source;

void Current_Tuning_Update(void);
void Speed_Tuning_Update(void);
void Current_Tuning_Source_Changed(void);
void Speed_Tuning_Source_Changed(void);
void Mechanical_ESO_Tuning_Update(void);

/* Refresh runtime values that depend on the active motor model. */
void Motor_Para_Update(void);

/* Pp also changes the mechanical-to-electrical coordinate mapping. */
void Motor_Pp_Changed(void);

bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para);

#endif /* MOTOR_PARA_H */
