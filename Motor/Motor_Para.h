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

typedef struct
{
    float Id_Kp;
    float Id_Ki;
    float Iq_Kp;
    float Iq_Ki;

} Current_Manual_T;

typedef struct
{
    float Kp;
    float Ki;

} Speed_Manual_T;

extern Current_Manual_T Current_Manual;
extern Speed_Manual_T Speed_Manual;

extern float Control_Current_Bw_Hz;
extern float Control_Speed_Bw_Hz;
extern uint8_t Control_Current_Tune_Source;
extern uint8_t Current_FF_Enable;
extern uint8_t Control_Speed_Tune_Source;

void Current_Tuning_Update(void);
void Speed_Tuning_Update(void);
/* Validate a candidate model/ESO bandwidth before publishing either one.
 * On failure, active parameters, coefficients and observer state are unchanged. */
bool Motor_Para_Update(const Motor_Para_T *Para, float Eso_Bw_Hz);

bool Motor_IF_Para_Build(float Vbus_V, float I_Max_A, Motor_IF_Para_T *Para);

#endif /* MOTOR_PARA_H */
