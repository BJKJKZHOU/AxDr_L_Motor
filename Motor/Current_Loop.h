/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "PID.h"

typedef struct
{
    float Id_Kp;
    float Id_Ki;
    float Iq_Kp;
    float Iq_Ki;

} Current_Loop_Gain_T;

extern PID_T Id_Ctrl;
extern PID_T Iq_Ctrl;

void Current_Loop_State_Reset(void);
void Current_Loop_Gain_Save(Current_Loop_Gain_T *Gain);
void Current_Loop_Gain_Set_RL(float Rs, float Ld, float Lq);
void Current_Loop_Gain_Restore(const Current_Loop_Gain_T *Gain);
void Current_Loop(float Id_Ref, float Iq_Ref, float *Ualpha, float *Ubeta);

#endif /* CURRENT_LOOP_H */
