/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "PID.h"

extern PID_T Id_Ctrl;
extern PID_T Iq_Ctrl;

void Current_Loop_State_Reset(void);
void Current_Loop_Track(float Theta_e,
                        float Id_Ref,
                        float Iq_Ref,
                        float Ualpha,
                        float Ubeta);
void Current_Loop(float Id_Ref, float Iq_Ref, float *Ualpha, float *Ubeta);

#endif /* CURRENT_LOOP_H */
