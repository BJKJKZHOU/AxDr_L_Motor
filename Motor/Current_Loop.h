/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include <stdint.h>

#include "Fast_Memory.h"
#include "LADRC.h"

#include "PID.h"

typedef enum
{
    CURRENT_CTRL_PI = 0,
    CURRENT_CTRL_LADRC,

} Current_Controller_e;

extern uint8_t Current_Controller;
extern float Control_LADRC_Bw_Hz;
extern float Control_LADRC_Eso_Bw_Hz;
extern LADRC_T Id_LADRC;
extern LADRC_T Iq_LADRC;

extern PID_T Id_Ctrl;
extern PID_T Iq_Ctrl;

void Current_Loop_State_Reset(void);
void Current_Loop_Track(float Id, float Iq, float Ud, float Uq,
                        float Ud_FF, float Uq_FF);
FAST_CODE void Current_Loop(float Id_Ref, float Iq_Ref, float U_Lim,
                            float Ud_FF, float Uq_FF,
                            float *Ud, float *Uq);

#endif /* CURRENT_LOOP_H */
