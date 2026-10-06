/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef IF_START_H
#define IF_START_H

#include "Fast_Memory.h"

#include <stdint.h>

typedef enum
{
    IF_RAMP = 0,
    IF_HOLD,
    IF_FAILED,

} IF_State_e;

typedef struct
{
    float Iq_Min_A;
    float Iq_Max_A;
    float We_Base;
    float Acc;
    float Iq_Slew_A_S;

    /* Kept for caller compatibility; open-loop I/F no longer consumes the
     * motor model for low-speed motion qualification. */
    float Rs_Ohm;
    float Ld_H;
    float Lq_H;

} IF_Para_T;

typedef struct
{
    IF_State_e Mode;
    float Theta_e;
    float We;
    float We_Target;
    float Iq;
    float Iq_Work_A;

} IF_State_T;

typedef struct
{
    IF_Para_T Para;
    IF_State_T State;

} IF_T;

void IF_Init(IF_T *IF, float Theta_Start, float We_Start);
FAST_CODE void IF_Target_Set(IF_T *IF, float We_Target);
FAST_CODE void IF_Run(IF_T *IF,
            float Id_A,
            float Iq_A,
            float Ud_V,
            float Uq_V,
            float *Theta_e,
            float *Id_Ref,
            float *Iq_Ref,
            float Ts);

#endif /* IF_START_H */
