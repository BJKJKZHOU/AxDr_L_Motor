/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef IF_START_H
#define IF_START_H

#include <stdbool.h>

typedef enum
{
    IF_RAMP = 0,
    IF_HOLD,

} IF_State_e;

typedef struct
{
    float Iq_Run_A;
    float We_Base;
    float Acc;
    float Breakaway_We_Ratio;
    float Breakaway_Acc_Ratio;
    float Iq_Slew_A_S;

} IF_Para_T;

typedef struct
{
    IF_State_e Mode;
    float Theta_e;
    float We;
    float We_Target;
    float Iq;

    float We_Breakaway;
    float Acc_Breakaway;
    float Acc_Run;
    bool Initial_Profile_Active;
    bool Breakaway_Active;

} IF_State_T;

typedef struct
{
    IF_Para_T Para;
    IF_State_T State;

} IF_T;

void IF_Init(IF_T *IF, float Theta_Start, float We_Start);
void IF_Target_Set(IF_T *IF, float We_Target);
void IF_Run(IF_T *IF, float *Theta_e, float *Id_Ref, float *Iq_Ref, float Ts);

#endif /* IF_START_H */
