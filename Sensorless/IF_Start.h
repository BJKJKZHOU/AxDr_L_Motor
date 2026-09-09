/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef IF_START_H
#define IF_START_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IF_KICK = 0,
    IF_RAMP,
    IF_HOLD,
    IF_FAILED,

} IF_State_e;

typedef enum
{
    IF_KICK_CURRENT = 0,
    IF_KICK_OBSERVE,

} IF_Kick_Mode_e;

typedef struct
{
    float Iq_Min_A;
    float Iq_Max_A;
    float We_Base;
    float Acc;
    float Iq_Slew_A_S;
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

    IF_Kick_Mode_e Kick_Mode;
    float Kick_Base_We;
    float Kick_We;
    float Kick_I_Target_A;
    uint8_t Kick_Lock_Steps;

    bool Kick_Current_Valid;
    bool Kick_Phase_Valid;
    float Kick_Id_Last;
    float Kick_Iq_Last;
    float Kick_Ed_F;
    float Kick_Eq_F;
    float Kick_Phase_Last;
    float Kick_Phase_Drift;
    float Kick_IF_Phase_Travel;
    float Kick_Phase_X_Sum;
    float Kick_Phase_Y_Sum;
    uint32_t Kick_Observe_Cnt;
    uint32_t Kick_Valid_Cnt;

} IF_State_T;

typedef struct
{
    IF_Para_T Para;
    IF_State_T State;

} IF_T;

void IF_Init(IF_T *IF, float Theta_Start, float We_Start);
void IF_Target_Set(IF_T *IF, float We_Target);
void IF_Run(IF_T *IF,
            float Id_A,
            float Iq_A,
            float Ud_V,
            float Uq_V,
            float *Theta_e,
            float *Id_Ref,
            float *Iq_Ref,
            float Ts);

#endif /* IF_START_H */
