/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SENSORLESS_H
#define SENSORLESS_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"
#include "PLL.h"

typedef enum
{
    SL_ALIGN = 0,
    SL_IF,
    SL_IF_TO_OBS,
    SL_OBS,
    SL_OBS_TO_IF,
    SL_FAILED,
    SL_IDLE,

} Sensorless_State_e;

/* Normal SENSORLESS_SPEED mode PLL runtime.
 *
 * PARAM_OBS_THETA / PARAM_OBS_WE intentionally remain bound to the normal
 * sensorless Flux-Observer PLL. Identification owns a separate Ident_PLL
 * instance and may run concurrently in a different workflow context. */
extern PLL_T Sensorless_PLL;

/* Stable compatibility name used by the generated Parameter binding. */
#define Flux_PLL Sensorless_PLL

bool Sensorless_Begin(void);
void Sensorless_Stop_Request(void);
void Sensorless_Stop(void);
bool Sensorless_Active(void);

void Sensorless_Control(float We_Ref, float Iq_Min, float Iq_Max);
void Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Theta_e, float *Id_Ref, float *Iq_Ref);
Sensorless_State_e Sensorless_State_Get(void);

#endif /* SENSORLESS_H */
