/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SENSORLESS_H
#define SENSORLESS_H

#include <stdbool.h>
#include <stdint.h>

#include "Flux_Observer.h"
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

extern Flux_Observer_T Flux_Obs;
extern PLL_T Flux_PLL;

bool Sensorless_Begin(void);
void Sensorless_Stop(void);
bool Sensorless_Active(void);

void Sensorless_Control(float We_Ref, float Iq_Min, float Iq_Max);
void Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Theta_e, float *Id_Ref, float *Iq_Ref);
Sensorless_State_e Sensorless_State_Get(void);

#endif /* SENSORLESS_H */
