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

} Sensorless_State_e;

extern Flux_Observer_T Flux_Obs;
extern PLL_T Flux_PLL;

extern volatile float Sensorless_Theta_IF;
extern volatile float Sensorless_Theta_Use;
extern volatile float Sensorless_Id_Ref;
extern volatile float Sensorless_Iq_Ref;
extern volatile float Sensorless_Blend;
extern volatile float Sensorless_We_Obs_F;

void Sensorless_Begin(void);
void Sensorless_Stop(void);
bool Sensorless_Active(void);
bool Sensorless_Ready(void);

bool Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Id_Ref, float *Iq_Ref);
Sensorless_State_e Sensorless_State_Get(void);

#endif /* SENSORLESS_H */
