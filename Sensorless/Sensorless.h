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

} Sensorless_State_e;

typedef enum
{
    SENSORLESS_REJECT_NONE = 0U,
    SENSORLESS_REJECT_WE = 1U << 0,
    SENSORLESS_REJECT_PLL = 1U << 1,
    SENSORLESS_REJECT_FLUX = 1U << 2,

} Sensorless_Reject_e;

typedef struct
{
    float We_IF;
    float We_Obs_F;
    float We_Err;
    float PLL_Err;
    float Flux_Ratio;
    float Theta_Err;
    float Stable_s;
    float Stable_Max_s;
    uint8_t Reject;
    uint8_t Reject_Seen;

} Sensorless_Diag_T;

extern Flux_Observer_T Flux_Obs;
extern PLL_T Flux_PLL;

extern volatile float Sensorless_Theta_IF;
extern volatile float Sensorless_Theta_Use;
extern volatile float Sensorless_Id_Ref;
extern volatile float Sensorless_Iq_Ref;
extern volatile float Sensorless_Blend;
extern volatile float Sensorless_We_Obs_F;

bool Sensorless_Begin(void);
void Sensorless_Stop(void);
bool Sensorless_Active(void);
bool Sensorless_Ready(void);
bool Sensorless_Shadow_Set(bool Enable);
bool Sensorless_Shadow_Get(void);
bool Sensorless_PLL_BW_Set(float Bw_Hz);
float Sensorless_PLL_BW_Get(void);

bool Sensorless_Run(float Ia_A, float Ib_A, float We_Ref, float *Theta_e, float *Id_Ref, float *Iq_Ref);
Sensorless_State_e Sensorless_State_Get(void);
const Sensorless_Diag_T *Sensorless_Diag_Get(void);

#endif /* SENSORLESS_H */
