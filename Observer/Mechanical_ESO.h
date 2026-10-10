/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MECHANICAL_ESO_H
#define MECHANICAL_ESO_H

#include "Fast_Memory.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float Theta; /* Single-turn mechanical angle, [0, 2*pi) rad. */
    float Wm;
    float Td;
    float Error;

} Mechanical_ESO_State_T;

typedef struct
{
    float Kt_Over_J;
    float Inv_J;
    float B_Over_J;
    float L1;
    float L2;
    float L3;
    uint8_t Valid;

} Mechanical_ESO_Para_T;

typedef struct
{
    Mechanical_ESO_Para_T Para;
    Mechanical_ESO_State_T State;

} Mechanical_ESO_T;

extern Mechanical_ESO_T Mechanical_ESO;
extern float Mechanical_ESO_Bw_Hz;

/* Compute coefficients; reject invalid physical inputs or non-finite results. */
bool Mechanical_ESO_Para_Build(Mechanical_ESO_Para_T *Result,
                                float J, float B, float Kt, float Wo);
/* Theta_Meas is the direction-corrected single-turn mechanical angle. */
FAST_CODE void Mechanical_ESO_Run(float Theta_Meas, bool Position_Valid, float Iq);
float Mechanical_ESO_Wm_Get(void);

#endif /* MECHANICAL_ESO_H */
