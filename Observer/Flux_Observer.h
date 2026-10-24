/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_OBSERVER_H
#define FLUX_OBSERVER_H

#include "Fast_Memory.h"

#include <stdbool.h>

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float BW_Hz;

} Flux_Observer_Para_T;

typedef struct
{
    float Gamma;
    float Flux_Used;
    float BW_Used;

} Flux_Observer_Calc_T;

typedef struct
{
    float LambdaAlpha;
    float LambdaBeta;

    /* Active flux: Lambda - Lq * I, aligned with rotor d when
     * Flux + (Ld - Lq) * Id > 0. */
    float PsiAlpha;
    float PsiBeta;

    float Flux_Err;

} Flux_Observer_State_T;

typedef struct
{
    Flux_Observer_Para_T Para;
    Flux_Observer_Calc_T Calc;
    Flux_Observer_State_T State;

} Flux_Observer_T;

/* Recalculate after parameter changes, before Run. A live observer must receive
 * the matching Para and Calc together; prepare runtime updates outside the ISR. */
FAST_CODE bool Flux_Observer_Calc_Update(const Flux_Observer_Para_T *Para, Flux_Observer_Calc_T *Calc);

void Flux_Observer_Reset(Flux_Observer_T *Obs, float Theta_e, float Ialpha, float Ibeta);

FAST_CODE bool Flux_Observer_Run(Flux_Observer_T *Obs,
                       float Ualpha,
                       float Ubeta,
                       float Ialpha,
                       float Ibeta,
                       float Ts);

#endif /* FLUX_OBSERVER_H */
