/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_OBSERVER_H
#define FLUX_OBSERVER_H

typedef struct
{
    float Rs;
    float Ls;
    float Flux;
    float Gamma;

} Flux_Observer_Para_T;

typedef struct
{
    float LambdaAlpha;
    float LambdaBeta;

    float PsiAlpha;
    float PsiBeta;

    float Flux_Err;

} Flux_Observer_State_T;

typedef struct
{
    Flux_Observer_Para_T Para;
    Flux_Observer_State_T State;

} Flux_Observer_T;

void Flux_Observer_Reset(Flux_Observer_T *Obs, float Theta_e, float Ialpha, float Ibeta);

void Flux_Observer_Run(Flux_Observer_T *Obs, float Ualpha, float Ubeta, float Ialpha, float Ibeta, float Ts);

#endif /* FLUX_OBSERVER_H */
