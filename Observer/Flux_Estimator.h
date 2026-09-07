/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_ESTIMATOR_H
#define FLUX_ESTIMATOR_H

#include <stdbool.h>

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float I_BW_Hz;
    float Est_BW_Hz;
    float We_Min;

} Flux_Estimator_Para_T;

typedef struct
{
    float Id_F;
    float Iq_F;
    float Psi_d;
    float Psi_q;
    float Flux;
    float Yd;
    float Yq;
    bool I_Valid;
    bool Model_Valid;

} Flux_Estimator_State_T;

typedef struct
{
    Flux_Estimator_Para_T Para;
    Flux_Estimator_State_T State;

} Flux_Estimator_T;

void Flux_Estimator_Reset(Flux_Estimator_T *Est);
void Flux_Estimator_Current_Reset(Flux_Estimator_T *Est);

bool Flux_Estimator_Model_Run(Flux_Estimator_T *Est,
                              float Ud,
                              float Uq,
                              float Id,
                              float Iq,
                              float We,
                              float Ts);

bool Flux_Estimator_Vector_Update(Flux_Estimator_T *Est, float We, float Ts);
bool Flux_Estimator_Scalar_Update(Flux_Estimator_T *Est, float We, float Ts);

#endif /* FLUX_ESTIMATOR_H */
