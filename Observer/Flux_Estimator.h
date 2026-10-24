/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_ESTIMATOR_H
#define FLUX_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FLUX_EST_VECTOR = 0, /* Active-flux vector in the I/F frame. */
    FLUX_EST_SCALAR,     /* PM flux in the observer frame. */

} Flux_Estimator_Mode_e;

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float I_BW_Hz;
    float Est_BW_Hz;
    float We_Min;
    float Window_Update_Ratio;
    uint32_t Window_Samples;

} Flux_Estimator_Para_T;

typedef struct
{
    float Id_F;
    float Iq_F;
    float Psi_d;
    float Psi_q;
    float Flux; /* PM flux, including after vector fitting. */
    float Yd;
    float Yq;
    float Window_Num;
    float Window_Den;
    float Window_Flux;
    uint32_t Window_Count;
    bool I_Valid;
    bool Model_Valid;
    bool Estimate_Valid;
    bool Window_Ready;

} Flux_Estimator_State_T;

typedef struct
{
    Flux_Estimator_Para_T Para;
    Flux_Estimator_State_T State;

} Flux_Estimator_T;

bool Flux_Estimator_Run(Flux_Estimator_T *Est,
                        Flux_Estimator_Mode_e Mode,
                        bool Measure,
                        float Ud,
                        float Uq,
                        float Id,
                        float Iq,
                        float We,
                        float Ts);

#endif /* FLUX_ESTIMATOR_H */
