/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef DPCC_H
#define DPCC_H

#include "Fast_Memory.h"

/* Current is sampled at PWM centre; a new voltage loads at the next bottom.
 * A/B describe the RL response with coupling/back EMF held over each interval.
 * Delay coefficients cover Ts/2, control coefficients cover one full Ts. */
typedef struct
{
    float Ad;
    float Aq;
    float Ad_Delay;
    float Aq_Delay;
    float Bd_Delay;
    float Bq_Delay;
    float Inv_Bd;
    float Inv_Bq;

} DPCC_Para_T;

DPCC_Para_T DPCC_Para_Calc(float Rs, float Ld, float Lq, float Ts);

/* Ud/Uq are the previous limited voltage vector, expressed at the sample
 * angle. We_Ld, We_Lq and Bemf are We*Ld, We*Lq and We*Flux; Theta_Step=We*Ts.
 * Outputs are total voltages in the sample frame, ready for the common
 * voltage-vector limit and inverse Park. The algorithm has no retained state. */
FAST_CODE void DPCC_Run(const DPCC_Para_T *Para,
                       float Id_Ref, float Iq_Ref, float Id, float Iq,
                       float Ud, float Uq, float We_Ld, float We_Lq,
                       float Bemf, float Theta_Step, float *Ud_Ref, float *Uq_Ref);

#endif /* DPCC_H */
