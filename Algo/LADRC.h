/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef LADRC_H
#define LADRC_H

#include "Fast_Memory.h"

/* First-order plant: d(Fbk)/dt = F0 + B0 * U + disturbance.
 * A current loop uses B0 = 1/L (A/s/V), F0 in A/s, and U in V.
 * One LADRC_T has one second-order LESO; dq control uses two instances.
 */
typedef struct
{
    struct
    {
        float Kp; /* LSEF feedback gain, rad/s */
        float Wo; /* LESO pole frequency, rad/s */
        float B0; /* Input gain, e.g. 1/L */
    } Para;

    struct
    {
        float Ref;
        float Fbk;
        float Ref_Dot; /* Reference derivative; zero when unused */
        float F0; /* Known disturbance in state derivative units */
        float U_Applied; /* Applied input aligned with this feedback sample */
        float Err;
        float Out; /* Full control input, before caller's output limit */
    } Sig;

    struct
    {
        float Z1; /* Plant output estimate */
        float Z2; /* Unknown disturbance estimate */
    } State;

} LADRC_T;

FAST_CODE void LADRC_Run(LADRC_T *Ctrl, float Ts);

#endif /* LADRC_H */
