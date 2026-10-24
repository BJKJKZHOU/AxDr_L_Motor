/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef HANDOVER_H
#define HANDOVER_H

#include "Fast_Memory.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t Blend_Cnt;

} Handover_T;

FAST_CODE bool Handover_Blend_Run(Handover_T *Handover,
                        uint32_t Blend_Limit,
                        float Theta_Source,
                        float Theta_Obs,
                        float Id_Source,
                        float Iq_Source,
                        float *Theta_Use,
                        float *Id_Ref,
                        float *Iq_Ref);

FAST_CODE float Handover_Ramp_Zero(float Value, float Step);

#endif /* HANDOVER_H */
