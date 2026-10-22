/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_IDENT_H
#define FLUX_IDENT_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef struct
{
    float Flux_Wb;
    uint8_t Valid;

} Flux_Result_T;

extern Flux_Result_T Flux_Result;

bool Flux_Start(float Wm_Target);
bool Flux_Active(void);
void Flux_Control(void);
Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float *Theta_e,
                                float *Id_Ref,
                                float *Iq_Ref,
                                float *Ualpha_V,
                                float *Ubeta_V);

#endif /* FLUX_IDENT_H */
