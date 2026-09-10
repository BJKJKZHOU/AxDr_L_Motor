/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_IDENT_H
#define FLUX_IDENT_H

#include <stdbool.h>

#include "Motor_Type.h"

typedef struct
{
    float Flux_Wb;
    bool Valid;

} Flux_Result_T;

bool Flux_Start(float Wm_Target);
bool Flux_Active(void);
void Flux_Control(void);
Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float Ic_A,
                                float *Theta_e,
                                float *Id_Ref,
                                float *Iq_Ref,
                                float *Ualpha_V,
                                float *Ubeta_V);
const Flux_Result_T *Flux_Result_Get(void);

#endif /* FLUX_IDENT_H */
