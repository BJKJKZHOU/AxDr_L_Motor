/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef JB_H
#define JB_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef struct
{
    float J_Kgm2;
    float B_Nms;
    uint8_t Valid;

} JB_Result_T;

extern JB_Result_T JB_Result;

bool JB_Start(void);
void JB_Control(void);
Motor_Fast_Mode_e JB_Fast_Run(float Ia_A,
                              float Ib_A,
                              float *Theta_e,
                              float *Id_Ref,
                              float *Iq_Ref,
                              float *Ualpha_V,
                              float *Ubeta_V);

#endif /* JB_H */
