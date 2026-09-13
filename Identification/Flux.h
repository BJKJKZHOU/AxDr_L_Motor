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
    bool Valid;

} Flux_Result_T;

/* DWT cycle profile for Flux timing diagnosis.
 * Context_Open / Observer / IF still describe the 20 kHz fast path.
 * Coarse / Handover are retained for host/debug compatibility after their
 * supervisory work moved to the 2 kHz Motor Thread; they are not summed with
 * fast-path timing unless explicitly instrumented there again. */
typedef struct
{
    uint32_t State;
    uint32_t Context_Open_Cyc;
    uint32_t Context_Open_Max;
    uint32_t Observer_Cyc;
    uint32_t Observer_Max;
    uint32_t Coarse_Cyc;
    uint32_t Coarse_Max;
    uint32_t IF_Cyc;
    uint32_t IF_Max;
    uint32_t Handover_Cyc;
    uint32_t Handover_Max;

} Flux_Time_T;

extern volatile Flux_Time_T Flux_Time;

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
