/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLUX_IDENT_H
#define FLUX_IDENT_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

typedef enum
{
    FLUX_IDLE = 0,
    FLUX_ALIGN,
    FLUX_ACCEL,
    FLUX_SETTLE,
    FLUX_MEASURE,
    FLUX_CALC,
    FLUX_DONE,
    FLUX_FAILED,
    FLUX_FINISH,

} Flux_State_e;

typedef struct
{
    float Flux_Wb;
    float Point_Max_Rel_Dev;
    float U_Util_Max;
    bool Valid;

} Flux_Result_T;

void Flux_Start(float Wm_Target);
void Flux_Reset(void);
void Flux_Fail(void);
void Flux_Control(void);
bool Flux_Active(void);
Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A, float Ib_A, float Ic_A, float *Theta_e, float *Id_Ref, float *Iq_Ref);
Flux_State_e Flux_State_Get(void);
const Flux_Result_T *Flux_Result_Get(void);

#endif /* FLUX_IDENT_H */
