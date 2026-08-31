/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef RS_LS_H
#define RS_LS_H

#include <stdbool.h>

#include "Motor_Type.h"

typedef struct
{
    float Rs_Ohm;
    float Ls_H;
    bool Valid;

} Rs_Ls_Result_T;

void Rs_Ls_Start(void);
void Rs_Ls_Abort(void);
bool Rs_Ls_Active(void);
Motor_Fast_Mode_e Rs_Ls_Run(float Ialpha_A,
                            float *Theta_e,
                            float *Id_Ref,
                            float *Iq_Ref,
                            float *Ualpha_V,
                            float *Ubeta_V);
const Rs_Ls_Result_T *Rs_Ls_Result_Get(void);

#endif /* RS_LS_H */
