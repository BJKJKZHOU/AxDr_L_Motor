/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef VF_H
#define VF_H

typedef struct
{
    float U_Max_V;
    float U_Slew_V_S;
    float We_Acc;

} VF_Para_T;

typedef struct
{
    float Theta_e;
    float We;
    float U;

} VF_State_T;

typedef struct
{
    VF_Para_T Para;
    VF_State_T State;

} VF_T;

void VF_Run(VF_T *VF,
            float We_Target,
            float U_Target_V,
            float *Ualpha_V,
            float *Ubeta_V,
            float Ts);

#endif /* VF_H */
