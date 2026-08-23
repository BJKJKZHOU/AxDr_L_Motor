/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Align.h"

static uint32_t Align_Cnt = 0U;
static float Align_U_V = 0.0f;

void Align_Reset(void)
{
    Align_Cnt = 0U;
    Align_U_V = 0.0f;
}

bool Align_Current(float Id_A, uint32_t Hold_Cnt, float *Id_Ref, float *Iq_Ref)
{
    *Id_Ref = Id_A;
    *Iq_Ref = 0.0f;

    if (Align_Cnt < Hold_Cnt)
    {
        Align_Cnt++;
    }

    return Align_Cnt >= Hold_Cnt;
}

bool Align_Voltage(float U_Step_V,
                   float U_Max_V,
                   float I_Limit_A,
                   uint32_t Hold_Cnt,
                   float Ialpha_A,
                   float *Ualpha,
                   float *Ubeta)
{
    float I_Abs;

    I_Abs = (Ialpha_A >= 0.0f) ? Ialpha_A : -Ialpha_A;

    if ((I_Abs < I_Limit_A) && (Align_U_V < U_Max_V))
    {
        Align_U_V += U_Step_V;

        if (Align_U_V > U_Max_V)
        {
            Align_U_V = U_Max_V;
        }
    }

    *Ualpha = Align_U_V;
    *Ubeta = 0.0f;

    if ((I_Abs >= I_Limit_A) || (Align_U_V >= U_Max_V))
    {
        if (Align_Cnt < Hold_Cnt)
        {
            Align_Cnt++;
        }
    }
    else
    {
        Align_Cnt = 0U;
    }

    return Align_Cnt >= Hold_Cnt;
}
