/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Voltage_Mod.h"

#include "Math.h"

void SVPWM_Calc(float Ualpha, float Ubeta, float Vbus, float *DutyA, float *DutyB, float *DutyC)
{
    float Ua;
    float Ub;
    float Uc;
    float Umax;
    float Umin;
    float Uoff;
    float Inv_Vbus;
    float Da;
    float Db;
    float Dc;

    if (Vbus <= 0.0f)
    {
        *DutyA = 0.5f;
        *DutyB = 0.5f;
        *DutyC = 0.5f;
        return;
    }

    Ua = Ualpha;
    Ub = -0.5f * Ualpha + SQRT3_HALF_F * Ubeta;
    Uc = -0.5f * Ualpha - SQRT3_HALF_F * Ubeta;

    Umax = Ua;
    if (Ub > Umax)
    {
        Umax = Ub;
    }
    if (Uc > Umax)
    {
        Umax = Uc;
    }

    Umin = Ua;
    if (Ub < Umin)
    {
        Umin = Ub;
    }
    if (Uc < Umin)
    {
        Umin = Uc;
    }

    Uoff = -0.5f * (Umax + Umin);

    Inv_Vbus = 1.0f / Vbus;
    Da = 0.5f + (Ua + Uoff) * Inv_Vbus;
    Db = 0.5f + (Ub + Uoff) * Inv_Vbus;
    Dc = 0.5f + (Uc + Uoff) * Inv_Vbus;

    /* Keep physical duty bounds during overmodulation without helper calls. */
    if (Da > 1.0f)
    {
        Da = 1.0f;
    }
    if (Da < 0.0f)
    {
        Da = 0.0f;
    }
    if (Db > 1.0f)
    {
        Db = 1.0f;
    }
    if (Db < 0.0f)
    {
        Db = 0.0f;
    }
    if (Dc > 1.0f)
    {
        Dc = 1.0f;
    }
    if (Dc < 0.0f)
    {
        Dc = 0.0f;
    }

    *DutyA = Da;
    *DutyB = Db;
    *DutyC = Dc;
}
