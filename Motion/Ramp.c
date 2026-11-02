/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Ramp.h"

static float Approach(float Target, float Ref, float Step)
{
    if (Ref < Target)
    {
        Ref += Step;
        if (Ref > Target)
        {
            Ref = Target;
        }
    }
    else if (Ref > Target)
    {
        Ref -= Step;
        if (Ref < Target)
        {
            Ref = Target;
        }
    }

    return Ref;
}

void Ramp_Run(Motion_Ref_T *Ref, float Wm_Target, float Acc, float Dec, float Ts)
{
    float Wm_Old;
    float Wm_New;
    float Step;

    Wm_Old = Ref->Wm;

    if (((Wm_Old > 0.0f) && (Wm_Target < 0.0f)) || ((Wm_Old < 0.0f) && (Wm_Target > 0.0f)))
    {
        Wm_New = Approach(0.0f, Wm_Old, Dec * Ts);
    }
    else
    {
        if (((Wm_Old >= 0.0f) && (Wm_Target > Wm_Old)) || ((Wm_Old <= 0.0f) && (Wm_Target < Wm_Old)))
        {
            Step = Acc * Ts;
        }
        else
        {
            Step = Dec * Ts;
        }

        Wm_New = Approach(Wm_Target, Wm_Old, Step);
    }

    Ref->Wm = Wm_New;
    Ref->Am = (Wm_New - Wm_Old) / Ts;
}
