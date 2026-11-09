/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motion.h"

#include "control_params.h"

Motion_Config_T Motion_Config =
{
    .Wm_Acc = MOTION_ACC_RAD_S2,
    .Wm_Dec = MOTION_DEC_RAD_S2,
    .Te_Rate = MOTION_TE_RATE_DEFAULT,
    .Profile = MOTION_TRAPEZOID,
};

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

    /* 反向时先减速到零，下一拍再向新方向加速。 */
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
