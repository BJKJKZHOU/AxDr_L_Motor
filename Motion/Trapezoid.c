/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Trapezoid.h"

#include <float.h>

#include "Math.h"
#include "Ramp.h"

static float Abs(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Position_Error(int32_t Turn_Target,
                            float Theta_Target,
                            int32_t Turn_Ref,
                            float Theta_Ref)
{
    int32_t Turn_Err;

    Turn_Err = Turn_Target - Turn_Ref;
    return (float)Turn_Err * TWO_PI_F + Theta_Target - Theta_Ref;
}

static void Position_Add(Motion_Ref_T *Ref, float Delta)
{
    Ref->Theta += Delta;

    while (Ref->Theta >= TWO_PI_F)
    {
        Ref->Theta -= TWO_PI_F;
        Ref->Turn++;
    }

    while (Ref->Theta < 0.0f)
    {
        Ref->Theta += TWO_PI_F;
        Ref->Turn--;
    }
}

void Trapezoid_Reset(Motion_Ref_T *Ref, int32_t Turn, float Theta, float Wm)
{
    Ref->Turn = Turn;
    Ref->Theta = Theta;
    Ref->Wm = Wm;
    Ref->Am = 0.0f;
}

void Trapezoid_Run(Motion_Ref_T *Ref,
                   int32_t Turn_Target,
                   float Theta_Target,
                   float Wm_Max,
                   float Acc,
                   float Dec,
                   float Ts)
{
    float Err;
    float Dir;
    float Wm_Old;
    float Wm;
    float Wm_Peak;
    float Time;
    float Step;
    float Ta;
    float Tv;
    float Td;
    float Delta;
    float Pos_Tol;

    if ((Wm_Max < 0.0f) || (Acc <= 0.0f) || (Dec <= 0.0f) || (Ts <= 0.0f))
    {
        Ref->Am = 0.0f;
        return;
    }

    Err = Position_Error(Turn_Target, Theta_Target, Ref->Turn, Ref->Theta);
    Wm_Old = Ref->Wm;
    Time = Ts;

    if (Wm_Max == 0.0f)
    {
        (void)Trapezoid_Stop(Ref, Dec, Ts);
        return;
    }

    /* The stopping boundary is repeatedly reconstructed from float turn/angle
     * values. Allow roundoff, not a motion-scale following-error deadband. */
    Pos_Tol = 4.0f * FLT_EPSILON * (Abs(Err) + TWO_PI_F);
    if ((Abs(Err) <= Pos_Tol) && (Abs(Wm_Old) <= Dec * Ts))
    {
        Ref->Turn = Turn_Target;
        Ref->Theta = Theta_Target;
        Ref->Wm = 0.0f;
        Ref->Am = -Wm_Old / Ts;
        return;
    }

    /* A changed target can be behind the stopping point. Brake before returning;
     * forcing the reference onto that target would violate the deceleration limit. */
    if ((Ref->Wm * Err < 0.0f) || (Ref->Wm * Ref->Wm > 2.0f * Dec * (Abs(Err) + Pos_Tol)))
    {
        Wm = Ref->Wm;
        Step = Abs(Wm) / Dec;
        if (Time < Step)
        {
            Ref->Wm += (Wm > 0.0f) ? -Dec * Time : Dec * Time;
            Position_Add(Ref, 0.5f * (Wm + Ref->Wm) * Time);
            Ref->Am = (Ref->Wm - Wm_Old) / Ts;
            return;
        }

        Position_Add(Ref, 0.5f * Wm * Step);
        Ref->Wm = 0.0f;
        Time -= Step;
        Err = Position_Error(Turn_Target, Theta_Target, Ref->Turn, Ref->Theta);
    }

    Dir = (Err >= 0.0f) ? 1.0f : -1.0f;
    Wm = Dir * Ref->Wm;

    /* Lowering the cruise limit must not make the reference velocity jump. */
    if (Wm > Wm_Max)
    {
        Step = (Wm - Wm_Max) / Dec;
        if (Time < Step)
        {
            Ref->Wm = Dir * (Wm - Dec * Time);
            Position_Add(Ref, Dir * (Wm * Time - 0.5f * Dec * Time * Time));
            Ref->Am = (Ref->Wm - Wm_Old) / Ts;
            return;
        }

        Position_Add(Ref, Dir * 0.5f * (Wm + Wm_Max) * Step);
        Ref->Wm = Dir * Wm_Max;
        Wm = Wm_Max;
        Time -= Step;
        Err = Position_Error(Turn_Target, Theta_Target, Ref->Turn, Ref->Theta);
    }

    /* Solve the remaining trapezoid in the travel direction. Evaluate position
     * and velocity at the same instant, including switches inside this sample. */
    Err *= Dir;
    Wm_Peak = __builtin_sqrtf((2.0f * Acc * Dec * Err + Dec * Wm * Wm) / (Acc + Dec));
    if (Wm_Peak > Wm_Max)
    {
        Wm_Peak = Wm_Max;
    }
    if (Wm_Peak < Wm)
    {
        Wm_Peak = Wm;
    }

    Ta = (Wm_Peak - Wm) / Acc;
    Td = Wm_Peak / Dec;
    Delta = Err - 0.5f * (Wm + Wm_Peak) * Ta - 0.5f * Wm_Peak * Td;
    Tv = ((Delta > 0.0f) && (Wm_Peak > 0.0f)) ? Delta / Wm_Peak : 0.0f;

    if (Time >= Ta + Tv + Td)
    {
        Ref->Turn = Turn_Target;
        Ref->Theta = Theta_Target;
        Ref->Wm = 0.0f;
    }
    else if (Time < Ta)
    {
        Position_Add(Ref, Dir * (Wm * Time + 0.5f * Acc * Time * Time));
        Ref->Wm = Dir * (Wm + Acc * Time);
    }
    else if (Time < Ta + Tv)
    {
        Delta = 0.5f * (Wm + Wm_Peak) * Ta + Wm_Peak * (Time - Ta);
        Position_Add(Ref, Dir * Delta);
        Ref->Wm = Dir * Wm_Peak;
    }
    else
    {
        Step = Time - Ta - Tv;
        /* Integrate only this sample, including any acceleration/cruise part.
         * Subtracting the full stopping distance from the target loses angle
         * precision and makes normalization time grow with that distance. */
        if (Err > TWO_PI_F)
        {
            Delta = 0.5f * (Wm + Wm_Peak) * Ta + Wm_Peak * Tv +
                    (Wm_Peak - 0.5f * Dec * Step) * Step;
            Position_Add(Ref, Dir * Delta);
            /* Keep speed on the stopping boundary after position rounding. */
            Err = Dir * Position_Error(Turn_Target, Theta_Target, Ref->Turn, Ref->Theta);
            Ref->Wm = (Err > 0.0f) ? Dir * __builtin_sqrtf(2.0f * Dec * Err) : 0.0f;
        }
        else
        {
            /* Within one turn, endpoint anchoring avoids accumulated roundoff
             * without large angles or distance-dependent normalization loops. */
            Ref->Wm = Dir * (Wm_Peak - Dec * Step);
            Ref->Turn = Turn_Target;
            Ref->Theta = Theta_Target;
            Position_Add(Ref, -Dir * Ref->Wm * Ref->Wm / (2.0f * Dec));
        }
    }

    Ref->Am = (Ref->Wm - Wm_Old) / Ts;
}

bool Trapezoid_Stop(Motion_Ref_T *Ref, float Dec, float Ts)
{
    float Wm_Old;
    float Step;
    float Delta;

    if ((Dec <= 0.0f) || (Ts <= 0.0f))
    {
        Ref->Am = 0.0f;
        return false;
    }

    Wm_Old = Ref->Wm;
    Step = Abs(Wm_Old) / Dec;
    if (Step > Ts)
    {
        Step = Ts;
    }
    Ramp_Run(Ref, 0.0f, Dec, Dec, Ts);
    Delta = 0.5f * (Wm_Old + Ref->Wm) * Step;
    Position_Add(Ref, Delta);

    return (Ref->Wm == 0.0f);
}
