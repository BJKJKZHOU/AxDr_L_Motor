/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "SCurve.h"

#include <float.h>
#include <math.h>

#include "Math.h"

static float Max(float A, float B)
{
    return (A > B) ? A : B;
}

static float Position_Error(int32_t Turn, float Theta, const Motion_Ref_T *Ref)
{
    return (float)((int64_t)Turn - Ref->Turn) * TWO_PI_F + Theta - Ref->Theta;
}

static void Position_Add(Motion_Ref_T *Ref, float Delta, float *Rem)
{
    float Theta;
    int32_t Turns;

    Delta += *Rem;
    Theta = Ref->Theta + Delta;
    *Rem = Delta - (Theta - Ref->Theta);
    Turns = (int32_t)floorf(Theta / TWO_PI_F);

    Ref->Turn += Turns;
    Ref->Theta = Theta - (float)Turns * TWO_PI_F;
    if (Ref->Theta >= TWO_PI_F)
    {
        Ref->Theta -= TWO_PI_F;
        Ref->Turn++;
    }
    else if (Ref->Theta < 0.0f)
    {
        Ref->Theta += TWO_PI_F;
        Ref->Turn--;
    }
}

static float Bernstein(const float *B, uint32_t Degree, float U)
{
    float V[6];

    for (uint32_t n = 0U; n <= Degree; n++)
    {
        V[n] = B[n];
    }
    for (uint32_t k = Degree; k > 0U; k--)
    {
        for (uint32_t n = 0U; n < k; n++)
        {
            V[n] += U * (V[n + 1U] - V[n]);
        }
    }
    return V[0];
}

static float Segment_Distance(const SCurve_Seg_T *Seg, float U0, float U1)
{
    float Mid = 0.5f * (U0 + U1);
    float Half = 0.5f * (U1 - U0);
    float Offset = 0.7745966692414834f * Half;
    float V0 = Bernstein(Seg->B, 5U, Mid - Offset);
    float V1 = Bernstein(Seg->B, 5U, Mid);
    float V2 = Bernstein(Seg->B, 5U, Mid + Offset);

    /* Three-point Gauss integration is exact for quintic velocity; evaluate
     * only the current interval, avoiding subtraction of large positions. */
    return Seg->T * (U1 - U0) * (Seg->Wm + (5.0f * V0 + 8.0f * V1 + 5.0f * V2) / 18.0f);
}

static void Segment_Add(SCurve_T *S, float W0, float W1, float T)
{
    if (T > 0.0f)
    {
        SCurve_Seg_T *Seg = &S->Seg[S->Count++];

        Seg->Wm = W0;
        Seg->B[0] = 0.0f;
        Seg->B[1] = 0.0f;
        Seg->B[2] = 0.0f;
        Seg->B[3] = W1 - W0;
        Seg->B[4] = W1 - W0;
        Seg->B[5] = W1 - W0;
        Seg->T = T;
    }
}

static void Release(SCurve_T *S, Motion_Ref_T *Start,
                    float Acc, float Dec, float Wm, uint8_t Profile)
{
    float A[5];
    float J[4];
    float U = S->Time / S->Seg[0].T;
    float T = 0.25f * (S->Seg[0].T - S->Time);
    float A0;
    float J0;
    float Scale = (Profile == MOTION_S_TIME) ? 1.875f : 1.0f;
    float Old_Scale = (S->Profile == MOTION_S_TIME) ? 1.875f : 1.0f;
    float Up = Max(Scale * Acc, Old_Scale * S->Acc);
    float Down = Max(Scale * Dec, Old_Scale * S->Dec);
    float Limit = Max(Max(fabsf(Wm), fabsf(S->Wm)), fabsf(Start->Wm));
    SCurve_Seg_T Seg = { .Wm = Start->Wm };
    bool Fits = false;

    for (uint32_t n = 0U; n < 5U; n++)
    {
        A[n] = 5.0f * (S->Seg[0].B[n + 1U] - S->Seg[0].B[n]) / S->Seg[0].T;
    }
    for (uint32_t n = 0U; n < 4U; n++)
    {
        J[n] = 4.0f * (A[n + 1U] - A[n]) / S->Seg[0].T;
    }
    A0 = Bernstein(A, 4U, U);
    J0 = Bernstein(J, 3U, U);
    if (Start->Wm < 0.0f)
    {
        float Swap = Up;
        Up = Down;
        Down = Swap;
    }
    /* A second command may arrive before a previous limit reduction has
     * finished. Its inherited acceleration and velocity cannot jump away. */
    Up = Max(Up, A0);
    Down = Max(Down, -A0);
    for (uint32_t n = 0U; n < 6U; n++)
    {
        Limit = Max(Limit, fabsf(S->Seg[0].Wm + S->Seg[0].B[n]));
    }

    /* Free terminal speed lets a quintic release acceleration/jerk without
     * root solving. Its acceleration Bernstein ordinates are
     * [A0, A0+J0*T/4, A0/2+J0*T/6, 0, 0]. Their convex hull certifies limits.
     * On a limit reduction, this one segment uses the old/new envelope;
     * all following segments use the new limits. */
    for (uint32_t Try = 0U; Try < 32U; Try++)
    {
        float A_Tol = 32.0f * FLT_EPSILON * Max(1.0f, Max(Up, Down));
        float V_Tol = 8.0f * FLT_EPSILON * Max(1.0f, Limit);

        Fits = true;
        A[0] = A0;
        A[1] = A0 + 0.25f * J0 * T;
        A[2] = 0.5f * A0 + J0 * T / 6.0f;
        A[3] = 0.0f;
        A[4] = 0.0f;
        Seg.T = T;
        for (uint32_t n = 0U; n < 5U; n++)
        {
            Seg.B[n + 1U] = Seg.B[n] + T * A[n] / 5.0f;
            if ((A[n] > Up + A_Tol) || (A[n] < -Down - A_Tol) ||
                (fabsf(Seg.Wm + Seg.B[n + 1U]) > Limit + V_Tol))
            {
                Fits = false;
            }
        }
        if (Fits)
        {
            break;
        }
        T *= 0.5f;
    }
    if (!Fits)
    {
        float B[6];
        float Tail[6];

        /* Numerical edge: retain the existing admissible segment tail,
         * then apply the latest target. Never accept an unchecked bridge. */
        for (uint32_t n = 0U; n < 6U; n++)
        {
            B[n] = S->Seg[0].B[n];
        }
        Tail[5] = B[5];
        for (uint32_t k = 5U; k > 0U; k--)
        {
            for (uint32_t n = 0U; n < k; n++)
            {
                B[n] += U * (B[n + 1U] - B[n]);
            }
            Tail[k - 1U] = B[k - 1U];
        }
        for (uint32_t n = 0U; n < 6U; n++)
        {
            Seg.B[n] = Tail[n] - Tail[0];
        }
        Seg.T = S->Seg[0].T - S->Time;
    }
    S->Count = 0U;
    S->Time = 0.0f;
    S->Time_Rem = 0.0f;
    if ((A0 != 0.0f) || (J0 != 0.0f))
    {
        S->Seg[S->Count++] = Seg;
        float Rem = 0.0f;

        Position_Add(Start, Segment_Distance(&Seg, 0.0f, 1.0f), &Rem);
        Start->Wm = Seg.Wm + Seg.B[5];
    }
}

static void Plan(SCurve_T *S, const Motion_Ref_T *Ref, bool Position,
                 int32_t Turn, float Theta, float Wm,
                 float Acc, float Dec, uint8_t Profile)
{
    Motion_Ref_T Start = *Ref;
    float K = (Profile == MOTION_S_PEAK) ? 1.875f : 1.0f;
    float Dir;
    float Err;
    float V;
    float Peak;
    float Ta;
    float Td;
    float Cruise;

    if (S->Count != 0U)
    {
        Release(S, &Start, Acc, Dec, Wm, Profile);
    }
    else
    {
        S->Time = 0.0f;
        S->Time_Rem = 0.0f;
    }
    S->Pos_Rem = 0.0f;
    S->Turn = Turn;
    S->Theta = Theta;
    S->Wm = Wm;
    S->Acc = Acc;
    S->Dec = Dec;
    S->Profile = Profile;
    S->Position = Position;
    S->To_Goal = false;

    if (!Position || (Wm == 0.0f))
    {
        if (Start.Wm * Wm < 0.0f)
        {
            Segment_Add(S, Start.Wm, 0.0f, K * fabsf(Start.Wm) / Dec);
            Start.Wm = 0.0f;
        }
        Segment_Add(S, Start.Wm, Wm, K * fabsf(Wm - Start.Wm) /
                    ((fabsf(Wm) > fabsf(Start.Wm)) ? Acc : Dec));
        return;
    }

    Err = Position_Error(Turn, Theta, &Start);
    Dir = (Err >= 0.0f) ? 1.0f : -1.0f;
    V = Dir * Start.Wm;
    Err *= Dir;
    if ((V < 0.0f) || (V > Wm) || (K * V * V / (2.0f * Dec) > Err))
    {
        /* Brake before returning to a target inside the smooth stopping
         * distance. The next plan starts at rest at that stopping point. */
        Segment_Add(S, Start.Wm, 0.0f, K * fabsf(Start.Wm) / Dec);
        return;
    }

    Peak = sqrtf((2.0f * Acc * Dec * Err / K + Dec * V * V) / (Acc + Dec));
    Peak = (Peak < Wm) ? Peak : Wm;
    Peak = Max(Peak, V);
    Ta = K * (Peak - V) / Acc;
    Td = K * Peak / Dec;
    Cruise = Err - 0.5f * (V + Peak) * Ta - 0.5f * Peak * Td;
    Segment_Add(S, Start.Wm, Dir * Peak, Ta);
    if ((Cruise > 0.0f) && (Peak > 0.0f))
    {
        Segment_Add(S, Dir * Peak, Dir * Peak, Cruise / Peak);
    }
    Segment_Add(S, Dir * Peak, 0.0f, Td);
    S->To_Goal = true;
}

static void Run(SCurve_T *S, Motion_Ref_T *Ref, bool Position,
                int32_t Turn, float Theta, float Wm,
                float Acc, float Dec, uint8_t Profile, float Ts)
{
    if (!isfinite(Wm) || !isfinite(Acc) || !isfinite(Dec) || !isfinite(Ts) ||
        (Acc <= 0.0f) || (Dec <= 0.0f) || (Ts <= 0.0f) ||
        ((Profile != MOTION_S_TIME) && (Profile != MOTION_S_PEAK)))
    {
        return;
    }
    if ((S->Position != Position) || (S->Turn != Turn) || (S->Theta != Theta) ||
        (S->Wm != Wm) || (S->Acc != Acc) || (S->Dec != Dec) || (S->Profile != Profile))
    {
        Plan(S, Ref, Position, Turn, Theta, Wm, Acc, Dec, Profile);
    }

    /* A sample may cross release/acceleration/cruise/braking boundaries.
     * At most a brake-and-return plus a four-segment plan is needed. */
    for (uint32_t Step = 0U; (Step < 12U) && (Ts > 0.0f); Step++)
    {
        SCurve_Seg_T *Seg;
        float Dt;
        float U;
        float Next;
        float Advance;
        float A[5];

        if (S->Count == 0U)
        {
            if (Position && (Wm > 0.0f) &&
                ((Ref->Turn != Turn) || (Ref->Theta != Theta) || (Ref->Wm != 0.0f)))
            {
                Plan(S, Ref, Position, Turn, Theta, Wm, Acc, Dec, Profile);
                if (S->Count == 0U)
                {
                    Ref->Turn = Turn;
                    Ref->Theta = Theta;
                    Ref->Wm = 0.0f;
                }
            }
            if (S->Count == 0U)
            {
                Ref->Am = 0.0f;
                if (!Position)
                {
                    Position_Add(Ref, Ref->Wm * Ts, &S->Pos_Rem);
                }
                break;
            }
        }
        Seg = &S->Seg[0];
        Dt = Seg->T - S->Time;
        if (Dt > Ts)
        {
            Dt = Ts;
        }
        Advance = Dt + S->Time_Rem;
        Next = S->Time + Advance;
        S->Time_Rem = Advance - (Next - S->Time);
        U = Next / Seg->T;
        if (U > 1.0f)
        {
            U = 1.0f;
        }
        Position_Add(Ref, Segment_Distance(Seg, S->Time / Seg->T, U), &S->Pos_Rem);
        Ref->Wm = Seg->Wm + Bernstein(Seg->B, 5U, U);
        for (uint32_t n = 0U; n < 5U; n++)
        {
            A[n] = 5.0f * (Seg->B[n + 1U] - Seg->B[n]) / Seg->T;
        }
        Ref->Am = Bernstein(A, 4U, U);
        S->Time = Next;
        Ts -= Dt;
        if ((S->Count == 1U) && S->To_Goal)
        {
            float Remaining = Segment_Distance(Seg, U, 1.0f);

            if (fabsf(Remaining) < TWO_PI_F)
            {
                Ref->Turn = Turn;
                Ref->Theta = Theta;
                S->Pos_Rem = 0.0f;
                Position_Add(Ref, -Remaining, &S->Pos_Rem);
            }
        }
        if (S->Time >= Seg->T)
        {
            Ref->Wm = Seg->Wm + Seg->B[5];
            Ref->Am = 0.0f;
            S->Time = 0.0f;
            S->Time_Rem = 0.0f;
            S->Count--;
            for (uint32_t n = 0U; n < S->Count; n++)
            {
                S->Seg[n] = S->Seg[n + 1U];
            }
            if ((S->Count == 0U) && S->To_Goal)
            {
                Ref->Turn = Turn;
                Ref->Theta = Theta;
                S->Pos_Rem = 0.0f;
            }
        }
    }
}

void SCurve_Position(SCurve_T *S, Motion_Ref_T *Ref,
                     int32_t Turn, float Theta, float Wm_Max,
                     float Acc, float Dec, uint8_t Profile, float Ts)
{
    Run(S, Ref, true, Turn, Theta, fabsf(Wm_Max), Acc, Dec, Profile, Ts);
}

void SCurve_Speed(SCurve_T *S, Motion_Ref_T *Ref, float Wm,
                  float Acc, float Dec, uint8_t Profile, float Ts)
{
    Run(S, Ref, false, 0, 0.0f, Wm, Acc, Dec, Profile, Ts);
}
