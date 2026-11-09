/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "SCurve.h"

#include <math.h>

#include "Math.h"

static float Max(float A, float B)
{
    return (A > B) ? A : B;
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

    /* 三点高斯积分对五次速度多项式精确；只积分当前区间，避免大位置相减。 */
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

static void Release(SCurve_T *S, Motion_Ref_T *Start)
{
    float T = 0.25f * (S->Seg[0].T - S->Time);
    float Delta = Start->Am * T;

    S->Count = 0U;
    S->Time = 0.0f;
    S->Time_Rem = 0.0f;
    if (Start->Am != 0.0f)
    {
        /* 加速度从当前值单调回零，不继承旧段的加加速度。
         * 取剩余时间的 1/4，使末速度仍在旧段控制点范围内。 */
        SCurve_Seg_T Seg = {
            .Wm = Start->Wm,
            .B = { 0.0f, 0.2f * Delta, 0.4f * Delta,
                   0.6f * Delta, 0.6f * Delta, 0.6f * Delta },
            .T = T,
        };
        float Rem = 0.0f;

        S->Seg[S->Count++] = Seg;
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
        Release(S, &Start);
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

    Err = (float)((int64_t)Turn - Start.Turn) * TWO_PI_F + Theta - Start.Theta;
    Dir = (Err >= 0.0f) ? 1.0f : -1.0f;
    V = Dir * Start.Wm;
    Err *= Dir;
    if ((V < 0.0f) || (V > Wm) || (K * V * V / (2.0f * Dec) > Err))
    {
        /* 方向相反、速度超限或目标在停止距离内时，先停车，再从停止点规划。 */
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
    if ((S->Position != Position) || (S->Turn != Turn) || (S->Theta != Theta) ||
        (S->Wm != Wm) || (S->Acc != Acc) || (S->Dec != Dec) || (S->Profile != Profile))
    {
        Plan(S, Ref, Position, Turn, Theta, Wm, Acc, Dec, Profile);
    }

    /* 单周期可跨过多个段；先停车再返回时，还需继续执行下一次规划。 */
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
    Run(S, Ref, true, Turn, Theta, Wm_Max, Acc, Dec, Profile, Ts);
}

void SCurve_Speed(SCurve_T *S, Motion_Ref_T *Ref, float Wm,
                  float Acc, float Dec, uint8_t Profile, float Ts)
{
    Run(S, Ref, false, 0, 0.0f, Wm, Acc, Dec, Profile, Ts);
}
