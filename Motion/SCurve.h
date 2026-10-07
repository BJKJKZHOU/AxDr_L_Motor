/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SCURVE_H
#define SCURVE_H

#include <stdbool.h>

#include "Motion_Type.h"

/* Quintic velocity in Bernstein form: Wm + B(u), u=t/T. Storing velocity
 * increments keeps derivatives accurate when a small change rides on high speed. */
typedef struct
{
    float Wm;
    float B[6];
    float T;

} SCurve_Seg_T;

typedef struct
{
    SCurve_Seg_T Seg[4]; /* acceleration release, acceleration, cruise, braking */
    float Time;
    float Time_Rem;
    float Pos_Rem; /* integration roundoff below the single-turn float resolution */
    /* Inputs retained from the last plan to detect changes requiring replanning. */
    int32_t Turn; /* position target, paired with Theta; zero in speed planning */
    float Theta;
    float Wm; /* signed speed target, or nonnegative position-mode speed limit (rad/s) */
    float Acc;
    float Dec;
    uint8_t Profile;
    uint8_t Count;
    bool Position;
    bool To_Goal;

} SCurve_T;

/* Zero-initialize on servo enable. Ref.Am is instantaneous acceleration here.
 * Input snapshots belong to the active plan, so changing any input replans once.
 * Speed also integrates position, allowing the same path to perform position Stop.
 * Stop is complete only when Count==0 and Ref.Wm==0, not at a velocity zero crossing. */
void SCurve_Position(SCurve_T *S, Motion_Ref_T *Ref,
                     int32_t Turn, float Theta, float Wm_Max,
                     float Acc, float Dec, uint8_t Profile, float Ts);
void SCurve_Speed(SCurve_T *S, Motion_Ref_T *Ref, float Wm,
                  float Acc, float Dec, uint8_t Profile, float Ts);

#endif /* SCURVE_H */
