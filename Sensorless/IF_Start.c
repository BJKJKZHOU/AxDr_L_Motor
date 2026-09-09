/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "IF_Start.h"

#include <stddef.h>

#include "Math.h"

/*
 * Continuous adaptive startup:
 * - establish the minimum current and sample zero-speed residual background;
 * - raise Iq continuously on a faster electrical time scale;
 * - raise We continuously on a slower mechanical startup time scale;
 * - clamp We at a low probe speed until motion is confirmed;
 * - only allow final failure after both Iq_Max and We_probe are reached and
 *   several complete electrical cycles still fail the motion check.
 *
 * Fast-loop motion metric keeps only multiply/add operations:
 *
 *   Rd = Ud*Ts - Rs*Id*Ts - Ld*dId + We*Lq*Iq*Ts
 *   Rq = Uq*Ts - Rs*Iq*Ts - Lq*dIq - We*Ld*Id*Ts
 *   R2 = Rd*Rd + Rq*Rq
 */
#define IF_KICK_CHECK_WE_RATIO       0.10f
#define IF_KICK_I_RAMP_TIME_S        1.50f
#define IF_KICK_WE_RAMP_TIME_S       2.40f
#define IF_KICK_BASE_SETTLE_S        0.200f
#define IF_KICK_BASE_SAMPLE_S        0.020f
#define IF_KICK_VERIFY_CYCLE_COUNT   3U
#define IF_KICK_MAX_FAIL_CYCLE_COUNT 3U
#define IF_KICK_R2_BASE_RATIO        4.0f

static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}

static float Sign_F(float X)
{
    return (X < 0.0f) ? -1.0f : 1.0f;
}

static void IF_Fail(IF_T *IF)
{
    IF->State.Mode = IF_FAILED;
    IF->State.We = 0.0f;
    IF->State.Iq = 0.0f;
}

static void Iq_Slew_Run(IF_T *IF, float Target, float Ts)
{
    float Step;

    Step = IF->Para.Iq_Slew_A_S * Ts;
    if (IF->State.Iq < Target)
    {
        IF->State.Iq += Step;
        if (IF->State.Iq > Target)
        {
            IF->State.Iq = Target;
        }
    }
    else if (IF->State.Iq > Target)
    {
        IF->State.Iq -= Step;
        if (IF->State.Iq < Target)
        {
            IF->State.Iq = Target;
        }
    }
}

static void Kick_Residual_Reset(IF_T *IF)
{
    IF->State.Kick_Current_Valid = false;
    IF->State.Kick_Id_Last = 0.0f;
    IF->State.Kick_Iq_Last = 0.0f;
}

static bool Kick_Residual_R2(IF_T *IF,
                             float Id_A,
                             float Iq_A,
                             float Ud_V,
                             float Uq_V,
                             float Ts,
                             float *R2)
{
    float dId;
    float dIq;
    float Rd;
    float Rq;

    if ((R2 == NULL) ||
        !__builtin_isfinite(Id_A) || !__builtin_isfinite(Iq_A) ||
        !__builtin_isfinite(Ud_V) || !__builtin_isfinite(Uq_V) ||
        !__builtin_isfinite(IF->State.We) || !__builtin_isfinite(Ts) ||
        (Ts <= 0.0f))
    {
        return false;
    }

    if (!IF->State.Kick_Current_Valid)
    {
        IF->State.Kick_Id_Last = Id_A;
        IF->State.Kick_Iq_Last = Iq_A;
        IF->State.Kick_Current_Valid = true;
        *R2 = 0.0f;
        return true;
    }

    dId = Id_A - IF->State.Kick_Id_Last;
    dIq = Iq_A - IF->State.Kick_Iq_Last;
    IF->State.Kick_Id_Last = Id_A;
    IF->State.Kick_Iq_Last = Iq_A;

    Rd = Ud_V * Ts - IF->Para.Rs_Ohm * Id_A * Ts - IF->Para.Ld_H * dId +
         IF->State.We * IF->Para.Lq_H * Iq_A * Ts;
    Rq = Uq_V * Ts - IF->Para.Rs_Ohm * Iq_A * Ts - IF->Para.Lq_H * dIq -
         IF->State.We * IF->Para.Ld_H * Id_A * Ts;

    if (!__builtin_isfinite(Rd) || !__builtin_isfinite(Rq))
    {
        return false;
    }

    *R2 = Rd * Rd + Rq * Rq;
    return __builtin_isfinite(*R2);
}

static bool Kick_Check_We_Build(IF_T *IF)
{
    float Target_Abs;
    float Check_Abs;

    Target_Abs = Abs_F(IF->State.We_Target);
    if (Target_Abs <= 0.0f)
    {
        return false;
    }

    Check_Abs = IF_KICK_CHECK_WE_RATIO * IF->Para.We_Base;
    if (Check_Abs > Target_Abs)
    {
        Check_Abs = Target_Abs;
    }
    if (Check_Abs <= 0.0f)
    {
        return false;
    }

    IF->State.Kick_Check_We = Sign_F(IF->State.We_Target) * Check_Abs;
    return true;
}

static void Kick_Cycle_Reset(IF_T *IF)
{
    IF->State.Kick_Cycle_R2_Sum = 0.0f;
    IF->State.Kick_Cycle_R2_Cnt = 0U;
    IF->State.Kick_Cycle_Travel = 0.0f;
    Kick_Residual_Reset(IF);
}

static bool Kick_Cycle_Passed(const IF_T *IF)
{
    float Run_Scaled;
    float Base_Scaled;

    if ((IF->State.Kick_Base_R2_Cnt == 0U) ||
        (IF->State.Kick_Cycle_R2_Cnt == 0U))
    {
        return false;
    }

    Run_Scaled = IF->State.Kick_Cycle_R2_Sum * (float)IF->State.Kick_Base_R2_Cnt;
    Base_Scaled = IF_KICK_R2_BASE_RATIO * IF->State.Kick_Base_R2_Sum *
                  (float)IF->State.Kick_Cycle_R2_Cnt;

    return __builtin_isfinite(Run_Scaled) && __builtin_isfinite(Base_Scaled) &&
           (Run_Scaled > Base_Scaled);
}

static void Kick_Search_Iq_Run(IF_T *IF, float Dir, float Ts)
{
    float Span;
    float Step;
    float Target;

    Span = IF->Para.Iq_Max_A - IF->Para.Iq_Min_A;
    Step = (Span / IF_KICK_I_RAMP_TIME_S) * Ts;
    Target = Dir * IF->Para.Iq_Max_A;

    if (Dir > 0.0f)
    {
        IF->State.Iq += Step;
        if (IF->State.Iq > Target)
        {
            IF->State.Iq = Target;
        }
    }
    else
    {
        IF->State.Iq -= Step;
        if (IF->State.Iq < Target)
        {
            IF->State.Iq = Target;
        }
    }
}

static void Kick_Search_Run(IF_T *IF,
                            float Id_A,
                            float Iq_A,
                            float Ud_V,
                            float Uq_V,
                            float Ts)
{
    float Dir;
    float We_Step;
    float Check_Abs;
    float R2;
    bool Cycle_Pass;
    bool Current_Max;
    bool Speed_Ready;

    Dir = Sign_F(IF->State.We_Target);
    Check_Abs = Abs_F(IF->State.Kick_Check_We);
    We_Step = (Check_Abs / IF_KICK_WE_RAMP_TIME_S) * Ts;

    Kick_Search_Iq_Run(IF, Dir, Ts);

    if (Abs_F(IF->State.We) < Check_Abs)
    {
        IF->State.We += Dir * We_Step;
        if (Abs_F(IF->State.We) >= Check_Abs)
        {
            IF->State.We = IF->State.Kick_Check_We;
            Kick_Cycle_Reset(IF);
        }
        return;
    }

    IF->State.We = IF->State.Kick_Check_We;

    R2 = 0.0f;
    if (!Kick_Residual_R2(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts, &R2))
    {
        IF_Fail(IF);
        return;
    }

    IF->State.Kick_Cycle_R2_Sum += R2;
    IF->State.Kick_Cycle_R2_Cnt++;
    IF->State.Kick_Cycle_Travel += Check_Abs * Ts;
    if (IF->State.Kick_Cycle_Travel < TWO_PI_F)
    {
        return;
    }

    Cycle_Pass = Kick_Cycle_Passed(IF);
    Current_Max = Abs_F(IF->State.Iq) >= IF->Para.Iq_Max_A;
    Speed_Ready = Abs_F(IF->State.We) >= Check_Abs;

    if (Cycle_Pass)
    {
        if (IF->State.Kick_Pass_Streak < UINT8_MAX)
        {
            IF->State.Kick_Pass_Streak++;
        }
        IF->State.Kick_Max_Fail_Cycles = 0U;
    }
    else
    {
        IF->State.Kick_Pass_Streak = 0U;

        if (Current_Max && Speed_Ready)
        {
            if (IF->State.Kick_Max_Fail_Cycles < UINT8_MAX)
            {
                IF->State.Kick_Max_Fail_Cycles++;
            }
        }
        else
        {
            IF->State.Kick_Max_Fail_Cycles = 0U;
        }
    }

    IF->State.Kick_Cycle_R2_Sum = 0.0f;
    IF->State.Kick_Cycle_R2_Cnt = 0U;
    IF->State.Kick_Cycle_Travel -= TWO_PI_F;

    if (IF->State.Kick_Pass_Streak >= IF_KICK_VERIFY_CYCLE_COUNT)
    {
        IF->State.Iq_Work_A = Abs_F(IF->State.Iq);
        if (IF->State.Iq_Work_A < IF->Para.Iq_Min_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
        }
        else if (IF->State.Iq_Work_A > IF->Para.Iq_Max_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Max_A;
        }

        IF->State.Mode = (IF->State.We == IF->State.We_Target) ? IF_HOLD : IF_RAMP;
        return;
    }

    if (Current_Max && Speed_Ready &&
        (IF->State.Kick_Max_Fail_Cycles >= IF_KICK_MAX_FAIL_CYCLE_COUNT))
    {
        IF_Fail(IF);
    }
}

static void Kick_Run(IF_T *IF,
                     float Id_A,
                     float Iq_A,
                     float Ud_V,
                     float Uq_V,
                     float Ts)
{
    float Dir;
    float R2;

    if (Abs_F(IF->State.We_Target) <= 0.0f)
    {
        IF->State.We = 0.0f;
        Iq_Slew_Run(IF, 0.0f, Ts);
        return;
    }

    Dir = Sign_F(IF->State.We_Target);

    switch (IF->State.Kick_Mode)
    {
        case IF_KICK_CURRENT:
            IF->State.We = 0.0f;
            Iq_Slew_Run(IF, Dir * IF->Para.Iq_Min_A, Ts);
            if (Abs_F(IF->State.Iq) < IF->Para.Iq_Min_A)
            {
                return;
            }
            if ((IF->State.Kick_Check_We == 0.0f) && !Kick_Check_We_Build(IF))
            {
                IF_Fail(IF);
                return;
            }

            IF->State.Kick_Time = 0.0f;
            Kick_Residual_Reset(IF);
            IF->State.Kick_Mode = IF_KICK_BASELINE_SETTLE;
            return;

        case IF_KICK_BASELINE_SETTLE:
            IF->State.We = 0.0f;
            Iq_Slew_Run(IF, Dir * IF->Para.Iq_Min_A, Ts);
            IF->State.Kick_Time += Ts;
            if (IF->State.Kick_Time >= IF_KICK_BASE_SETTLE_S)
            {
                IF->State.Kick_Time = 0.0f;
                IF->State.Kick_Base_R2_Sum = 0.0f;
                IF->State.Kick_Base_R2_Cnt = 0U;
                Kick_Residual_Reset(IF);
                IF->State.Kick_Mode = IF_KICK_BASELINE_SAMPLE;
            }
            return;

        case IF_KICK_BASELINE_SAMPLE:
            IF->State.We = 0.0f;
            Iq_Slew_Run(IF, Dir * IF->Para.Iq_Min_A, Ts);
            R2 = 0.0f;
            if (!Kick_Residual_R2(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts, &R2))
            {
                IF_Fail(IF);
                return;
            }
            if (IF->State.Kick_Current_Valid)
            {
                IF->State.Kick_Base_R2_Sum += R2;
                IF->State.Kick_Base_R2_Cnt++;
            }
            IF->State.Kick_Time += Ts;
            if (IF->State.Kick_Time >= IF_KICK_BASE_SAMPLE_S)
            {
                if (IF->State.Kick_Base_R2_Cnt == 0U)
                {
                    IF_Fail(IF);
                    return;
                }

                IF->State.Kick_Time = 0.0f;
                IF->State.Kick_Pass_Streak = 0U;
                IF->State.Kick_Max_Fail_Cycles = 0U;
                Kick_Cycle_Reset(IF);
                IF->State.Kick_Mode = IF_KICK_SEARCH;
            }
            return;

        case IF_KICK_SEARCH:
            Kick_Search_Run(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts);
            return;

        default:
            IF_Fail(IF);
            return;
    }
}

void IF_Init(IF_T *IF, float Theta_Start, float We_Start)
{
    IF_Para_T Para;
    float Iq_Work_Pre;

    if (IF == NULL)
    {
        return;
    }

    Para = IF->Para;
    Iq_Work_Pre = IF->State.Iq_Work_A;
    IF->State = (IF_State_T){ 0 };
    IF->Para = Para;
    IF->State.Theta_e = Angle_Wrap(Theta_Start);
    IF->State.We = We_Start;
    IF->State.We_Target = We_Start;
    IF->State.Kick_Mode = IF_KICK_CURRENT;

    if ((IF->Para.Iq_Min_A <= 0.0f) ||
        (IF->Para.Iq_Max_A < IF->Para.Iq_Min_A) ||
        (IF->Para.We_Base <= 0.0f) ||
        (IF->Para.Acc <= 0.0f) ||
        (IF->Para.Iq_Slew_A_S <= 0.0f) ||
        (IF->Para.Rs_Ohm < 0.0f) ||
        (IF->Para.Ld_H <= 0.0f) ||
        (IF->Para.Lq_H <= 0.0f) ||
        !__builtin_isfinite(IF->Para.Iq_Min_A) ||
        !__builtin_isfinite(IF->Para.Iq_Max_A) ||
        !__builtin_isfinite(IF->Para.We_Base) ||
        !__builtin_isfinite(IF->Para.Acc) ||
        !__builtin_isfinite(IF->Para.Iq_Slew_A_S) ||
        !__builtin_isfinite(IF->Para.Rs_Ohm) ||
        !__builtin_isfinite(IF->Para.Ld_H) ||
        !__builtin_isfinite(IF->Para.Lq_H) ||
        !__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(We_Start))
    {
        IF_Fail(IF);
        return;
    }

    if (Abs_F(We_Start) <= 0.0f)
    {
        IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
        IF->State.Mode = IF_KICK;
    }
    else
    {
        IF->State.Iq_Work_A = Iq_Work_Pre;
        if (IF->State.Iq_Work_A < IF->Para.Iq_Min_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Min_A;
        }
        else if (IF->State.Iq_Work_A > IF->Para.Iq_Max_A)
        {
            IF->State.Iq_Work_A = IF->Para.Iq_Max_A;
        }
        IF->State.Mode = IF_RAMP;
    }
}

void IF_Target_Set(IF_T *IF, float We_Target)
{
    if ((IF == NULL) || !__builtin_isfinite(We_Target) ||
        (IF->State.We_Target == We_Target))
    {
        return;
    }

    IF->State.We_Target = We_Target;
    if ((IF->State.Mode == IF_FAILED) || (IF->State.Mode == IF_KICK))
    {
        return;
    }
    IF->State.Mode = (IF->State.We == We_Target) ? IF_HOLD : IF_RAMP;
}

void IF_Run(IF_T *IF,
            float Id_A,
            float Iq_A,
            float Ud_V,
            float Uq_V,
            float *Theta_e,
            float *Id_Ref,
            float *Iq_Ref,
            float Ts)
{
    float Iq_Target;
    float We_Step;

    if ((IF == NULL) || (Theta_e == NULL) || (Id_Ref == NULL) || (Iq_Ref == NULL) ||
        (Ts <= 0.0f) || !__builtin_isfinite(Ts))
    {
        return;
    }

    if (!__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(IF->State.We) ||
        !__builtin_isfinite(IF->State.We_Target) ||
        !__builtin_isfinite(IF->State.Iq) ||
        !__builtin_isfinite(IF->State.Iq_Work_A))
    {
        IF_Fail(IF);
    }

    if (IF->State.Mode == IF_FAILED)
    {
        *Theta_e = __builtin_isfinite(IF->State.Theta_e) ? IF->State.Theta_e : 0.0f;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    if (IF->State.Mode == IF_KICK)
    {
        Kick_Run(IF, Id_A, Iq_A, Ud_V, Uq_V, Ts);
    }
    else
    {
        if (IF->State.Mode == IF_RAMP)
        {
            We_Step = IF->Para.Acc * Ts;
            if (IF->State.We < IF->State.We_Target)
            {
                IF->State.We += We_Step;
                if (IF->State.We >= IF->State.We_Target)
                {
                    IF->State.We = IF->State.We_Target;
                    IF->State.Mode = IF_HOLD;
                }
            }
            else
            {
                IF->State.We -= We_Step;
                if (IF->State.We <= IF->State.We_Target)
                {
                    IF->State.We = IF->State.We_Target;
                    IF->State.Mode = IF_HOLD;
                }
            }
        }

        if (IF->State.We > 0.0f)
        {
            Iq_Target = IF->State.Iq_Work_A;
        }
        else if (IF->State.We < 0.0f)
        {
            Iq_Target = -IF->State.Iq_Work_A;
        }
        else
        {
            Iq_Target = 0.0f;
        }
        Iq_Slew_Run(IF, Iq_Target, Ts);
    }

    if ((IF->State.Mode == IF_FAILED) ||
        !__builtin_isfinite(IF->State.Theta_e) ||
        !__builtin_isfinite(IF->State.We) ||
        !__builtin_isfinite(IF->State.Iq))
    {
        IF_Fail(IF);
        *Theta_e = 0.0f;
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return;
    }

    *Theta_e = IF->State.Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = IF->State.Iq;
    IF->State.Theta_e = Angle_Wrap(IF->State.Theta_e + IF->State.We * Ts);
}
