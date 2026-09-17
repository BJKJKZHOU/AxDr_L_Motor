/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Mechanical_ESO.h"

#include "control_params.h"

Mechanical_ESO_T Mechanical_ESO = { 0 };
float Mechanical_ESO_Bw_Hz = MECH_ESO_BW_HZ_DEFAULT;

bool Mechanical_ESO_Config(float J, float B, float Kt, float Wo)
{
    float B_Over_J;
    float Wo2;

    Mechanical_ESO.Para = (Mechanical_ESO_Para_T){ 0 };

    if (!__builtin_isfinite(J) || (J <= 0.0f) ||
        !__builtin_isfinite(B) || (B < 0.0f) ||
        !__builtin_isfinite(Kt) || (Kt <= 0.0f) ||
        !__builtin_isfinite(Wo) || (Wo <= 0.0f))
    {
        return false;
    }

    Mechanical_ESO.Para.Inv_J = 1.0f / J;
    Mechanical_ESO.Para.Kt_Over_J = Kt * Mechanical_ESO.Para.Inv_J;
    Mechanical_ESO.Para.B_Over_J = B * Mechanical_ESO.Para.Inv_J;

    B_Over_J = Mechanical_ESO.Para.B_Over_J;
    Wo2 = Wo * Wo;

    /*
     * Error dynamics are pole-placed at (s + Wo)^3 for the physical-state
     * model [Theta, Wm, Td], where Td is load/disturbance torque in N*m:
     *
     *   Theta_dot = Wm
     *   Wm_dot    = Kt/J * Iq - B/J * Wm - Td/J
     *   Td_dot    = 0
     *
     * With e = Theta_meas - Theta_hat, L3 is negative because Td enters the
     * mechanical acceleration equation through -Td/J.
     */
    Mechanical_ESO.Para.L1 = 3.0f * Wo - B_Over_J;
    Mechanical_ESO.Para.L2 = 3.0f * Wo2 - Mechanical_ESO.Para.L1 * B_Over_J;
    Mechanical_ESO.Para.L3 = -J * Wo2 * Wo;

    if (!__builtin_isfinite(Mechanical_ESO.Para.L1) ||
        !__builtin_isfinite(Mechanical_ESO.Para.L2) ||
        !__builtin_isfinite(Mechanical_ESO.Para.L3) ||
        (Mechanical_ESO.Para.L1 <= 0.0f) ||
        (Mechanical_ESO.Para.L2 <= 0.0f))
    {
        Mechanical_ESO.Para = (Mechanical_ESO_Para_T){ 0 };
        return false;
    }

    Mechanical_ESO.Para.Valid = 1U;
    return true;
}

void Mechanical_ESO_Reset(float Theta, float Wm)
{
    Mechanical_ESO.State.Theta = __builtin_isfinite(Theta) ? Theta : 0.0f;
    Mechanical_ESO.State.Wm = __builtin_isfinite(Wm) ? Wm : 0.0f;
    Mechanical_ESO.State.Td = 0.0f;
    Mechanical_ESO.State.Error = 0.0f;
}

void Mechanical_ESO_Run(float Theta_Meas, bool Position_Valid, float Iq)
{
    float Error = 0.0f;
    float Theta_Dot;
    float Wm_Dot;
    float Td_Dot;

    if (Mechanical_ESO.Para.Valid == 0U)
    {
        return;
    }

    if (!__builtin_isfinite(Iq))
    {
        Iq = 0.0f;
    }

    if (Position_Valid && __builtin_isfinite(Theta_Meas))
    {
        Error = Theta_Meas - Mechanical_ESO.State.Theta;
    }

    Mechanical_ESO.State.Error = Error;

    Theta_Dot = Mechanical_ESO.State.Wm + Mechanical_ESO.Para.L1 * Error;
    Wm_Dot = Mechanical_ESO.Para.Kt_Over_J * Iq -
             Mechanical_ESO.Para.B_Over_J * Mechanical_ESO.State.Wm -
             Mechanical_ESO.Para.Inv_J * Mechanical_ESO.State.Td +
             Mechanical_ESO.Para.L2 * Error;
    Td_Dot = Mechanical_ESO.Para.L3 * Error;

    Mechanical_ESO.State.Theta += Theta_Dot * CUR_TS;
    Mechanical_ESO.State.Wm += Wm_Dot * CUR_TS;
    Mechanical_ESO.State.Td += Td_Dot * CUR_TS;
}

float Mechanical_ESO_Wm_Get(void)
{
    return Mechanical_ESO.State.Wm;
}
