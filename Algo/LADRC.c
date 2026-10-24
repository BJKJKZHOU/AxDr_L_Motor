/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "LADRC.h"

float LADRC_Speed(float Wm_Err, float Wm, float Td, float Wc,
                  float B_Over_J, float Inv_J, float Kt_Over_J)
{
    /* J*dWm/dt = Kt*Iq - B*Wm - Td. Compensate friction and the
     * external ESO disturbance once, with proportional speed feedback. */
    return (Wc * Wm_Err + B_Over_J * Wm + Inv_J * Td) / Kt_Over_J;
}

void LADRC_Run(LADRC_T *Ctrl, float Ts)
{
    float Eso_Err;
    float Wo;

    /* The observer uses the applied input, not the new or unclipped command.
     * The caller owns PWM-delay alignment, output limits and state seeding. */
    Eso_Err = Ctrl->State.Z1 - Ctrl->Sig.Fbk;
    Wo = Ctrl->Para.Wo;

    Ctrl->State.Z1 += Ts * (Ctrl->State.Z2 + Ctrl->Sig.F0 +
                            Ctrl->Para.B0 * Ctrl->Sig.U_Applied - 2.0f * Wo * Eso_Err);
    Ctrl->State.Z2 -= Ts * Wo * Wo * Eso_Err;

    Ctrl->Sig.Err = Ctrl->Sig.Ref - Ctrl->State.Z1;
    /* -F0/B0 includes the known model compensation; do not add it twice. */
    Ctrl->Sig.Out = (Ctrl->Para.Kp * Ctrl->Sig.Err + Ctrl->Sig.Ref_Dot -
                     Ctrl->State.Z2 - Ctrl->Sig.F0) / Ctrl->Para.B0;
}
