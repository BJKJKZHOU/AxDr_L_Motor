/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "DPCC.h"

#include "Math.h"
#include "Sin_LUT.h"

DPCC_Para_T DPCC_Para_Calc(float Rs, float Ld, float Lq, float Ts)
{
    DPCC_Para_T Para;
    float Dd = -__builtin_expm1f(-Rs * Ts / Ld);
    float Dq = -__builtin_expm1f(-Rs * Ts / Lq);
    float Dd_Delay = -__builtin_expm1f(-Rs * (0.5f * Ts) / Ld);
    float Dq_Delay = -__builtin_expm1f(-Rs * (0.5f * Ts) / Lq);

    Para.Ad = 1.0f - Dd;
    Para.Aq = 1.0f - Dq;
    Para.Ad_Delay = 1.0f - Dd_Delay;
    Para.Aq_Delay = 1.0f - Dq_Delay;
    Para.Bd_Delay = Dd_Delay / Rs;
    Para.Bq_Delay = Dq_Delay / Rs;
    Para.Inv_Bd = Rs / Dd;
    Para.Inv_Bq = Rs / Dq;
    return Para;
}

void DPCC_Run(const DPCC_Para_T *Para,
              float Id_Ref, float Iq_Ref, float Id, float Iq,
              float Ud, float Uq, float We_Ld, float We_Lq,
              float Bemf, float Theta_Step, float *Ud_Ref, float *Uq_Ref)
{
    float Sin;
    float Cos;
    float Ud_Delay;
    float Uq_Delay;
    float Id_Pred;
    float Iq_Pred;
    float Ud_Next;
    float Uq_Next;

    /* The old alpha/beta voltage is fixed until the next bottom. Evaluate it
     * at the middle of the remaining half-period in the rotating dq frame. */
    SinCos(Angle_Wrap(0.25f * Theta_Step), &Sin, &Cos);
    Ud_Delay = Ud * Cos + Uq * Sin;
    Uq_Delay = -Ud * Sin + Uq * Cos;
    Id_Pred = Para->Ad_Delay * Id + Para->Bd_Delay * (Ud_Delay + We_Lq * Iq);
    Iq_Pred = Para->Aq_Delay * Iq + Para->Bq_Delay * (Uq_Delay - We_Ld * Id - Bemf);

    Ud_Next = (Id_Ref - Para->Ad * Id_Pred) * Para->Inv_Bd - We_Lq * Iq_Pred;
    Uq_Next = (Iq_Ref - Para->Aq * Iq_Pred) * Para->Inv_Bq + We_Ld * Id_Pred + Bemf;

    /* New PWM centre is Ts after this sample (Ts/2 delay + Ts/2 hold).
     * Return its voltage vector in the sample frame used by inverse Park. */
    SinCos(Angle_Wrap(Theta_Step), &Sin, &Cos);
    *Ud_Ref = Ud_Next * Cos - Uq_Next * Sin;
    *Uq_Ref = Ud_Next * Sin + Uq_Next * Cos;
}
