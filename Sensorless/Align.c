/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Align.h"

bool Align_Run(Align_T *Align)
{
    Align->Sig.Id_Ref = Align->Para.Id_A;
    Align->Sig.Iq_Ref = 0.0f;

    if (Align->State.Cnt < Align->Para.Hold_Cnt)
    {
        Align->State.Cnt++;
    }

    return Align->State.Cnt >= Align->Para.Hold_Cnt;
}
