/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef RAMP_H
#define RAMP_H

#include "Motion_Type.h"

void Ramp_Reset(Motion_Ref_T *Ref, float Wm);
void Ramp_Run(Motion_Ref_T *Ref, float Wm_Target, float Acc, float Dec, float Ts);

#endif /* RAMP_H */
