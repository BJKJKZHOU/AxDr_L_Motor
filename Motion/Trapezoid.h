/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <stdint.h>

#include "Motion_Type.h"

void Trapezoid_Reset(Motion_Ref_T *Ref, int32_t Turn, float Theta, float Wm);
void Trapezoid_Run(Motion_Ref_T *Ref,
                   int32_t Turn_Target,
                   float Theta_Target,
                   float Wm_Max,
                   float Acc,
                   float Dec,
                   float Ts);

#endif /* TRAPEZOID_H */
