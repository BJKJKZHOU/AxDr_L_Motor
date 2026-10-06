/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SIN_LUT_H
#define SIN_LUT_H

#include "Fast_Memory.h"

/*
 * Lookup-table sine/cosine for electrical angle in radians.
 * Theta is expected in [0, 2pi); Motor_Run.Theta_e already follows this range.
 */
FAST_CODE void SinCos(float Theta, float *Sin, float *Cos);

#endif /* SIN_LUT_H */
