/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MATH_H
#define MATH_H

#include "Fast_Memory.h"

#include <stdint.h>

#define PI_F         3.14159265358979323846f
#define TWO_PI_F     6.28318530717958647692f
#define SQRT3_F      1.73205080756887729353f
#define INV_SQRT3_F  0.57735026918962576451f
#define SQRT3_HALF_F 0.86602540378443864676f

FAST_CODE int8_t Limit_Value(float *Value, float Min, float Max);
FAST_CODE void Vector2_Limit(float *X, float *Y, float Lim);
FAST_CODE float Angle_Wrap(float Theta);

/* Polynomial 0x07, MSB first, no reflection or final XOR.
 * Pass the initial CRC on the first byte, then the previous result. */
extern const uint8_t CRC8_07_Table[256];

static inline uint8_t CRC8_07(uint8_t Crc, uint8_t Data)
{
    return CRC8_07_Table[Crc ^ Data];
}

#endif
