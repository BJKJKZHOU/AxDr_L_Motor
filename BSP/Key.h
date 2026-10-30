/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#define KEY_SCAN_TICK 5U /* Keep the panel task's existing RTOS tick cadence. */
#define KEY_1 (1U << 0)
#define KEY_2 (1U << 1)
#define KEY_3 (1U << 2)

typedef struct
{
    uint8_t State; /* Debounced logical level: 1=pressed; initially zero. */
    uint8_t Cnt;

} Key_T;

/* Scan once per KEY_SCAN_TICK; return newly pressed keys as a bit mask. */
uint32_t Key_Scan(Key_T Keys[3]);

#endif /* KEY_H */
