/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#define KEY_SCAN_TICK 5U /* 扫描间隔，单位为系统 tick。 */
#define KEY_1 (1U << 0)
#define KEY_2 (1U << 1)
#define KEY_3 (1U << 2)

typedef struct
{
    uint8_t State; /* 消抖后的状态：1 为按下，初值为 0。 */
    uint8_t Cnt;

} Key_T;

/* 每隔 KEY_SCAN_TICK 调用，返回本次按下的按键组合。 */
uint32_t Key_Scan(Key_T Keys[3]);

#endif /* KEY_H */
