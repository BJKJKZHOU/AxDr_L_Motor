/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef LCD_WAVE_H
#define LCD_WAVE_H

#include <stdbool.h>
#include <stdint.h>

/* 由 Panel 线程循环调用，采样和画面刷新各自按设定间隔执行。 */
void LCD_Wave_Init(uint32_t Tick);
void LCD_Wave_Run(uint32_t Tick);

#endif /* LCD_WAVE_H */
