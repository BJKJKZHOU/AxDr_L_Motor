/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef LCD_WAVE_H
#define LCD_WAVE_H

#include <stdbool.h>
#include <stdint.h>

/* SWD diagnostics; unchanged frames do not count as completed updates. */
typedef struct
{
    uint32_t Frame_Cnt;
    uint32_t Byte_Cnt;
    uint32_t Flush_Ticks;
    uint32_t Flush_Max;
    uint32_t Render_Cyc;
    uint32_t Render_Max;
} LCD_Time_T;

extern volatile LCD_Time_T LCD_Time;
/* 0: waveform, 1: pause after pending DMA, 2: full refresh, 3: corner pattern. */
extern volatile uint32_t LCD_Test;

/* One call per Panel Thread tick; sampling and frame generation are independent. */
bool LCD_Wave_Init(uint32_t Tick);
void LCD_Wave_Run(uint32_t Tick);

#endif /* LCD_WAVE_H */
