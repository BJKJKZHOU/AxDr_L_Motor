/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ST7789_H
#define ST7789_H

#include <stdbool.h>
#include <stdint.h>

/* V1.3 IPSLCD1: physical 135x240 panel, mounted landscape (MY | MV). */
#define ST7789_WIDTH    240U
#define ST7789_HEIGHT   135U
#define ST7789_X_OFFSET 40U
#define ST7789_Y_OFFSET 52U
#define ST7789_MADCTL   0xA0U

bool ST7789_Init(void);
bool ST7789_Ready(void);
void ST7789_Wait(void);

/*
 * RGB565 pixels are big-endian (high byte first), tightly packed by rows.
 * One rectangle is transferred with one SPI3 TX DMA request.
 * Pixels must remain unchanged until ST7789_Ready() returns true.
 */
bool ST7789_Write(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height,
                  const uint8_t *Pixels);

#endif /* ST7789_H */
