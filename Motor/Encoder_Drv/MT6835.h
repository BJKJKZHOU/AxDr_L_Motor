/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MT6835_H
#define MT6835_H

#include <stdint.h>

typedef struct
{
    uint32_t CRC_Err;
    uint16_t Under_Voltage_Cnt;
    uint8_t Status;
    uint8_t Over_Speed;
    uint8_t Weak_Field;
    uint8_t Under_Voltage;

} MT6835_State_T;

extern volatile MT6835_State_T MT6835_State;

void MT6835_Config(void);
void MT6835_Start(void);
void MT6835_IRQHandler(void);

#endif /* MT6835_H */
