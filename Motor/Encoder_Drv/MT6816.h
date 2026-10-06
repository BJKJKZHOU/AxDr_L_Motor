/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MT6816_H
#define MT6816_H

#include "Fast_Memory.h"

#include <stdint.h>

typedef struct
{
    uint32_t Parity_Err;
    uint16_t No_Mag_Cnt;
    uint8_t No_Mag;

} MT6816_State_T;

extern volatile MT6816_State_T MT6816_State;

void MT6816_Config(void);
FAST_CODE void MT6816_Start(void);
FAST_CODE void MT6816_IRQHandler(void);

#endif /* MT6816_H */
