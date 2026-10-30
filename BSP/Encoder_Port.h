/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include "stm32g4xx_hal.h"

/* Board mapping stays visible for direct counter reads in the sample ISR. */
#define ENCODER_ABZ_TIM TIM3
#define ENCODER_ABZ_CPR_MAX 65536U

/* Called with interrupts masked while rebinding the encoder driver. */
void Encoder_Port_Release(void);
void Encoder_Port_SPI(void);
void Encoder_Port_ABZ(uint32_t Cpr);

#endif /* ENCODER_PORT_H */
