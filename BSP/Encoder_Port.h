/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include "Board_Config.h"

/* 切换驱动时调用，调用前需屏蔽中断。 */
void Encoder_Port_Release(void);
void Encoder_Port_SPI(void);
void Encoder_Port_ABZ(uint32_t Cpr);

#endif /* ENCODER_PORT_H */
