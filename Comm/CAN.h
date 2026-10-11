/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

/* FDCAN1 hardware is initialized by CubeMX before ThreadX starts. */
bool CAN_Init(void);
void CAN_Poll(void);
/* True when the FDCAN peripheral has started successfully. */
bool CAN_Active(void);

extern volatile uint32_t CAN_Rx_Cnt;
extern volatile uint32_t CAN_Tx_Cnt;
extern volatile uint32_t CAN_Error_Cnt;

#endif /* CAN_H */
