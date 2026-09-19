/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdint.h>

int NVS_Storage_Init(void);
int NVS_Storage_Smoke_Test(void);

extern volatile int NVS_Storage_Smoke_Result;
extern volatile uint32_t NVS_Storage_Smoke_Readback;

#endif /* NVS_STORAGE_H */
