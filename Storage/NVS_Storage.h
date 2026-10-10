/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdint.h>

int NVS_Storage_Init(void);
/* Missing or invalid scalar records retain RAM defaults. Restore the motor
 * model and ESO bandwidth together before updating derived coefficients.
 * If the resulting coefficients are not representable, keep the active model
 * and reject saved calibration; independent parameters still load. */
int NVS_Storage_Load_All(void);
/* Save every persistent Parameter, including Manual Kp/Ki even when the
 * effective controller uses Bandwidth tuning. */
int NVS_Storage_Save_All(void);
/* Save selected persistent IDs in caller order (calibration Valid-last).
 * An invalid list is rejected before writing; writes are not transactional. */
int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count);
int NVS_Storage_Smoke_Test(void);

extern volatile int NVS_Storage_Smoke_Result;
extern volatile uint32_t NVS_Storage_Smoke_Readback;

#endif /* NVS_STORAGE_H */
