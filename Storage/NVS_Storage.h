/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdint.h>

#define NVS_ALL UINT16_MAX

typedef enum
{
    NVS_INCLUDE,
    NVS_EXCLUDE,

} NVS_Select_e;

int NVS_Storage_Init(void);
/* Ordinary single ID: restore without callbacks; missing record returns -ENOENT.
 * Model/ESO fields are validated and published together, including single loads.
 * NVS_ALL: keep RAM values for missing/invalid scalar records, then validate the
 * complete model. A rejected model is not published and returns -EINVAL;
 * other records still load, but encoder hardware is not rebound and stored
 * calibration is not accepted. */
int NVS_Storage_Load(uint16_t Id);
/* INCLUDE writes in list order; EXCLUDE writes all persistent IDs not listed.
 * Empty INCLUDE does nothing; empty EXCLUDE saves all. No tuning-mode policy. */
int NVS_Storage_Save(const uint16_t *Ids, uint16_t Count, NVS_Select_e Select);
int NVS_Storage_Smoke_Test(void);

extern volatile int NVS_Storage_Smoke_Result;
extern volatile uint32_t NVS_Storage_Smoke_Readback;

#endif /* NVS_STORAGE_H */
