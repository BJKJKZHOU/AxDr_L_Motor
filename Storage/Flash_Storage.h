/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#define STORAGE_FLASH_TOTAL_SIZE   (512U * 1024U)
#define STORAGE_NVS_OFFSET         0x0007C000UL
#define STORAGE_NVS_SIZE           (16U * 1024U)
#define STORAGE_NVS_SECTOR_SIZE    2048U
#define STORAGE_NVS_SECTOR_COUNT   8U
#define STORAGE_FLASH_WRITE_SIZE   8U

bool Flash_Storage_Geometry_Valid(void);

#endif /* FLASH_STORAGE_H */
