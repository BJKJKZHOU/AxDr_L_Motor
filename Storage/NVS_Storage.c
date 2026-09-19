/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "NVS_Storage.h"

#include <errno.h>

#include "Flash_Storage.h"
#include <zephyr/kvss/nvs.h>

#define STORAGE_SMOKE_ID 0x7FF0U

static struct nvs_fs Storage_NVS;

volatile int NVS_Storage_Smoke_Result = 0;
volatile uint32_t NVS_Storage_Smoke_Readback = 0U;

int NVS_Storage_Init(void)
{
    if (!Flash_Storage_Geometry_Valid())
    {
        return -ENOTSUP;
    }

    Storage_NVS.offset = STORAGE_NVS_OFFSET;
    Storage_NVS.sector_size = STORAGE_NVS_SECTOR_SIZE;
    Storage_NVS.sector_count = STORAGE_NVS_SECTOR_COUNT;
    Storage_NVS.flash_device = NULL;

    return nvs_mount(&Storage_NVS);
}

int NVS_Storage_Smoke_Test(void)
{
    const uint32_t Pattern1 = 0x13579BDFU;
    const uint32_t Pattern2 = 0x2468ACE0U;
    uint32_t Readback = 0U;
    ssize_t Rc;
    int Status;

    NVS_Storage_Smoke_Result = -100;
    NVS_Storage_Smoke_Readback = 0U;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -1;
        return -1;
    }

    Status = nvs_clear(&Storage_NVS);
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -2;
        return -2;
    }

    /* Zephyr nvs_clear() deliberately marks the filesystem unmounted. */
    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -3;
        return -3;
    }

    Rc = nvs_write(&Storage_NVS, STORAGE_SMOKE_ID, &Pattern1, sizeof(Pattern1));
    if (Rc != (ssize_t)sizeof(Pattern1))
    {
        NVS_Storage_Smoke_Result = -4;
        return -4;
    }

    Rc = nvs_read(&Storage_NVS, STORAGE_SMOKE_ID, &Readback, sizeof(Readback));
    if ((Rc != (ssize_t)sizeof(Readback)) || (Readback != Pattern1))
    {
        NVS_Storage_Smoke_Readback = Readback;
        NVS_Storage_Smoke_Result = -5;
        return -5;
    }

    Rc = nvs_write(&Storage_NVS, STORAGE_SMOKE_ID, &Pattern2, sizeof(Pattern2));
    if (Rc != (ssize_t)sizeof(Pattern2))
    {
        NVS_Storage_Smoke_Result = -6;
        return -6;
    }

    Readback = 0U;
    Rc = nvs_read(&Storage_NVS, STORAGE_SMOKE_ID, &Readback, sizeof(Readback));
    if ((Rc != (ssize_t)sizeof(Readback)) || (Readback != Pattern2))
    {
        NVS_Storage_Smoke_Readback = Readback;
        NVS_Storage_Smoke_Result = -7;
        return -7;
    }

    Status = nvs_clear(&Storage_NVS);
    if (Status != 0)
    {
        NVS_Storage_Smoke_Result = -8;
        return -8;
    }

    NVS_Storage_Smoke_Readback = Readback;
    NVS_Storage_Smoke_Result = 0;
    return 0;
}
