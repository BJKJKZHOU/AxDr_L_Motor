/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "NVS_Storage.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "Flash_Storage.h"
#include "Parameter.h"
#include <zephyr/kvss/nvs.h>

static struct nvs_fs Storage_NVS = {
    .offset = STORAGE_NVS_OFFSET,
    .sector_size = STORAGE_NVS_SECTOR_SIZE,
    .sector_count = STORAGE_NVS_SECTOR_COUNT,
};

static int NVS_Storage_Init(void)
{
    if (Storage_NVS.ready)
    {
        return 0;
    }

    if (!Flash_Storage_Geometry_Valid())
    {
        return -ENOTSUP;
    }

    return nvs_mount(&Storage_NVS);
}

int NVS_Storage_Load_All(void)
{
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    uint32_t Index = 0U;
    uint16_t Id;
    uint8_t Size;
    ssize_t Rc;
    int Status;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    while (Parameter_Persistent_Next(&Index, &Id, &Type, &Value))
    {
        Size = Parameter_Value_Size(Type);
        if (Size == 0U)
        {
            continue;
        }

        memset(&Value, 0, sizeof(Value));
        Rc = nvs_read(&Storage_NVS, Id, &Value, Size);
        if (Rc == (ssize_t)Size)
        {
            (void)Parameter_Restore(Id, Type, Value);
        }
    }

    return 0;
}

static int Record_Save(uint16_t Id, Parameter_Type_e Type, Parameter_Value_T Value)
{
    uint8_t Size = Parameter_Value_Size(Type);
    ssize_t Rc;

    if (Size == 0U)
    {
        return -EINVAL;
    }

    Rc = nvs_write(&Storage_NVS, Id, &Value, Size);
    if (Rc < 0)
    {
        return (int)Rc;
    }

    /* Zephyr NVS also returns zero when the value is already stored. */
    return ((Rc == 0) || (Rc == (ssize_t)Size)) ? 0 : -EIO;
}

int NVS_Storage_Save_All(void)
{
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    uint32_t Index = 0U;
    uint16_t Id;
    int Status;

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    while (Parameter_Persistent_Next(&Index, &Id, &Type, &Value))
    {
        Status = Record_Save(Id, Type, Value);
        if (Status != 0)
        {
            return Status;
        }
    }

    return 0;
}

int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count)
{
    Parameter_Value_T Value;
    Parameter_Type_e Type;
    int Status;

    if ((Count != 0U) && (Ids == NULL))
    {
        return -EINVAL;
    }

    if (Count == 0U)
    {
        return 0;
    }

    Status = NVS_Storage_Init();
    if (Status != 0)
    {
        return Status;
    }

    /* 按传入顺序保存，寻相调用将有效标志放在最后。 */
    for (uint16_t n = 0U; n < Count; n++)
    {
        if (!Parameter_Persistent_Read(Ids[n], &Type, &Value))
        {
            return -EINVAL;
        }

        Status = Record_Save(Ids[n], Type, Value);
        if (Status != 0)
        {
            return Status;
        }
    }

    return 0;
}
