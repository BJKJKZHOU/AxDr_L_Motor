/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Flash_Storage.h"

#include <errno.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include <zephyr/drivers/flash.h>

_Static_assert(STORAGE_NVS_OFFSET + STORAGE_NVS_SIZE == STORAGE_FLASH_TOTAL_SIZE,
               "NVS partition must end at flash boundary");
_Static_assert((STORAGE_NVS_OFFSET % STORAGE_NVS_SECTOR_SIZE) == 0U,
               "NVS partition must start on a flash page boundary");

static const struct flash_parameters Flash_Parameters = {
    .write_block_size = STORAGE_FLASH_WRITE_SIZE,
    .caps = { .no_explicit_erase = false },
    .erase_value = 0xFFU,
};

static bool Storage_Range_Valid(off_t Offset, size_t Length)
{
    uint32_t Start;
    uint32_t End;

    if (Offset < 0)
    {
        return false;
    }

    Start = (uint32_t)Offset;
    End = Start + (uint32_t)Length;

    return (End >= Start) &&
           (Start >= STORAGE_NVS_OFFSET) &&
           (End <= (STORAGE_NVS_OFFSET + STORAGE_NVS_SIZE));
}

bool Flash_Storage_Geometry_Valid(void)
{
    return (FLASH->OPTR & FLASH_OPTR_DBANK) != 0U;
}

int flash_read(const struct device *Dev, off_t Offset, void *Data, size_t Length)
{
    (void)Dev;

    if ((Data == NULL) || !Storage_Range_Valid(Offset, Length))
    {
        return -EINVAL;
    }

    memcpy(Data, (const void *)(FLASH_BASE + (uint32_t)Offset), Length);
    return 0;
}

int flash_write(const struct device *Dev, off_t Offset, const void *Data, size_t Length)
{
    uint64_t Word;
    uint32_t Address;
    const uint8_t *Src;

    (void)Dev;

    if ((Data == NULL) || !Flash_Storage_Geometry_Valid() ||
        !Storage_Range_Valid(Offset, Length) ||
        (((uint32_t)Offset % STORAGE_FLASH_WRITE_SIZE) != 0U) ||
        ((Length % STORAGE_FLASH_WRITE_SIZE) != 0U))
    {
        return -EINVAL;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return -EIO;
    }

    Address = FLASH_BASE + (uint32_t)Offset;
    Src = (const uint8_t *)Data;

    for (size_t n = 0U; n < Length; n += STORAGE_FLASH_WRITE_SIZE)
    {
        memcpy(&Word, &Src[n], sizeof(Word));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address + (uint32_t)n, Word) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return -EIO;
        }
    }

    (void)HAL_FLASH_Lock();
    return 0;
}

int flash_flatten(const struct device *Dev, off_t Offset, size_t Size)
{
    FLASH_EraseInitTypeDef Erase = { 0 };
    uint32_t PageError = 0U;
    uint32_t Page;

    (void)Dev;

    if (!Flash_Storage_Geometry_Valid() ||
        !Storage_Range_Valid(Offset, Size) ||
        (((uint32_t)Offset % STORAGE_NVS_SECTOR_SIZE) != 0U) ||
        ((Size == 0U) || ((Size % STORAGE_NVS_SECTOR_SIZE) != 0U)))
    {
        return -EINVAL;
    }

    Page = ((uint32_t)Offset - 0x00040000UL) / STORAGE_NVS_SECTOR_SIZE;

    Erase.TypeErase = FLASH_TYPEERASE_PAGES;
    Erase.Banks = FLASH_BANK_2;
    Erase.Page = Page;
    Erase.NbPages = (uint32_t)(Size / STORAGE_NVS_SECTOR_SIZE);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return -EIO;
    }

    if (HAL_FLASHEx_Erase(&Erase, &PageError) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return -EIO;
    }

    (void)HAL_FLASH_Lock();
    return 0;
}

const struct flash_parameters *flash_get_parameters(const struct device *Dev)
{
    (void)Dev;
    return &Flash_Parameters;
}

size_t flash_get_write_block_size(const struct device *Dev)
{
    (void)Dev;
    return STORAGE_FLASH_WRITE_SIZE;
}

int flash_get_page_info_by_offs(const struct device *Dev,
                                off_t Offset,
                                struct flash_pages_info *Info)
{
    uint32_t PageStart;

    (void)Dev;

    if ((Info == NULL) || !Storage_Range_Valid(Offset, 1U))
    {
        return -EINVAL;
    }

    PageStart = ((uint32_t)Offset / STORAGE_NVS_SECTOR_SIZE) * STORAGE_NVS_SECTOR_SIZE;
    Info->start_offset = (off_t)PageStart;
    Info->size = STORAGE_NVS_SECTOR_SIZE;
    Info->index = PageStart / STORAGE_NVS_SECTOR_SIZE;
    return 0;
}
