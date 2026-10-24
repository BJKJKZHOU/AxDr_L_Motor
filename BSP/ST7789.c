/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "ST7789.h"

#include "main.h"
#include "spi.h"
#include "tx_api.h"

#define LCD_SPI_TIMEOUT_MS 50U

#define LCD_CMD_SWRESET 0x01U
#define LCD_CMD_SLPOUT  0x11U
#define LCD_CMD_NORON   0x13U
#define LCD_CMD_INVON   0x21U
#define LCD_CMD_DISPON  0x29U
#define LCD_CMD_CASET   0x2AU
#define LCD_CMD_RASET   0x2BU
#define LCD_CMD_RAMWR   0x2CU
#define LCD_CMD_MADCTL  0x36U
#define LCD_CMD_COLMOD  0x3AU

static bool LCD_Online = false;
static bool LCD_Tx_Active = false;
static TX_SEMAPHORE LCD_Done;
static uint8_t LCD_Black_Line[ST7789_WIDTH * 2U] = { 0 };

static bool LCD_Command(uint8_t Cmd, const uint8_t *Data, uint16_t Count)
{
    HAL_StatusTypeDef Status;

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    Status = HAL_SPI_Transmit(&hspi3, &Cmd, 1U, LCD_SPI_TIMEOUT_MS);

    if ((Status == HAL_OK) && (Count != 0U))
    {
        HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
        Status = HAL_SPI_Transmit(&hspi3, (uint8_t *)Data, Count, LCD_SPI_TIMEOUT_MS);
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    return Status == HAL_OK;
}

static bool LCD_Window(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height)
{
    uint16_t X0 = X + ST7789_X_OFFSET;
    uint16_t X1 = X0 + Width - 1U;
    uint16_t Y0 = Y + ST7789_Y_OFFSET;
    uint16_t Y1 = Y0 + Height - 1U;
    uint8_t Col[4] = { (uint8_t)(X0 >> 8U), (uint8_t)X0,
                       (uint8_t)(X1 >> 8U), (uint8_t)X1 };
    uint8_t Row[4] = { (uint8_t)(Y0 >> 8U), (uint8_t)Y0,
                       (uint8_t)(Y1 >> 8U), (uint8_t)Y1 };

    return LCD_Command(LCD_CMD_CASET, Col, sizeof(Col)) &&
           LCD_Command(LCD_CMD_RASET, Row, sizeof(Row));
}

static bool LCD_RAM_Write_Begin(void)
{
    uint8_t Cmd = LCD_CMD_RAMWR;

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi3, &Cmd, 1U, LCD_SPI_TIMEOUT_MS) != HAL_OK)
    {
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    return true;
}

static bool LCD_Clear(void)
{
    if (!LCD_Window(0U, 0U, ST7789_WIDTH, ST7789_HEIGHT) ||
        !LCD_RAM_Write_Begin())
    {
        return false;
    }

    for (uint16_t y = 0U; y < ST7789_HEIGHT; y++)
    {
        if (HAL_SPI_Transmit(&hspi3, LCD_Black_Line, sizeof(LCD_Black_Line),
                             LCD_SPI_TIMEOUT_MS) != HAL_OK)
        {
            HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
            return false;
        }
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    return true;
}

bool ST7789_Init(void)
{
    static const uint8_t Format = 0x55U; /* RGB565 */
    static const uint8_t Direction = ST7789_MADCTL;

    /* SPI3 is also wired to a separate CS. Do not select both devices. */
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    /* Q4 is a PNP high-side switch: PB7 high keeps the backlight off. */
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET);

    /* Only enable the panel after CubeMX generates and links SPI3 TX DMA. */
    if (hspi3.hdmatx == NULL)
    {
        return false;
    }

    if (tx_semaphore_create(&LCD_Done, "LCD DMA", 0U) != TX_SUCCESS)
    {
        return false;
    }

    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(20U);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);

    if (!LCD_Command(LCD_CMD_SWRESET, NULL, 0U))
    {
        return false;
    }
    HAL_Delay(150U);

    if (!LCD_Command(LCD_CMD_SLPOUT, NULL, 0U))
    {
        return false;
    }
    HAL_Delay(120U);

    if (!LCD_Command(LCD_CMD_COLMOD, &Format, 1U) ||
        !LCD_Command(LCD_CMD_MADCTL, &Direction, 1U) ||
        !LCD_Command(LCD_CMD_INVON, NULL, 0U) ||
        !LCD_Command(LCD_CMD_NORON, NULL, 0U) ||
        !LCD_Clear() ||
        !LCD_Command(LCD_CMD_DISPON, NULL, 0U))
    {
        return false;
    }

    HAL_Delay(20U);
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_RESET);
    LCD_Online = true;
    return true;
}

bool ST7789_Ready(void)
{
    if (!LCD_Online)
    {
        return false;
    }

    if (!LCD_Tx_Active)
    {
        return true;
    }

    if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY_TX)
    {
        return false;
    }

    LCD_Tx_Active = false;
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

    if ((HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY) ||
        (HAL_SPI_GetError(&hspi3) != HAL_SPI_ERROR_NONE))
    {
        LCD_Online = false;
        return false;
    }

    return true;
}

bool ST7789_Write(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height,
                  const uint8_t *Pixels)
{
    uint32_t Bytes = (uint32_t)Width * Height * 2U;

    if (!ST7789_Ready() || (Pixels == NULL) ||
        (Width == 0U) || (Height == 0U) ||
        ((uint32_t)X + Width > ST7789_WIDTH) ||
        ((uint32_t)Y + Height > ST7789_HEIGHT) ||
        (Bytes > UINT16_MAX))
    {
        return false;
    }

    if (!LCD_Window(X, Y, Width, Height) || !LCD_RAM_Write_Begin())
    {
        return false;
    }

    if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)Pixels, (uint16_t)Bytes) != HAL_OK)
    {
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
        return false;
    }

    LCD_Tx_Active = true;
    return true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *Spi)
{
    if ((Spi->Instance == SPI3) && LCD_Online)
    {
        (void)tx_semaphore_put(&LCD_Done);
    }
}

/* DMA 完成时唤醒，最长等待一个 tick，以便及时扫描按键和刷新画面。 */
void ST7789_Wait(void)
{
    if (LCD_Online)
    {
        (void)tx_semaphore_get(&LCD_Done, 1U);
    }
    else
    {
        tx_thread_sleep(1U);
    }
}
