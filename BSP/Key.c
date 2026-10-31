/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Key.h"

#include "main.h"

#define KEY_STABLE_CNT 3U

static uint32_t Key_Press(Key_T *Key, GPIO_TypeDef *Port, uint16_t Pin)
{
    uint8_t Sample;

    /* 按键按下为低电平。 */
    Sample = (HAL_GPIO_ReadPin(Port, Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (Sample == Key->State)
    {
        Key->Cnt = 0U;
    }
    else if (++Key->Cnt == KEY_STABLE_CNT)
    {
        Key->Cnt = 0U;
        Key->State = Sample;
        return Sample;
    }

    return 0U;
}

uint32_t Key_Scan(Key_T Keys[3])
{
    uint32_t Press;

    Press = Key_Press(&Keys[0], KEY1_GPIO_Port, KEY1_Pin) * KEY_1;
    Press |= Key_Press(&Keys[1], KEY2_GPIO_Port, KEY2_Pin) * KEY_2;
    Press |= Key_Press(&Keys[2], KEY3_GPIO_Port, KEY3_Pin) * KEY_3;
    return Press;
}
