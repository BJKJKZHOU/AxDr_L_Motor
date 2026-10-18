/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "panel_thread.h"

#include "LCD_Wave.h"
#include "ST7789.h"
#include "main.h"
#include "Motor_Control.h"
#include "RGB.h"
#include "motor_thread.h"

/* LCD rendering/HAL nesting plus the Cortex-M4F exception context. */
#define PANEL_STACK_SIZE  1024U
#define PANEL_THREAD_PRIO 12U
#define KEY_SCAN_TICK     5U
#define KEY_STABLE_CNT    3U

typedef struct
{
    GPIO_PinState Sample;
    GPIO_PinState State;
    uint8_t Cnt;

} Key_T;

static TX_THREAD Panel_Thread;

static uint32_t Key_Press(Key_T *Key, GPIO_TypeDef *Port, uint16_t Pin)
{
    GPIO_PinState Sample;

    Sample = HAL_GPIO_ReadPin(Port, Pin);

    if (Sample != Key->Sample)
    {
        Key->Sample = Sample;
        Key->Cnt = 1U;
    }
    else if (Key->Cnt < KEY_STABLE_CNT)
    {
        Key->Cnt++;
    }

    if ((Key->Cnt == KEY_STABLE_CNT) && (Sample != Key->State))
    {
        Key->State = Sample;
        return (Sample == GPIO_PIN_RESET) ? 1U : 0U;
    }

    return 0U;
}

static void Panel_LED(Motor_State_e State)
{
    switch (State)
    {
        case DISABLED:
            RGB_Set(0U, 0U, 8U);
            break;

        case ENABLED:
            RGB_Set(8U, 8U, 0U);
            break;

        case RUN:
            RGB_Set(0U, 8U, 0U);
            break;

        default:
            RGB_Set(8U, 0U, 0U);
            break;
    }
}

static void Panel_Entry(ULONG thread_input)
{
    Key_T Key1 = { GPIO_PIN_SET, GPIO_PIN_SET, 0U };
    Key_T Key2 = { GPIO_PIN_SET, GPIO_PIN_SET, 0U };
    Key_T Key3 = { GPIO_PIN_SET, GPIO_PIN_SET, 0U };
    Motor_State_e State;
    Motor_State_e State_Pre;
    Motor_Cmd_Msg_T Msg = { 0 };
    uint32_t Key1_Press;
    uint32_t Key2_Press;
    uint32_t Key3_Press;
    uint32_t Tick;
    uint32_t Key_Tick;

    (void)thread_input;

    State_Pre = Motor_State_Get();
    Panel_LED(State_Pre);

    (void)LCD_Wave_Init((uint32_t)tx_time_get());
    Key_Tick = (uint32_t)tx_time_get() - KEY_SCAN_TICK;

    while (1)
    {
        Tick = (uint32_t)tx_time_get();

        if ((uint32_t)(Tick - Key_Tick) >= KEY_SCAN_TICK)
        {
            Key_Tick = Tick;
            Key1_Press = Key_Press(&Key1, KEY1_GPIO_Port, KEY1_Pin);
            Key2_Press = Key_Press(&Key2, KEY2_GPIO_Port, KEY2_Pin);
            Key3_Press = Key_Press(&Key3, KEY3_GPIO_Port, KEY3_Pin);

            if (Key1_Press != 0U)
            {
                Msg.Cmd = (ULONG)MOTOR_CMD_EN_TOGGLE;
                Msg.Arg = 0U;
                (void)tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT);
            }
            else if (Key2_Press != 0U)
            {
                Msg.Cmd = (ULONG)MOTOR_CMD_RUN_TOGGLE;
                Msg.Arg = 0U;
                (void)tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT);
            }
            else if ((Key3_Press != 0U) && (Motor_State_Get() == DISABLED))
            {
                Msg.Cmd = (ULONG)MOTOR_CMD_MODE_NEXT;
                Msg.Arg = 0U;
                (void)tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT);
            }

            State = Motor_State_Get();

            if (State != State_Pre)
            {
                Panel_LED(State);
                State_Pre = State;
            }
        }

        LCD_Wave_Run(Tick);
        ST7789_Wait();
    }
}

UINT Panel_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer, PANEL_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&Panel_Thread,
                         "Panel Thread",
                         Panel_Entry,
                         0U,
                         pointer,
                         PANEL_STACK_SIZE,
                         PANEL_THREAD_PRIO,
                         PANEL_THREAD_PRIO,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}
