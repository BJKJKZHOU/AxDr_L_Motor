/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "panel_thread.h"

#include "LCD_Wave.h"
#include "ST7789.h"
#include "Key.h"
#include "Motor_Control.h"
#include "RGB.h"
#include "motor_thread.h"

/* LCD rendering/HAL nesting plus the Cortex-M4F exception context. */
#define PANEL_STACK_SIZE  1024U
#define PANEL_THREAD_PRIO 12U

static TX_THREAD Panel_Thread;

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
    Key_T Keys[3] = { 0 };
    Motor_State_e State;
    Motor_State_e State_Pre;
    Motor_Cmd_Msg_T Msg = { 0 };
    uint32_t Press;
    uint32_t Tick;
    uint32_t Key_Tick;

    (void)thread_input;

    State_Pre = Motor_State_Get();
    Panel_LED(State_Pre);

    LCD_Wave_Init((uint32_t)tx_time_get());
    Key_Tick = (uint32_t)tx_time_get() - KEY_SCAN_TICK;

    while (1)
    {
        Tick = (uint32_t)tx_time_get();

        if ((uint32_t)(Tick - Key_Tick) >= KEY_SCAN_TICK)
        {
            Key_Tick = Tick;
            Press = Key_Scan(Keys);

            if (((Press & KEY_1) != 0U) || ((Press & KEY_2) != 0U) ||
                (((Press & KEY_3) != 0U) && (Motor_State_Get() == DISABLED)))
            {
                if ((Press & KEY_1) != 0U)
                {
                    Msg.Cmd = (ULONG)MOTOR_CMD_EN_TOGGLE;
                }
                else if ((Press & KEY_2) != 0U)
                {
                    Msg.Cmd = (ULONG)MOTOR_CMD_RUN_TOGGLE;
                }
                else
                {
                    Msg.Cmd = (ULONG)MOTOR_CMD_MODE_NEXT;
                }

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
