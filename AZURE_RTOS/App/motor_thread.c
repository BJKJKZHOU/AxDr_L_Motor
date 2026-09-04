/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "motor_thread.h"

#include <string.h>

#include "Motor_Control.h"
#include "Parameter.h"
#include "Plot.h"
#include "Protocol.h"
#include "Protection.h"
#include "USB_Thread.h"

#define MOTOR_STACK_SIZE  512U
#define MOTOR_THREAD_PRIO 5U
#define MOTOR_CMD_Q_LEN   8U
#define MOTOR_CMD_Q_WORDS 5U

TX_SEMAPHORE Motor_Sem;
TX_QUEUE Motor_Cmd_Q;
volatile ULONG Motor_Ready = 0U;

static TX_THREAD Motor_Thread;
static uint8_t Normal_Div = 0U;

static void Motor_Entry(ULONG thread_input);

static void Motor_Action_Response(const Motor_Cmd_Msg_T *Msg, AxDr_Status_e Status)
{
    ULONG Reserved;

    Reserved = Msg->Reserved;
    if ((Reserved & MOTOR_ACTION_RESPONSE) == 0U)
    {
        return;
    }

    Protocol_Action_Response(
        (uint8_t)((Reserved >> MOTOR_ACTION_TXN_SHIFT) & MOTOR_ACTION_BYTE_MASK),
        (uint8_t)((Reserved >> MOTOR_ACTION_MSG_SHIFT) & MOTOR_ACTION_BYTE_MASK),
        (uint8_t)((Reserved >> MOTOR_ACTION_OP_SHIFT) & MOTOR_ACTION_BYTE_MASK),
        Status);
}

static void Motor_Cmd_Run(void)
{
    Motor_State_e State;
    Motor_Cmd_Msg_T Msg;
    AxDr_Status_e Action_Status;
    Parameter_Value_T Parameter_Value;
    Parameter_Status_e Parameter_Status;

    while (tx_queue_receive(&Motor_Cmd_Q, &Msg, TX_NO_WAIT) == TX_SUCCESS)
    {
        switch ((Motor_Cmd_e)Msg.Cmd)
        {
            case MOTOR_CMD_ENABLE:
                Action_Status = AXDR_OK;
                if ((Motor_State_Get() != DISABLED) || !Protection_Enable_Allowed())
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else
                {
                    Motor_Enable();
                    if (Motor_State_Get() != ENABLED)
                    {
                        Action_Status = AXDR_ERR_CONFIG;
                    }
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_RUN:
                Action_Status = AXDR_OK;
                if ((Motor_State_Get() != ENABLED) || (Motor_Mode_Get() == IDENT))
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else
                {
                    Motor_Start();
                    if (Motor_State_Get() != RUN)
                    {
                        Action_Status = AXDR_ERR_CONFIG;
                    }
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_STOP:
                Action_Status = AXDR_OK;
                if (Motor_State_Get() != RUN)
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else
                {
                    Motor_Stop();
                    if (Motor_State_Get() != ENABLED)
                    {
                        Action_Status = AXDR_ERR_CONFIG;
                    }
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_DISABLE:
                Motor_Disable();
                Action_Status = (Motor_State_Get() == DISABLED) ? AXDR_OK : AXDR_ERR_CONFIG;
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_EN_TOGGLE:
                State = Motor_State_Get();

                if (State == DISABLED)
                {
                    if (Protection_Enable_Allowed())
                    {
                        Motor_Enable();
                    }
                }
                else
                {
                    Motor_Disable();
                }
                break;

            case MOTOR_CMD_RUN_TOGGLE:
                State = Motor_State_Get();

                if (State == ENABLED)
                {
                    Motor_Start();
                }
                else if (State == RUN)
                {
                    Motor_Stop();
                }
                break;

            case MOTOR_CMD_IDENT_START:
                Action_Status = AXDR_OK;
                if ((Motor_State_Get() != ENABLED) ||
                    (Motor_Mode_Get() != IDENT) ||
                    !Protection_Enable_Allowed())
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else if (!Motor_Ident_Start((Ident_Mode_e)Msg.Arg))
                {
                    Action_Status = AXDR_ERR_CONFIG;
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_IDENT_ABORT:
                Action_Status = ((Motor_State_Get() == RUN) &&
                                 (Motor_Mode_Get() == IDENT) &&
                                 Identification_Active()) ?
                                    AXDR_OK : AXDR_ERR_STATE;
                if ((Action_Status == AXDR_OK) && !Motor_Ident_Abort())
                {
                    Action_Status = AXDR_ERR_CONFIG;
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_PARAMETER_WRITE:
                memset(&Parameter_Value, 0, sizeof(Parameter_Value));
                memcpy(&Parameter_Value, &Msg.Arg2, sizeof(Msg.Arg2));
                memcpy((uint8_t *)&Parameter_Value + sizeof(Msg.Arg2),
                       &Msg.Arg3,
                       sizeof(Msg.Arg3));
                Parameter_Status = Parameter_Write((uint16_t)Msg.Arg,
                                                   (Parameter_Type_e)(Msg.Reserved & 0xFFU),
                                                   Parameter_Value);

                if ((Msg.Reserved & MOTOR_PARAM_RESPONSE) != 0U)
                {
                    Protocol_Parameter_Write_Response(
                        (uint8_t)(Msg.Reserved >> MOTOR_PARAM_TXN_SHIFT),
                        Parameter_Status);
                }
                break;

            case MOTOR_CMD_IDENT_APPLY:
                Action_Status = AXDR_OK;
                if ((Motor_State_Get() == RUN) ||
                    (Motor_Mode_Get() != IDENT) ||
                    (Identification_State_Get() != IDENT_DONE))
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else if (!Motor_Ident_Apply())
                {
                    Action_Status = AXDR_ERR_CONFIG;
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            case MOTOR_CMD_PROTECTION_CLEAR:
                Action_Status = AXDR_OK;
                if (Motor_State_Get() != DISABLED)
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else if (!Protection_Clear())
                {
                    Action_Status = AXDR_ERR_CONFIG;
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            default:
                break;
        }
    }
}

UINT Motor_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    if (tx_semaphore_create(&Motor_Sem, "Motor Semaphore", 0U) != TX_SUCCESS)
    {
        return TX_SEMAPHORE_ERROR;
    }

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer, MOTOR_CMD_Q_LEN * sizeof(Motor_Cmd_Msg_T), TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_queue_create(&Motor_Cmd_Q,
                        "Motor Command",
                        MOTOR_CMD_Q_WORDS,
                        pointer,
                        MOTOR_CMD_Q_LEN * sizeof(Motor_Cmd_Msg_T)) != TX_SUCCESS)
    {
        return TX_QUEUE_ERROR;
    }

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer, MOTOR_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&Motor_Thread,
                         "Motor Thread",
                         Motor_Entry,
                         0U,
                         pointer,
                         MOTOR_STACK_SIZE,
                         MOTOR_THREAD_PRIO,
                         MOTOR_THREAD_PRIO,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}

static void Motor_Entry(ULONG thread_input)
{
    (void)thread_input;

    Motor_Ready = 1U;

    while (1)
    {
        if (tx_semaphore_get(&Motor_Sem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            Protection_Control();
            Motor_Cmd_Run();
            Motor_Control();
            Protocol_Event_Poll();
            USB_Tx_Poll();

            Normal_Div ^= 1U;

            if (Normal_Div == 0U)
            {
                Plot_Normal_Sample();
            }
        }
    }
}
