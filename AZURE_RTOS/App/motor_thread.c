/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "motor_thread.h"

#include <stdbool.h>
#include <string.h>

#include "Motor_Control.h"
#include "NVS_Storage.h"
#include "Parameter.h"
#include "Plot.h"
#include "Protocol.h"
#include "Protection.h"
#include "Servo_Phase.h"
#include "USB_Thread.h"
#include "main.h"

#define MOTOR_STACK_SIZE  512U
#define MOTOR_THREAD_PRIO 5U
#define MOTOR_CMD_Q_LEN   8U
#define MOTOR_CMD_Q_WORDS 5U

TX_SEMAPHORE Motor_Sem;
TX_QUEUE Motor_Cmd_Q;
volatile ULONG Motor_Ready = 0U;
volatile Motor_Time_T Motor_Time = { 0 };

static TX_THREAD Motor_Thread;
static uint8_t Normal_Div = 0U;
static bool Ident_Action_Pending = false;
static Motor_Cmd_Msg_T Ident_Action_Msg = { 0 };
static bool Phase_Action_Pending = false;
static Motor_Cmd_Msg_T Phase_Action_Msg = { 0 };

static void Motor_Entry(ULONG thread_input);

static void Motor_Time_Set(uint32_t Cyc, volatile uint32_t *Current, volatile uint32_t *Max)
{
    *Current = Cyc;
    if (Cyc > *Max)
    {
        *Max = Cyc;
    }
}

Motor_Parameter_Request_Status_e Motor_Parameter_Write_Request(
    uint16_t Id,
    Parameter_Type_e Type,
    const Parameter_Value_T *Value,
    uint8_t Txn)
{
    Motor_Cmd_Msg_T Msg = { 0 };

    if (Type == PARAM_ACTION)
    {
        switch (Id)
        {
#define PARAM_GENERATE_ACTION
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_ACTION
            default:
                return MOTOR_PARAM_REQUEST_ERR_ID;
        }

        Msg.Reserved = ((ULONG)Id << MOTOR_ACTION_ID_SHIFT) |
                       ((ULONG)Txn << MOTOR_ACTION_TXN_SHIFT);
    }
    else
    {
        Msg.Cmd = (ULONG)MOTOR_CMD_PARAMETER_WRITE;
        Msg.Arg = (ULONG)Id;
        memcpy(&Msg.Arg2, Value, sizeof(Msg.Arg2));
        memcpy(&Msg.Arg3,
               (const uint8_t *)Value + sizeof(Msg.Arg2),
               sizeof(Msg.Arg3));
        Msg.Reserved = (ULONG)Type | ((ULONG)Txn << MOTOR_PARAM_TXN_SHIFT);
    }

    if (tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT) != TX_SUCCESS)
    {
        return MOTOR_PARAM_REQUEST_ERR_QUEUE;
    }

    return MOTOR_PARAM_REQUEST_OK;
}

static uint16_t Motor_Action_Id_Get(const Motor_Cmd_Msg_T *Msg)
{
    return (uint16_t)((Msg->Reserved >> MOTOR_ACTION_ID_SHIFT) & MOTOR_ACTION_ID_MASK);
}

static uint8_t Motor_Action_Txn_Get(const Motor_Cmd_Msg_T *Msg)
{
    return (uint8_t)((Msg->Reserved >> MOTOR_ACTION_TXN_SHIFT) & MOTOR_ACTION_TXN_MASK);
}

static void Motor_Action_Response(const Motor_Cmd_Msg_T *Msg, AxDr_Status_e Status)
{
    Protocol_Action_Response(Motor_Action_Txn_Get(Msg),
                             Motor_Action_Id_Get(Msg),
                             Status);
}

static void Motor_Action_Complete(const Motor_Cmd_Msg_T *Msg, AxDr_Status_e Status)
{
    Protocol_Action_Complete(Motor_Action_Txn_Get(Msg),
                             Motor_Action_Id_Get(Msg),
                             Status);
}

static void Motor_Async_Action_Poll(void)
{
    if (Ident_Action_Pending && !Identification_Active())
    {
        Motor_Action_Complete(&Ident_Action_Msg,
                              Identification_Result_Valid() ? AXDR_OK : AXDR_ERR_CONFIG);
        Ident_Action_Pending = false;
    }

    if (Phase_Action_Pending && !Servo_Phase_Active())
    {
        Motor_Action_Complete(&Phase_Action_Msg,
                              Servo_Phase_Result_Valid() ? AXDR_OK : AXDR_ERR_CONFIG);
        Phase_Action_Pending = false;
    }
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
                    else if (Motor_Mode_Get() == PHASE_SEARCH)
                    {
                        Phase_Action_Msg = Msg;
                        Phase_Action_Pending = true;
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
                    /*
                     * Stop is accepted here but motion modes may remain RUN
                     * until their configured deceleration reaches zero.
                     */
                    Motor_Stop();
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
                else
                {
                    Ident_Action_Msg = Msg;
                    Ident_Action_Pending = true;
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

                Protocol_Parameter_Write_Response(
                    (uint8_t)(Msg.Reserved >> MOTOR_PARAM_TXN_SHIFT),
                    (uint16_t)Msg.Arg,
                    Parameter_Status);
                break;

            case MOTOR_CMD_IDENT_APPLY:
                Action_Status = AXDR_OK;
                if ((Motor_State_Get() == RUN) ||
                    (Motor_Mode_Get() != IDENT) ||
                    !Identification_Result_Valid())
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

            case MOTOR_CMD_PARAMETER_SAVE:
                Action_Status = AXDR_OK;
                if (Motor_State_Get() != DISABLED)
                {
                    Action_Status = AXDR_ERR_STATE;
                }
                else if (NVS_Storage_Save_All() != 0)
                {
                    Action_Status = AXDR_ERR_CONFIG;
                }
                Motor_Action_Response(&Msg, Action_Status);
                break;

            default:
                break;
        }

        /* Close any finite action ended by this command before another command can reuse its context. */
        Motor_Async_Action_Poll();
    }
}

UINT Motor_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    /* Restore persisted configuration before the Motor thread starts running.
     * Missing or invalid records leave the compiled defaults in place. */
    (void)NVS_Storage_Load_All();

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
    uint32_t Total_T0;
    uint32_t T0;
    uint32_t Cyc;
    uint32_t Sem_Count;

    (void)thread_input;

    Motor_Ready = 1U;

    while (1)
    {
        if (tx_semaphore_get(&Motor_Sem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            Total_T0 = DWT->CYCCNT;

            /* Count remaining tokens after consuming this activation. A nonzero
             * value means the 2 kHz Motor Thread has already fallen behind. */
            Sem_Count = (uint32_t)Motor_Sem.tx_semaphore_count;
            Motor_Time.Sem_Count = Sem_Count;
            if (Sem_Count > Motor_Time.Sem_Max)
            {
                Motor_Time.Sem_Max = Sem_Count;
            }

            T0 = DWT->CYCCNT;
            Protection_Control();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Protection_Cyc, &Motor_Time.Protection_Max);

            T0 = DWT->CYCCNT;
            Motor_Cmd_Run();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Cmd_Cyc, &Motor_Time.Cmd_Max);

            T0 = DWT->CYCCNT;
            Motor_Control();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Control_Cyc, &Motor_Time.Control_Max);

            T0 = DWT->CYCCNT;
            Motor_Async_Action_Poll();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Async_Cyc, &Motor_Time.Async_Max);

            T0 = DWT->CYCCNT;
            Protocol_Event_Poll();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Event_Cyc, &Motor_Time.Event_Max);

            T0 = DWT->CYCCNT;
            USB_Tx_Poll();
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.USB_Poll_Cyc, &Motor_Time.USB_Poll_Max);

            Normal_Div ^= 1U;

            T0 = DWT->CYCCNT;
            if (Normal_Div == 0U)
            {
                Motor_Plot_Normal_Update();
                Plot_Normal_Sample();
            }
            Cyc = DWT->CYCCNT - T0;
            Motor_Time_Set(Cyc, &Motor_Time.Plot_Cyc, &Motor_Time.Plot_Max);

            Cyc = DWT->CYCCNT - Total_T0;
            Motor_Time_Set(Cyc, &Motor_Time.Total_Cyc, &Motor_Time.Total_Max);
        }
    }
}
