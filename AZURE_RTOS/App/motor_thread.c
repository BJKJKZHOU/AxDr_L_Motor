/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "motor_thread.h"

#include <stdbool.h>
#include <string.h>

#include "Encoder.h"
#include "Mechanical_ESO.h"
#include "Motor_Control.h"
#include "Enc_Control.h"
#include "Motor_Para.h"
#include "NVS_Storage.h"
#include "motor_work.h"
#include "Parameter.h"
#include "Plot.h"
#include "Protocol.h"
#include "Protection.h"
#include "Signal_Injection.h"
#include "USB_Thread.h"
#include "main.h"

/* S-curve replanning also needs room for the preempted FPU context. */
#define MOTOR_STACK_SIZE  1024U
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
static ULONG Ident_Action = 0U;
static bool Enc_Action_Pending = false;
static ULONG Enc_Action = 0U;
static bool Save_Action_Pending = false;
static ULONG Save_Action = 0U;

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

static uint16_t Motor_Action_Id_Get(ULONG Action)
{
    return (uint16_t)((Action >> MOTOR_ACTION_ID_SHIFT) & MOTOR_ACTION_ID_MASK);
}

static uint8_t Motor_Action_Txn_Get(ULONG Action)
{
    return (uint8_t)((Action >> MOTOR_ACTION_TXN_SHIFT) & MOTOR_ACTION_TXN_MASK);
}

static void Motor_Action_Response(ULONG Action, AxDr_Status_e Status)
{
    Protocol_Write_Response(Motor_Action_Txn_Get(Action),
                            Motor_Action_Id_Get(Action),
                            Status);
}

static void Motor_Action_Complete(ULONG Action, AxDr_Status_e Status)
{
    Protocol_Action_Complete(Motor_Action_Txn_Get(Action),
                             Motor_Action_Id_Get(Action),
                             Status);
}

static void Motor_Async_Action_Poll(void)
{
    if (Ident_Action_Pending && (Ident_State != IDENT_RUNNING) && (Ident_State != IDENT_FINISH))
    {
        Motor_Action_Complete(Ident_Action,
                              (Ident_State == IDENT_DONE) ? AXDR_OK : AXDR_ERR_CONFIG);
        Ident_Action_Pending = false;
    }

    if (Save_Action_Pending && (Motor_Work_State >= MOTOR_WORK_DONE))
    {
        Motor_Action_Response(Save_Action,
                              (Motor_Work_State == MOTOR_WORK_DONE) ? AXDR_OK : AXDR_ERR_CONFIG);
        Save_Action_Pending = false;
        Motor_Work_Ack();
    }

    if (Enc_Action_Pending && !Enc_Control_Active())
    {
        Motor_Action_Complete(Enc_Action, (Enc_Step == ENC_DONE) ? AXDR_OK : AXDR_ERR_CONFIG);
        Enc_Action_Pending = false;
    }
}

static AxDr_Status_e Motor_Status_Map(Motor_Status_e Status)
{
    switch (Status)
    {
        case MOTOR_OK:
            return AXDR_OK;
        case MOTOR_ERR_STATE:
            return AXDR_ERR_STATE;
        case MOTOR_ERR_CONFIG:
        default:
            return AXDR_ERR_CONFIG;
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
                Action_Status = Enc_Action_Pending ? AXDR_ERR_STATE :
                                Motor_Status_Map(Motor_Enable());
                Motor_Action_Response(Msg.Reserved, Action_Status);
                break;

            case MOTOR_CMD_RUN:
                Action_Status = Enc_Action_Pending ? AXDR_ERR_STATE :
                                Motor_Status_Map(Motor_Start());
                if ((Action_Status == AXDR_OK) && (Motor_Mode == PHASE_SEARCH))
                {
                    Enc_Action = Msg.Reserved;
                    Enc_Action_Pending = true;
                }
                Motor_Action_Response(Msg.Reserved, Action_Status);
                break;

            case MOTOR_CMD_STOP:
                Motor_Stop();
                Motor_Action_Response(Msg.Reserved, AXDR_OK);
                break;

            case MOTOR_CMD_DISABLE:
                Motor_Disable();
                Motor_Action_Response(Msg.Reserved, AXDR_OK);
                break;

            case MOTOR_CMD_EN_TOGGLE:
                if (Enc_Control_Active())
                {
                    Motor_Disable();
                    break;
                }
                State = Motor_State_Get();

                if ((State == DISABLED) && (Motor_Work_State != MOTOR_WORK_IDLE))
                {
                    break;
                }

                if (State == DISABLED)
                {
                    (void)Motor_Enable();
                }
                else
                {
                    Motor_Disable();
                }
                break;

            case MOTOR_CMD_RUN_TOGGLE:
                if (Enc_Control_Active())
                {
                    Motor_Stop();
                    break;
                }
                State = Motor_State_Get();

                if (State == ENABLED)
                {
                    (void)Motor_Start();
                }
                else if (State == RUN)
                {
                    Motor_Stop();
                }
                break;

            case MOTOR_CMD_MODE_NEXT:
                if (Motor_Work_State != MOTOR_WORK_IDLE)
                {
                    break;
                }
                Parameter_Value = (Parameter_Value_T){ 0 };
                switch (Motor_Mode)
                {
                    case TORQUE:
                        Parameter_Value.U8 = (uint8_t)SPEED;
                        break;
                    case SPEED:
                        Parameter_Value.U8 = (uint8_t)POSITION;
                        break;
                    case POSITION:
                        Parameter_Value.U8 = (uint8_t)SENSORLESS_SPEED;
                        break;
                    case SENSORLESS_SPEED:
                    case OPEN_LOOP:
                    case IDENT:
                    case PHASE_SEARCH:
                        Parameter_Value.U8 = (uint8_t)TORQUE;
                        break;
                    default:
                        continue;
                }
                (void)Parameter_Write(PARAM_MOTOR_MODE, PARAM_U8, Parameter_Value);
                break;

            case MOTOR_CMD_ENC_CAL_START:
                Action_Status = Enc_Action_Pending ? AXDR_ERR_STATE :
                                Motor_Status_Map(Motor_Enc_Cal_Start());
                if (Action_Status == AXDR_OK)
                {
                    Enc_Action = Msg.Reserved;
                    Enc_Action_Pending = true;
                }
                Motor_Action_Response(Msg.Reserved, Action_Status);
                break;

            case MOTOR_CMD_IDENT_START:
                Action_Status = Motor_Status_Map(Motor_Ident_Start((Ident_Mode_e)Msg.Arg));
                if (Action_Status == AXDR_OK)
                {
                    Ident_Action = Msg.Reserved;
                    Ident_Action_Pending = true;
                }
                Motor_Action_Response(Msg.Reserved, Action_Status);
                break;

            case MOTOR_CMD_PARAMETER_WRITE:
                memcpy(&Parameter_Value, &Msg.Arg2, sizeof(Msg.Arg2));
                memcpy((uint8_t *)&Parameter_Value + sizeof(Msg.Arg2),
                       &Msg.Arg3,
                       sizeof(Msg.Arg3));
                Parameter_Status = (Motor_Work_State == MOTOR_WORK_IDLE) ?
                                   Parameter_Write((uint16_t)Msg.Arg,
                                                   (Parameter_Type_e)(Msg.Reserved & 0xFFU),
                                                   Parameter_Value) : PARAM_ERR_STATE;

                Protocol_Parameter_Write_Response(
                    (uint8_t)(Msg.Reserved >> MOTOR_PARAM_TXN_SHIFT),
                    (uint16_t)Msg.Arg,
                    Parameter_Status);
                break;

            case MOTOR_CMD_IDENT_APPLY:
                Motor_Action_Response(Msg.Reserved, Motor_Status_Map(Motor_Ident_Apply()));
                break;

            case MOTOR_CMD_PROTECTION_CLEAR:
                Action_Status = Protection_Clear() ? AXDR_OK : AXDR_ERR_STATE;
                Motor_Action_Response(Msg.Reserved, Action_Status);
                break;

            case MOTOR_CMD_SIGNAL_START:
                Motor_Action_Response(Msg.Reserved, Motor_Status_Map(Motor_Signal_Start()));
                break;

            case MOTOR_CMD_SIGNAL_ABORT:
                Signal_Injection_Stop();
                Motor_Action_Response(Msg.Reserved, AXDR_OK);
                break;

            case MOTOR_CMD_PARAMETER_SAVE:
                if ((Motor_State_Get() != DISABLED) ||
                    (Motor_Work_State != MOTOR_WORK_IDLE) || Enc_Control_Active())
                {
                    Motor_Action_Response(Msg.Reserved, AXDR_ERR_STATE);
                }
                else if (!Motor_Work_Request(MOTOR_WORK_SAVE_ALL))
                {
                    Motor_Action_Response(Msg.Reserved, AXDR_ERR_CONFIG);
                }
                else
                {
                    /* Keep the original final ActionResponse after NVS finishes. */
                    Save_Action = Msg.Reserved;
                    Save_Action_Pending = true;
                }
                break;

            default:
                break;
        }

        /* 先通知被本条命令结束的动作，再处理后续命令，避免覆盖原请求信息。 */
        Motor_Async_Action_Poll();
    }
}

UINT Motor_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;
    Motor_Para_T Model = Motor_Para;
    float Eso_Bw_Hz = Mechanical_ESO_Bw_Hz;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    /* Motor_Ready 置位前完成加载和配置，此时编码器尚未开始采样。 */
    (void)NVS_Storage_Load_All();
    if (!Motor_Para_Update(&Motor_Para, Mechanical_ESO_Bw_Hz))
    {
        /* 保留原有模型，其他参数仍使用按表恢复的值。 */
        Motor_Para = Model;
        Mechanical_ESO_Bw_Hz = Eso_Bw_Hz;
        Current_Tuning_Update();
        Speed_Tuning_Update();
    }
    Encoder_DMA_Config();

    UINT Work_Result = Motor_Work_Thread_Init(byte_pool);
    if (Work_Result != TX_SUCCESS)
    {
        return Work_Result;
    }

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
