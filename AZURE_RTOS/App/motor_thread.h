/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include <stdint.h>

#include "Parameter.h"
#include "tx_api.h"

#define MOTOR_PARAM_TXN_SHIFT 8U

/* Action context carried with the internal Motor queue message. */
#define MOTOR_ACTION_ID_SHIFT  0U
#define MOTOR_ACTION_ID_MASK   0xFFFFUL
#define MOTOR_ACTION_TXN_SHIFT 16U
#define MOTOR_ACTION_TXN_MASK  0xFFUL

typedef enum
{
    MOTOR_CMD_ENABLE = 0,
    MOTOR_CMD_RUN,
    MOTOR_CMD_STOP,
    MOTOR_CMD_DISABLE,
    MOTOR_CMD_EN_TOGGLE,
    MOTOR_CMD_RUN_TOGGLE,
    MOTOR_CMD_IDENT_START,
    MOTOR_CMD_IDENT_ABORT,
    MOTOR_CMD_PARAMETER_WRITE,
    MOTOR_CMD_IDENT_APPLY,
    MOTOR_CMD_PROTECTION_CLEAR,
    MOTOR_CMD_PARAMETER_SAVE,
    MOTOR_CMD_SIGNAL_START,
    MOTOR_CMD_SIGNAL_ABORT,

} Motor_Cmd_e;

typedef struct
{
    ULONG Cmd;
    ULONG Arg;
    ULONG Arg2;
    ULONG Arg3;
    ULONG Reserved;

} Motor_Cmd_Msg_T;

typedef enum
{
    MOTOR_PARAM_REQUEST_OK = 0,
    MOTOR_PARAM_REQUEST_ERR_ID,
    MOTOR_PARAM_REQUEST_ERR_QUEUE,

} Motor_Parameter_Request_Status_e;

/* DWT cycle timing for the 2 kHz Motor Thread. Current and Max are kept
 * separately so a non-halting SWD trace can inspect both transient load and
 * worst-case load without adding any runtime logging. */
typedef struct
{
    uint32_t Total_Cyc;
    uint32_t Total_Max;
    uint32_t Protection_Cyc;
    uint32_t Protection_Max;
    uint32_t Cmd_Cyc;
    uint32_t Cmd_Max;
    uint32_t Control_Cyc;
    uint32_t Control_Max;
    uint32_t Async_Cyc;
    uint32_t Async_Max;
    uint32_t Event_Cyc;
    uint32_t Event_Max;
    uint32_t USB_Poll_Cyc;
    uint32_t USB_Poll_Max;
    uint32_t Plot_Cyc;
    uint32_t Plot_Max;
    uint32_t Sem_Count;
    uint32_t Sem_Max;

} Motor_Time_T;

extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Motor_Cmd_Q;
extern volatile ULONG Motor_Ready;
extern volatile Motor_Time_T Motor_Time;

Motor_Parameter_Request_Status_e Motor_Parameter_Write_Request(
    uint16_t Id,
    Parameter_Type_e Type,
    const Parameter_Value_T *Value,
    uint8_t Txn);
UINT Motor_Thread_Init(VOID *memory_ptr);

#endif /* MOTOR_THREAD_H */
