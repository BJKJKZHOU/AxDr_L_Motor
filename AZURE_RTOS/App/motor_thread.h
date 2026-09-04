/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "tx_api.h"

#define MOTOR_PARAM_TXN_SHIFT 8U
#define MOTOR_PARAM_RESPONSE  (1UL << 16)

#define MOTOR_ACTION_OP_SHIFT   0U
#define MOTOR_ACTION_MSG_SHIFT  8U
#define MOTOR_ACTION_TXN_SHIFT  16U
#define MOTOR_ACTION_RESPONSE   (1UL << 24)
#define MOTOR_ACTION_BYTE_MASK  0xFFUL

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

} Motor_Cmd_e;

typedef struct
{
    ULONG Cmd;
    ULONG Arg;
    ULONG Arg2;
    ULONG Arg3;
    ULONG Reserved;

} Motor_Cmd_Msg_T;

extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Motor_Cmd_Q;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);

#endif /* MOTOR_THREAD_H */
