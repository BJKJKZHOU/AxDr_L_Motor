/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "tx_api.h"

typedef enum
{
    MOTOR_CMD_ENABLE = 0,
    MOTOR_CMD_RUN,
    MOTOR_CMD_STOP,
    MOTOR_CMD_DISABLE,
    MOTOR_CMD_EN_TOGGLE,
    MOTOR_CMD_RUN_TOGGLE,
    MOTOR_CMD_MODE_SET,
    MOTOR_CMD_IDENT_SET,
    MOTOR_CMD_SPEED_SET,
    MOTOR_CMD_TORQUE_SET,
    MOTOR_CMD_POSITION_SET,
    MOTOR_CMD_ENCODER_TYPE_SET,
    MOTOR_CMD_PHASE_CURRENT_SET,
    MOTOR_CMD_IDENT_APPLY,
    MOTOR_CMD_I_LIMIT_SET,
    MOTOR_CMD_PP_SET,

} Motor_Cmd_e;

typedef struct
{
    ULONG Cmd;
    ULONG Arg;
    ULONG Arg2;
    ULONG Reserved;

} Motor_Cmd_Msg_T;

extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Motor_Cmd_Q;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);

#endif /* MOTOR_THREAD_H */
