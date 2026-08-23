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
    MOTOR_CMD_SENSORLESS_DIR_SET,
    MOTOR_CMD_SENSORLESS_TARGET_SET,
    MOTOR_CMD_SENSORLESS_SPEED_SET,
    MOTOR_CMD_IDENT_APPLY,

} Motor_Cmd_e;

extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Motor_Cmd_Q;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);

#endif /* MOTOR_THREAD_H */
