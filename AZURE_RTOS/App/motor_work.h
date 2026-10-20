/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_WORK_H
#define MOTOR_WORK_H

#include <stdbool.h>
#include "tx_api.h"

typedef enum
{
    MOTOR_WORK_IDLE = 0,
    MOTOR_WORK_BUSY,
    MOTOR_WORK_DONE,
    MOTOR_WORK_FAILED,

} Motor_Work_State_e;

typedef enum
{
    MOTOR_WORK_FIT = 0,
    MOTOR_WORK_SAVE_ALL,
    MOTOR_WORK_SAVE_CAL,

} Motor_Work_Job_e;

extern volatile Motor_Work_State_e Motor_Work_State;

bool Motor_Work_Request(Motor_Work_Job_e Job);
void Motor_Work_Ack(void);
UINT Motor_Work_Thread_Init(TX_BYTE_POOL *Pool);

#endif /* MOTOR_WORK_H */
