/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "motor_work.h"

#include "NVS_Storage.h"
#include "Enc_Cal.h"
#include "Servo_Phase.h"

#define WORK_STACK_SIZE 2048U
#define WORK_THREAD_PRIO 9U

volatile Motor_Work_State_e Motor_Work_State = MOTOR_WORK_IDLE;

static TX_THREAD Work_Thread;
static TX_SEMAPHORE Work_Sem;
static Motor_Work_Job_e Job = MOTOR_WORK_SAVE_ALL;

static void Work_Entry(ULONG Input)
{
    int Result;

    (void)Input;
    while (1)
    {
        if (tx_semaphore_get(&Work_Sem, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }

        if (Job == MOTOR_WORK_FIT)
        {
            Result = Enc_Cal_Fit_Apply() ? 0 : -1;
        }
        else if (Job == MOTOR_WORK_SAVE_PHASE)
        {
            Result = Servo_Phase_Save() ? 0 : -1;
        }
        else if (Job == MOTOR_WORK_SAVE_ENC)
        {
            Result = Enc_Cal_Save() ? 0 : -1;
        }
        else
        {
            Result = NVS_Storage_Save_All();
        }
        Motor_Work_State = (Result == 0) ? MOTOR_WORK_DONE : MOTOR_WORK_FAILED;
    }
}

bool Motor_Work_Request(Motor_Work_Job_e Request)
{
    if (Motor_Work_State != MOTOR_WORK_IDLE)
    {
        return false;
    }

    Job = Request;
    Motor_Work_State = MOTOR_WORK_BUSY;
    if (tx_semaphore_put(&Work_Sem) != TX_SUCCESS)
    {
        Motor_Work_State = MOTOR_WORK_IDLE;
        return false;
    }
    return true;
}

void Motor_Work_Ack(void)
{
    if (Motor_Work_State >= MOTOR_WORK_DONE)
    {
        Motor_Work_State = MOTOR_WORK_IDLE;
    }
}

UINT Motor_Work_Thread_Init(TX_BYTE_POOL *Pool)
{
    CHAR *Stack;

    if (tx_semaphore_create(&Work_Sem, "Motor Work", 0U) != TX_SUCCESS)
    {
        return TX_SEMAPHORE_ERROR;
    }

    if (tx_byte_allocate(Pool, (VOID **)&Stack, WORK_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&Work_Thread, "Motor Work", Work_Entry, 0U,
                         Stack, WORK_STACK_SIZE, WORK_THREAD_PRIO, WORK_THREAD_PRIO,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }
    return TX_SUCCESS;
}
