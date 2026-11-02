/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "motor_work.h"

#include "NVS_Storage.h"
#include "Enc_Cal.h"
#include "Parameter.h"

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
        (void)tx_semaphore_get(&Work_Sem, TX_WAIT_FOREVER);

        if (Job == MOTOR_WORK_FIT)
        {
            Result = Enc_Cal_Fit_Apply() ? 0 : -1;
        }
        else if (Job == MOTOR_WORK_SAVE_PHASE)
        {
            const uint16_t Ids[] = {
                PARAM_CAL_ENC_DIR, PARAM_CAL_THETA_OFF, PARAM_CAL_VALID,
            };

            /* 先删除旧有效标志，方向和零位保存完后再写入有效标志。 */
            Result = NVS_Storage_Delete(PARAM_CAL_VALID);
            if (Result == 0)
            {
                Result = NVS_Storage_Save_Ids(Ids, sizeof(Ids) / sizeof(Ids[0]));
            }
        }
        else if (Job == MOTOR_WORK_SAVE_ENC)
        {
            const uint16_t Ids[] = {
                PARAM_CAL_VALID,
                PARAM_ENCODER_HARMONIC_C1, PARAM_ENCODER_HARMONIC_S1,
                PARAM_ENCODER_HARMONIC_C2, PARAM_ENCODER_HARMONIC_S2,
            };

            /* 先保存无效标志，再保存新系数。 */
            Result = NVS_Storage_Save_Ids(Ids, sizeof(Ids) / sizeof(Ids[0]));
        }
        else
        {
            Result = NVS_Storage_Save_All();
        }
        Motor_Work_State = (Result == 0) ? MOTOR_WORK_DONE : MOTOR_WORK_FAILED;
    }
}

void Motor_Work_Request(Motor_Work_Job_e Request)
{
    Job = Request;
    Motor_Work_State = MOTOR_WORK_BUSY;
    (void)tx_semaphore_put(&Work_Sem);
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
