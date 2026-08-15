#include "motor_thread.h"

#include "Motor_Control.h"


TX_THREAD tx_app_thread;
TX_SEMAPHORE Motor_Sem;
volatile ULONG Motor_Ready = 0U;


UINT Motor_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    if (tx_semaphore_create(&Motor_Sem, "Motor Semaphore", 0U) != TX_SUCCESS)
    {
        return TX_SEMAPHORE_ERROR;
    }

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         TX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&tx_app_thread, "Main Thread", MainThread_Entry, 0U, pointer,
                         TX_APP_STACK_SIZE, TX_APP_THREAD_PRIO,
                         TX_APP_THREAD_PREEMPTION_THRESHOLD,
                         TX_APP_THREAD_TIME_SLICE,
                         TX_APP_THREAD_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}


void MainThread_Entry(ULONG thread_input)
{
    (void)thread_input;

    Motor_Ready = 1U;

    while (1)
    {
        if (tx_semaphore_get(&Motor_Sem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            Motor_Control();
        }
    }
}
