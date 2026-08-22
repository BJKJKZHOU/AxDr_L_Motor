#include "motor_thread.h"

#include "Motor_Control.h"
#include "Plot.h"


#define MOTOR_STACK_SIZE    512U
#define MOTOR_THREAD_PRIO   5U
#define MOTOR_CMD_Q_LEN     8U


TX_SEMAPHORE Motor_Sem;
TX_QUEUE Motor_Cmd_Q;
volatile ULONG Motor_Ready = 0U;


static TX_THREAD Motor_Thread;
static uint8_t Normal_Div = 0U;


static void Motor_Entry(ULONG thread_input);


static void Motor_Cmd_Run(void)
{
    Motor_State_e State;
    ULONG Cmd;
    uint8_t Cmd_Id;
    uint8_t Arg;

    while (tx_queue_receive(&Motor_Cmd_Q, &Cmd, TX_NO_WAIT) == TX_SUCCESS)
    {
        Cmd_Id = (uint8_t)Cmd;
        Arg = (uint8_t)(Cmd >> 8);

        switch ((Motor_Cmd_e)Cmd_Id)
        {
            case MOTOR_CMD_ENABLE:
                Motor_Enable();
                break;

            case MOTOR_CMD_RUN:
                Motor_Start();
                break;

            case MOTOR_CMD_STOP:
                Motor_Stop();
                break;

            case MOTOR_CMD_DISABLE:
                Motor_Disable();
                break;

            case MOTOR_CMD_EN_TOGGLE:
                State = Motor_State_Get();

                if (State == DISABLED)
                {
                    Motor_Enable();
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

            case MOTOR_CMD_MODE_SET:
                Motor_Mode_Set((Motor_Mode_e)Arg);
                break;

            case MOTOR_CMD_IDENT_SET:
                Motor_Ident_Mode_Set((Ident_Mode_e)Arg);
                break;

            case MOTOR_CMD_SENSORLESS_DIR_SET:
                Motor_Sensorless_Dir_Set((Arg == 2U) ? -1 : 1);
                break;

            case MOTOR_CMD_IDENT_APPLY:
                (void)Motor_Ident_Apply();
                break;

            default:
                break;
        }
    }
}


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
                         MOTOR_CMD_Q_LEN * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_queue_create(&Motor_Cmd_Q, "Motor Command", TX_1_ULONG, pointer,
                        MOTOR_CMD_Q_LEN * sizeof(ULONG)) != TX_SUCCESS)
    {
        return TX_QUEUE_ERROR;
    }

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         MOTOR_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&Motor_Thread, "Motor Thread", Motor_Entry, 0U, pointer,
                         MOTOR_STACK_SIZE, MOTOR_THREAD_PRIO, MOTOR_THREAD_PRIO,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}


static void Motor_Entry(ULONG thread_input)
{
    (void)thread_input;

    Motor_Ready = 1U;

    while (1)
    {
        if (tx_semaphore_get(&Motor_Sem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            Motor_Cmd_Run();
            Motor_Control();

            Normal_Div ^= 1U;

            if (Normal_Div == 0U)
            {
                Plot_Normal_Sample();
            }
        }
    }
}
