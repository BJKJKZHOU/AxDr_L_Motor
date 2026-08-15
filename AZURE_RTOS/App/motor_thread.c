#include "motor_thread.h"

#include "Motor_Control.h"


#define MOTOR_STACK_SIZE    512U
#define MOTOR_THREAD_PRIO   5U
#define SERVO_CMD_Q_LEN     4U


TX_SEMAPHORE Motor_Sem;
TX_QUEUE Servo_Cmd_Q;
volatile ULONG Motor_Ready = 0U;


static TX_THREAD Motor_Thread;


static void Motor_Entry(ULONG thread_input);


static void Servo_Cmd_Run(void)
{
    Servo_State_e State;
    ULONG Cmd;

    while (tx_queue_receive(&Servo_Cmd_Q, &Cmd, TX_NO_WAIT) == TX_SUCCESS)
    {
        switch ((Servo_Cmd_e)Cmd)
        {
            case SERVO_CMD_ENABLE:
                Servo_Enable();
                break;

            case SERVO_CMD_RUN:
                Servo_Run();
                break;

            case SERVO_CMD_STOP:
                Servo_Stop();
                break;

            case SERVO_CMD_DISABLE:
                Servo_Disable();
                break;

            case SERVO_CMD_EN_TOGGLE:
                State = Servo_State_Get();

                if (State == SERVO_DISABLED)
                {
                    Servo_Enable();
                }
                else
                {
                    Servo_Disable();
                }
                break;

            case SERVO_CMD_RUN_TOGGLE:
                State = Servo_State_Get();

                if (State == SERVO_ENABLED)
                {
                    Servo_Run();
                }
                else if (State == SERVO_RUN)
                {
                    Servo_Stop();
                }
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
                         SERVO_CMD_Q_LEN * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_queue_create(&Servo_Cmd_Q, "Servo Command", TX_1_ULONG, pointer,
                        SERVO_CMD_Q_LEN * sizeof(ULONG)) != TX_SUCCESS)
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
            Servo_Cmd_Run();
            Motor_Control();
        }
    }
}
