#include "motor_thread.h"

#include "Identification.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "Plot.h"
#include "Start.h"


#define MOTOR_STACK_SIZE        512U
#define MOTOR_THREAD_PRIO       5U
#define SERVO_CMD_Q_LEN         4U
#define IDENT_CMD_Q_LEN         4U
#define SENSORLESS_CMD_Q_LEN    4U


TX_SEMAPHORE Motor_Sem;
TX_QUEUE Servo_Cmd_Q;
TX_QUEUE Ident_Cmd_Q;
TX_QUEUE Sensorless_Cmd_Q;
volatile ULONG Motor_Ready = 0U;


static TX_THREAD Motor_Thread;
static uint8_t Normal_Div = 0U;


static void Motor_Entry(ULONG thread_input);


static void Ident_Cmd_Run(void)
{
    ULONG Cmd;
    uint8_t Cmd_Id;
    uint8_t Arg;

    while (tx_queue_receive(&Ident_Cmd_Q, &Cmd, TX_NO_WAIT) == TX_SUCCESS)
    {
        Cmd_Id = (uint8_t)Cmd;
        Arg = (uint8_t)(Cmd >> 8);

        if (Cmd_Id == (uint8_t)IDENT_CMD_START)
        {
            if ((Servo_State_Get() == SERVO_DISABLED) &&
                !Identification_Active() &&
                !Sensorless_Start_Active() &&
                Identification_Start((Ident_Mode_e)Arg))
            {
                PWM_Enable();
            }
        }
        else if (Cmd_Id == (uint8_t)IDENT_CMD_ABORT)
        {
            if (Identification_Active())
            {
                PWM_Disable();
                Identification_Abort();
            }
        }
        else if (Cmd_Id == (uint8_t)IDENT_CMD_APPLY)
        {
            if ((Servo_State_Get() == SERVO_DISABLED) &&
                !Identification_Active() &&
                !Sensorless_Start_Active())
            {
                (void)Identification_Apply();
            }
        }
    }
}


static void Sensorless_Cmd_Run(void)
{
    ULONG Cmd;
    uint8_t Cmd_Id;
    uint8_t Arg;
    int8_t Dir;

    while (tx_queue_receive(&Sensorless_Cmd_Q, &Cmd, TX_NO_WAIT) == TX_SUCCESS)
    {
        Cmd_Id = (uint8_t)Cmd;
        Arg = (uint8_t)(Cmd >> 8);

        if (Cmd_Id == (uint8_t)SENSORLESS_CMD_START)
        {
            if ((Servo_State_Get() == SERVO_DISABLED) &&
                !Identification_Active() &&
                !Sensorless_Start_Active())
            {
                Dir = (Arg == 2U) ? -1 : 1;
                Sensorless_Start_Begin(Dir);
                PWM_Enable();
            }
        }
        else if (Cmd_Id == (uint8_t)SENSORLESS_CMD_STOP)
        {
            if (Sensorless_Start_Active())
            {
                PWM_Disable();
                Sensorless_Start_Stop();
            }
        }
    }
}


static void Servo_Cmd_Run(void)
{
    Servo_State_e State;
    ULONG Cmd;
    uint8_t Cmd_Id;
    uint8_t Arg;

    while (tx_queue_receive(&Servo_Cmd_Q, &Cmd, TX_NO_WAIT) == TX_SUCCESS)
    {
        Cmd_Id = (uint8_t)Cmd;
        Arg = (uint8_t)(Cmd >> 8);

        if (Identification_Active())
        {
            if (Cmd_Id == (uint8_t)SERVO_CMD_DISABLE)
            {
                Servo_Disable();
                Identification_Abort();
            }

            continue;
        }

        if (Sensorless_Start_Active())
        {
            if (Cmd_Id == (uint8_t)SERVO_CMD_DISABLE)
            {
                Servo_Disable();
                Sensorless_Start_Stop();
            }

            continue;
        }

        switch ((Servo_Cmd_e)Cmd_Id)
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

            case SERVO_CMD_MODE_SET:
                Ctrl_Mode_Set((Ctrl_Mode_e)Arg);
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
                         IDENT_CMD_Q_LEN * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_queue_create(&Ident_Cmd_Q, "Identification Command", TX_1_ULONG, pointer,
                        IDENT_CMD_Q_LEN * sizeof(ULONG)) != TX_SUCCESS)
    {
        return TX_QUEUE_ERROR;
    }

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         SENSORLESS_CMD_Q_LEN * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_queue_create(&Sensorless_Cmd_Q, "Sensorless Command", TX_1_ULONG, pointer,
                        SENSORLESS_CMD_Q_LEN * sizeof(ULONG)) != TX_SUCCESS)
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
    bool Was_Active;

    (void)thread_input;

    Motor_Ready = 1U;

    while (1)
    {
        if (tx_semaphore_get(&Motor_Sem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            Ident_Cmd_Run();
            Sensorless_Cmd_Run();
            Servo_Cmd_Run();

            Was_Active = Identification_Active();

            if (Was_Active)
            {
                Identification_Update();

                if (!Identification_Active())
                {
                    PWM_Disable();
                }
            }
            else if (!Sensorless_Start_Active())
            {
                Motor_Control();
            }

            Normal_Div ^= 1U;

            if (Normal_Div == 0U)
            {
                Plot_Normal_Sample();
            }
        }
    }
}
