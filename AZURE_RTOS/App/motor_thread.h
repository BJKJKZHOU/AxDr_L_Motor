#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "tx_api.h"


typedef enum
{
    SERVO_CMD_ENABLE = 0,
    SERVO_CMD_RUN,
    SERVO_CMD_STOP,
    SERVO_CMD_DISABLE,
    SERVO_CMD_EN_TOGGLE,
    SERVO_CMD_RUN_TOGGLE,
    SERVO_CMD_MODE_SET,
} Servo_Cmd_e;


typedef enum
{
    IDENT_CMD_START = 0,
    IDENT_CMD_ABORT,
    IDENT_CMD_APPLY,
} Ident_Cmd_e;


typedef enum
{
    SENSORLESS_CMD_START = 0,
    SENSORLESS_CMD_STOP,
} Sensorless_Cmd_e;


extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Servo_Cmd_Q;
extern TX_QUEUE Ident_Cmd_Q;
extern TX_QUEUE Sensorless_Cmd_Q;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);


#endif /* MOTOR_THREAD_H */
