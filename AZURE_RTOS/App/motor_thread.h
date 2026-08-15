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
} Servo_Cmd_e;


extern TX_SEMAPHORE Motor_Sem;
extern TX_QUEUE Servo_Cmd_Q;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);


#endif /* MOTOR_THREAD_H */
