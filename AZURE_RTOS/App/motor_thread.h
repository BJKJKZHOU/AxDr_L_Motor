#ifndef MOTOR_THREAD_H
#define MOTOR_THREAD_H

#include "tx_api.h"


#define TX_APP_STACK_SIZE                       512
#define TX_APP_THREAD_PRIO                      5

#ifndef TX_APP_THREAD_PREEMPTION_THRESHOLD
#define TX_APP_THREAD_PREEMPTION_THRESHOLD      TX_APP_THREAD_PRIO
#endif

#ifndef TX_APP_THREAD_TIME_SLICE
#define TX_APP_THREAD_TIME_SLICE                TX_NO_TIME_SLICE
#endif

#ifndef TX_APP_THREAD_AUTO_START
#define TX_APP_THREAD_AUTO_START                TX_AUTO_START
#endif


extern TX_THREAD tx_app_thread;
extern TX_SEMAPHORE Motor_Sem;
extern volatile ULONG Motor_Ready;

UINT Motor_Thread_Init(VOID *memory_ptr);
void MainThread_Entry(ULONG thread_input);


#endif /* MOTOR_THREAD_H */
