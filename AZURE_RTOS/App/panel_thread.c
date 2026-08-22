#include "panel_thread.h"

#include "main.h"
#include "Motor_Control.h"
#include "RGB.h"
#include "motor_thread.h"


#define PANEL_STACK_SIZE    512U
#define PANEL_THREAD_PRIO   10U
#define KEY_SCAN_TICK       5U
#define KEY_STABLE_CNT      3U


typedef struct
{
    GPIO_PinState Sample;
    GPIO_PinState State;
    uint8_t Cnt;

} Key_T;


static TX_THREAD Panel_Thread;


static uint32_t Key_Press(Key_T *Key, GPIO_TypeDef *Port, uint16_t Pin)
{
    GPIO_PinState Sample;

    Sample = HAL_GPIO_ReadPin(Port, Pin);

    if (Sample != Key->Sample)
    {
        Key->Sample = Sample;
        Key->Cnt = 1U;
    }
    else if (Key->Cnt < KEY_STABLE_CNT)
    {
        Key->Cnt++;
    }

    if ((Key->Cnt == KEY_STABLE_CNT) && (Sample != Key->State))
    {
        Key->State = Sample;
        return (Sample == GPIO_PIN_RESET) ? 1U : 0U;
    }

    return 0U;
}


static void Panel_LED(Motor_State_e State)
{
    switch (State)
    {
        case DISABLED:
            RGB_Set(0U, 0U, 8U);
            break;

        case ENABLED:
            RGB_Set(8U, 8U, 0U);
            break;

        case RUN:
            RGB_Set(0U, 8U, 0U);
            break;

        default:
            RGB_Set(8U, 0U, 0U);
            break;
    }
}


static void Panel_Entry(ULONG thread_input)
{
    Key_T Key1 = {GPIO_PIN_SET, GPIO_PIN_SET, 0U};
    Key_T Key2 = {GPIO_PIN_SET, GPIO_PIN_SET, 0U};
    Motor_State_e State;
    Motor_State_e State_Pre;
    ULONG Cmd;
    uint32_t Key1_Press;
    uint32_t Key2_Press;

    (void)thread_input;

    State_Pre = Motor_State_Get();
    Panel_LED(State_Pre);

    while (1)
    {
        Key1_Press = Key_Press(&Key1, KEY1_GPIO_Port, KEY1_Pin);
        Key2_Press = Key_Press(&Key2, KEY2_GPIO_Port, KEY2_Pin);

        if (Key1_Press != 0U)
        {
            Cmd = (ULONG)MOTOR_CMD_EN_TOGGLE;
            (void)tx_queue_send(&Motor_Cmd_Q, &Cmd, TX_NO_WAIT);
        }
        else if (Key2_Press != 0U)
        {
            Cmd = (ULONG)MOTOR_CMD_RUN_TOGGLE;
            (void)tx_queue_send(&Motor_Cmd_Q, &Cmd, TX_NO_WAIT);
        }

        State = Motor_State_Get();

        if (State != State_Pre)
        {
            Panel_LED(State);
            State_Pre = State;
        }

        tx_thread_sleep(KEY_SCAN_TICK);
    }
}


UINT Panel_Thread_Init(VOID *memory_ptr)
{
    TX_BYTE_POOL *byte_pool;
    CHAR *pointer;

    byte_pool = (TX_BYTE_POOL *)memory_ptr;

    if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                         PANEL_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&Panel_Thread, "Panel Thread", Panel_Entry, 0U, pointer,
                         PANEL_STACK_SIZE, PANEL_THREAD_PRIO, PANEL_THREAD_PRIO,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}
