/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protection.h"

#include "Board_Config.h"
#include "Encoder.h"
#include "Motor_ADC.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "main.h"

volatile Protection_T Protection = { 0 };
const float Protection_Vbus_Min = VBUS_UV_WARNING_V;
const float Protection_Vbus_Max = VBUS_OV_FAULT_V;

static uint32_t Current_Overload_Count = 0U;
static uint32_t Current_Fast_Count = 0U;

void Protection_Fault_Set(uint32_t Event)
{
    uint32_t Primask = __get_PRIMASK();

    /* 故障位置位不可被其他更新打断。 */
    __disable_irq();
    Protection.Fault |= Event;
    __set_PRIMASK(Primask);
}

void Protection_Fast_Run(void)
{
    float Current_Sq;
    uint32_t Event = 0U;

    if (ADC.Vbus_V < VBUS_UV_WARNING_V)
    {
        Protection.Warning |= PROT_UNDERVOLTAGE;
    }
    else
    {
        Protection.Warning &= ~PROT_UNDERVOLTAGE;
    }

    if (Protection.Fault != 0U)
    {
        return;
    }

    if (ADC.Vbus_V > VBUS_OV_FAULT_V)
    {
        Event |= PROT_OVERVOLTAGE;
    }

    if (Motor_Encoder_Required() && (Encoder.Fault != 0U))
    {
        Event |= PROT_ENCODER;
    }

    /* 等幅三相电流矢量的幅值平方，不依赖控制角度，覆盖各运行模式。 */
    Current_Sq = 0.666666667f *
                 (ADC.Ia_A * ADC.Ia_A + ADC.Ib_A * ADC.Ib_A + ADC.Ic_A * ADC.Ic_A);

    if (Current_Sq > (PROT_CURRENT_OVERLOAD_A * PROT_CURRENT_OVERLOAD_A))
    {
        Current_Overload_Count++;
    }
    else
    {
        Current_Overload_Count = 0U;
    }

    if (Current_Sq > (PROT_CURRENT_FAST_A * PROT_CURRENT_FAST_A))
    {
        Current_Fast_Count++;
    }
    else
    {
        Current_Fast_Count = 0U;
    }

    if ((Current_Overload_Count >= PROT_CURRENT_OVERLOAD_CYCLES) ||
        (Current_Fast_Count >= PROT_CURRENT_FAST_CYCLES))
    {
        Event |= PROT_OVERCURRENT;
    }

    if (Event != 0U)
    {
        PWM_Disable();
        Protection_Fault_Set(Event);
    }
}

bool Protection_Clear(void)
{
    if (Motor_State_Get() != DISABLED)
    {
        return false;
    }

    Protection.Report = 0U;
    Protection.Fault = 0U;
    Protection.Trip = 0U;
    Current_Overload_Count = 0U;
    Current_Fast_Count = 0U;

    return true;
}
