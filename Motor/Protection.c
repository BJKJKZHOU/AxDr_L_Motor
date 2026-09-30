/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protection.h"

#include "Encoder.h"
#include "Motor_ADC.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"
#include "motor_params.h"

volatile Protection_T Protection = { 0 };

static uint32_t Vbus_Under_Count = 0U;
static uint32_t Vbus_Over_Count = 0U;

void Protection_Report_Set(uint32_t Event)
{
    Protection.Report |= Event;
}

void Protection_Report_Clear(uint32_t Event)
{
    Protection.Report &= ~Event;
}

void Protection_Warning_Set(uint32_t Event)
{
    Protection.Warning |= Event;
}

void Protection_Warning_Clear(uint32_t Event)
{
    Protection.Warning &= ~Event;
}

void Protection_Fault_Set(uint32_t Event)
{
    Protection.Fault |= Event;
}

/*
 * AxDr_L development board note:
 * there is currently no comparator/driver-fault -> TIM1 BKIN hardware path.
 * This entry is reserved for a future board or other hardware that can assert
 * a real fast trip source. Software-detected faults on the current board must
 * normally be raised through Protection_Fault_Set().
 */
void Protection_Trip_Set(uint32_t Event)
{
    Protection.Trip |= Event;
    PWM_Disable();
}

float Protection_Vbus_Min_Get(void)
{
    return VBUS_UV_FAULT_V;
}

float Protection_Vbus_Max_Get(void)
{
    return VBUS_OV_FAULT_V;
}

bool Protection_Enable_Allowed(void)
{
    if ((Protection.Fault != 0U) || (Protection.Trip != 0U))
    {
        return false;
    }

    if (!__builtin_isfinite(ADC.Vbus_V) ||
        (ADC.Vbus_V < VBUS_UV_FAULT_V) ||
        (ADC.Vbus_V > VBUS_OV_FAULT_V))
    {
        return false;
    }

    return true;
}

bool Protection_Clear(void)
{
    if (Motor_State_Get() != DISABLED)
    {
        return false;
    }

    Protection.Report = 0U;
    Protection.Warning = 0U;
    Protection.Fault = 0U;
    Protection.Trip = 0U;
    Vbus_Under_Count = 0U;
    Vbus_Over_Count = 0U;

    return true;
}

void Protection_Control(void)
{
    Motor_State_e State;

    State = Motor_State_Get();

    if (State == DISABLED)
    {
        Vbus_Under_Count = 0U;
        Vbus_Over_Count = 0U;
    }
    else if (!__builtin_isfinite(ADC.Vbus_V))
    {
        Protection_Fault_Set(PROT_UNDERVOLTAGE);
    }
    else
    {
        if (ADC.Vbus_V < VBUS_UV_FAULT_V)
        {
            if (Vbus_Under_Count < VBUS_UV_DEBOUNCE_TICKS)
            {
                Vbus_Under_Count++;
            }
        }
        else
        {
            Vbus_Under_Count = 0U;
        }

        if (ADC.Vbus_V > VBUS_OV_FAULT_V)
        {
            if (Vbus_Over_Count < VBUS_OV_DEBOUNCE_TICKS)
            {
                Vbus_Over_Count++;
            }
        }
        else
        {
            Vbus_Over_Count = 0U;
        }

        if (Vbus_Under_Count >= VBUS_UV_DEBOUNCE_TICKS)
        {
            Protection_Fault_Set(PROT_UNDERVOLTAGE);
        }

        if (Vbus_Over_Count >= VBUS_OV_DEBOUNCE_TICKS)
        {
            Protection_Fault_Set(PROT_OVERVOLTAGE);
        }
    }

    if (Motor_Encoder_Required() && (Encoder.Fault != 0U))
    {
        Protection_Fault_Set(PROT_ENCODER);
    }

    if ((Protection.Fault != 0U) || (Protection.Trip != 0U))
    {
        if (State != DISABLED)
        {
            Motor_Disable();
        }
    }
}
