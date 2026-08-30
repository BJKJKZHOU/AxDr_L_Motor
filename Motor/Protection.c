/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protection.h"

#include "Encoder.h"
#include "Motor_Control.h"
#include "Motor_PWM.h"

volatile Protection_T Protection = { 0 };

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

void Protection_Stop_Set(uint32_t Event)
{
    Protection.Stop |= Event;
}

/*
 * AxDr_L development board note:
 * there is currently no comparator/driver-fault -> TIM1 BKIN hardware path.
 * This entry is reserved for a future board or other hardware that can assert
 * a real fast trip source. Software-detected faults on the current board must
 * normally be raised through Protection_Stop_Set().
 */
void Protection_Trip_Set(uint32_t Event)
{
    Protection.Trip |= Event;
    PWM_Disable();
}

bool Protection_Enable_Allowed(void)
{
    return (Protection.Stop == 0U) && (Protection.Trip == 0U);
}

bool Protection_Clear(void)
{
    if (Motor_State_Get() != DISABLED)
    {
        return false;
    }

    Protection.Report = 0U;
    Protection.Warning = 0U;
    Protection.Stop = 0U;
    Protection.Trip = 0U;

    return true;
}

void Protection_Control(void)
{
    if (Encoder.Fault != 0U)
    {
        Protection_Stop_Set(PROT_ENCODER);
    }

    if ((Protection.Stop != 0U) || (Protection.Trip != 0U))
    {
        if (Motor_State_Get() != DISABLED)
        {
            Motor_Disable();
        }
    }
}
