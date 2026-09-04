/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PROT_NONE = 0U,

    PROT_ENCODER = (1UL << 0),
    PROT_ENCODER_FIELD = (1UL << 1),
    PROT_ENCODER_OVERSPEED = (1UL << 2),

    PROT_OVERCURRENT = (1UL << 3),
    PROT_OVERVOLTAGE = (1UL << 4),
    PROT_UNDERVOLTAGE = (1UL << 5),
    PROT_OVERTEMP = (1UL << 6),
    PROT_PHASE_LOSS = (1UL << 7),
    PROT_DRIVER = (1UL << 8),

    PROT_SENSORLESS_START_FAILED = (1UL << 11),

} Protection_Event_e;

typedef struct
{
    uint32_t Report;
    uint32_t Warning;
    uint32_t Stop;

    /*
     * TRIP is reserved for hardware-fast shutdown sources such as
     * comparator/driver fault -> TIM1 Break. The current AxDr_L development
     * board has no such hardware trip path, so normal software protection on
     * this board must use STOP rather than assuming a hardware TRIP exists.
     */
    uint32_t Trip;

} Protection_T;

extern volatile Protection_T Protection;

void Protection_Control(void);
bool Protection_Enable_Allowed(void);
bool Protection_Clear(void);

void Protection_Report_Set(uint32_t Event);
void Protection_Report_Clear(uint32_t Event);
void Protection_Warning_Set(uint32_t Event);
void Protection_Warning_Clear(uint32_t Event);
void Protection_Stop_Set(uint32_t Event);
void Protection_Trip_Set(uint32_t Event);

#endif /* PROTECTION_H */
