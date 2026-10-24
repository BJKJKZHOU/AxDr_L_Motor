/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#include "Fast_Memory.h"

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
    PROT_ADC = (1UL << 9),

} Protection_Event_e;

typedef struct
{
    uint32_t Report;
    uint32_t Warning;
    uint32_t Fault;

    /* 保留通信中的硬件关断状态；当前板的保护故障记录在 Fault。 */
    uint32_t Trip;

} Protection_T;

extern volatile Protection_T Protection;

void Protection_Control(void);
bool Protection_Enable_Allowed(void);
bool Protection_Clear(void);
FAST_CODE bool Protection_Current_Fast(float Ia_A, float Ib_A, float Ic_A);

float Protection_Vbus_Min_Get(void);
float Protection_Vbus_Max_Get(void);

void Protection_Fault_Set(uint32_t Event);

#endif /* PROTECTION_H */
