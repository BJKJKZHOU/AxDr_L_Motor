/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#include "Fast_Memory.h"

#include <stdint.h>

typedef struct
{
    uint16_t Ia_Raw;
    uint16_t Ib_Raw;
    uint16_t Ic_Raw;

    uint16_t Ia_Off;
    uint16_t Ib_Off;
    uint16_t Ic_Off;

    float Ia_A;
    float Ib_A;
    float Ic_A;

    uint16_t Vbus_Raw;
    float Vbus_V;

} ADC_T;

extern volatile ADC_T ADC;
extern volatile float Motor_Plot_Iq_Ref;

void ADC_Calib(void);
FAST_CODE void Fast_Loop(void);
float Motor_Iq_Ref_Get(void);

#endif /* MOTOR_ADC_H */
