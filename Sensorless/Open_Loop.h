/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef OPEN_LOOP_H
#define OPEN_LOOP_H

#include <stdint.h>
#include "Motor_Type.h"

typedef enum
{
    OPEN_LOOP_IF = 0,
    OPEN_LOOP_VF,

} Open_Loop_Method_e;

extern uint8_t Open_Loop_Method;

void Open_Loop_Reset(void);
Motor_Fast_Mode_e Open_Loop(float We_Ref, float *Theta_e,
                           float *Id_Ref, float *Iq_Ref,
                           float *Ualpha, float *Ubeta);

#endif /* OPEN_LOOP_H */
