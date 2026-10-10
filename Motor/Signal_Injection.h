/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SIGNAL_INJECTION_H
#define SIGNAL_INJECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "Fast_Memory.h"

typedef enum
{
    SIGNAL_ID = 0,
    SIGNAL_IQ,
    SIGNAL_SPEED,
} Signal_Target_e;

typedef struct
{
    struct
    {
        float Freq_Hz;
        float Amp_A;
        float Time_S;
        float Speed_Amp;
        uint8_t Target;
    } Para;

    struct
    {
        float Phase;
        float Phase_Step;
        float Amp;
        float Out;
        uint32_t Sample_Cnt;
        uint32_t Sample_Max;
        uint8_t Active;
        uint8_t Target;
    } State;

} Signal_Injection_T;

extern volatile Signal_Injection_T Signal_Injection;

/* One finite sine experiment on either current axis or the sensored speed loop. */
bool Signal_Injection_Start(float I_Max, float Wm_Max);
void Signal_Injection_Stop(void);
FAST_CODE float Signal_Injection_Run(void);

#endif /* SIGNAL_INJECTION_H */
