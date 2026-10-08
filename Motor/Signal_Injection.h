/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef SIGNAL_INJECTION_H
#define SIGNAL_INJECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "Fast_Memory.h"

typedef struct
{
    struct
    {
        float Freq_Hz;
        float Amp_A;
        float Time_S;
    } Para;

    struct
    {
        float Phase;
        float Phase_Step;
        float Amp_A;
        float Out;
        uint32_t Sample_Cnt;
        uint32_t Sample_Max;
        uint8_t Active;
    } State;

} Signal_Injection_T;

extern volatile Signal_Injection_T Signal_Injection;

/* One finite d-axis sine test; configured and started from the Motor thread. */
bool Signal_Injection_Start(float I_Max);
void Signal_Injection_Stop(void);
FAST_CODE float Signal_Injection_Run(void);

#endif /* SIGNAL_INJECTION_H */
