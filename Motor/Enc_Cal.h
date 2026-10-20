/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENC_CAL_H
#define ENC_CAL_H

#include <stdbool.h>

#include "Fast_Memory.h"
#include "Motor_Type.h"

typedef enum
{
    CAL_IDLE = 0,
    CAL_SPEED,
    CAL_VF,
    CAL_SAMPLE,
    CAL_DECEL,
    CAL_PHASE,
    CAL_DONE,
    CAL_FAILED,

} Enc_Cal_Step_e;

typedef struct
{
    Enc_Cal_Step_e Step;
    float Wm_Target;

} Enc_Cal_T;

extern volatile Enc_Cal_T Enc_Cal;

bool Enc_Cal_Start(void);
void Enc_Cal_Run(void);
FAST_CODE Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                            float *Id_Ref,
                                            float *Iq_Ref,
                                            float *Ualpha,
                                            float *Ubeta);

#endif /* ENC_CAL_H */
