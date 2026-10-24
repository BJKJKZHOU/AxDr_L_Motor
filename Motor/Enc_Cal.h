/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENC_CAL_H
#define ENC_CAL_H

#include <stdbool.h>

#include "Fast_Memory.h"
#include "Math.h"
#include "Motor_Type.h"

#define ENC_CAL_WM (1000.0f * TWO_PI_F / 60.0f)

typedef enum
{
    CAL_IDLE = 0,
    CAL_SPEED,
    CAL_VF,
    CAL_SAMPLE,
    CAL_HANDOFF,
    CAL_DECEL,
    CAL_FIT,
    CAL_FIT_WAIT,
    CAL_SAVE_WAIT,
    CAL_DONE,
    CAL_FAILED,

} Enc_Cal_Step_e;

typedef struct
{
    Enc_Cal_Step_e Step;

} Enc_Cal_T;

extern volatile Enc_Cal_T Enc_Cal;
extern volatile bool Enc_Cal_Handoff_Ready;

bool Enc_Cal_Start(void);
void Enc_Cal_Abort(void);
bool Enc_Cal_Fit_Apply(void);
bool Enc_Cal_Save(void);
void Enc_Cal_Run(void);
FAST_CODE Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                            float *Ualpha,
                                            float *Ubeta);

#endif /* ENC_CAL_H */
