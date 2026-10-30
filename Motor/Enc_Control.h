/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENC_CONTROL_H
#define ENC_CONTROL_H

#include <stdbool.h>

#include "Fast_Memory.h"
#include "Motor_Type.h"

typedef enum
{
    ENC_PHASE = 1,
    ENC_CAL,

} Enc_Mode_e;

typedef enum
{
    ENC_IDLE = 0,
    ENC_CAL_RUN,
    ENC_PHASE_RUN,
    ENC_PHASE_SAVE,
    ENC_STOP_WAIT,
    ENC_DONE,
    ENC_FAILED,

} Enc_Step_e;

/* 寻相和校准期间持续采集编码器反馈。 */
extern volatile Enc_Step_e Enc_Step;

bool Enc_Control_Start(Enc_Mode_e Mode);
void Enc_Control_Abort(void);
void Enc_Control_Run(void);
FAST_CODE Motor_Fast_Mode_e Enc_Control_Fast_Run(float *Theta_e,
                                                 float *Id_Ref,
                                                 float *Iq_Ref,
                                                 float *Ualpha,
                                                 float *Ubeta);

static inline bool Enc_Control_Active(void)
{
    return (Enc_Step > ENC_IDLE) && (Enc_Step < ENC_DONE);
}

#endif /* ENC_CONTROL_H */
