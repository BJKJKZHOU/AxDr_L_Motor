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

/* Normal encoder feedback remains live throughout these operations. */
extern volatile Enc_Step_e Enc_Step;

bool Enc_Control_Start(Enc_Mode_e Mode);
void Enc_Control_Abort(void);
void Enc_Control_Run(void);
static inline bool Enc_Control_Active(void)
{
    return (Enc_Step > ENC_IDLE) && (Enc_Step < ENC_DONE);
}

#endif /* ENC_CONTROL_H */
