/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENC_CAL_H
#define ENC_CAL_H

#include <stdbool.h>

#include "Fast_Memory.h"
#include "Motor_Type.h"

/* One automatic encoder harmonic calibration; normal user modes are unchanged. */
bool Enc_Cal_Start(void);
void Enc_Cal_Run(void);
bool Enc_Cal_Active(void);
bool Enc_Cal_Succeeded(void);

/* Used only by the existing 2 kHz speed path and 20 kHz motor fast path. */
bool Enc_Cal_Sensored(void);
float Enc_Cal_Wm_Target(void);
FAST_CODE Motor_Fast_Mode_e Enc_Cal_Fast_Run(float *Theta_e,
                                            float *Ualpha,
                                            float *Ubeta);
FAST_CODE void Enc_Cal_Sample(float Theta_Native);

#endif /* ENC_CAL_H */
