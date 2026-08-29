/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef VOLTAGE_DIAG_H
#define VOLTAGE_DIAG_H

#include <stdbool.h>

bool Voltage_Diag_Config(float We_Target, float U_Target, float U_Align);
bool Voltage_Diag_Enable(bool Enable);
bool Voltage_Diag_Enabled(void);
void Voltage_Diag_Begin(void);
void Voltage_Diag_Stop(void);
bool Voltage_Diag_Run(float Ia, float Ib, float Ic, float I_Max,
                      float *Theta, float *Ualpha, float *Ubeta);
void Voltage_Diag_Get(float *We_Target, float *U_Target, float *U_Align,
                      float *We_Ref, float *U_Ref);

#endif /* VOLTAGE_DIAG_H */
