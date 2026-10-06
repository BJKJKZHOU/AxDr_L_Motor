/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include "Fast_Memory.h"

/* Duty inputs use the normalized range [0.0f, 1.0f]. */
void PWM_Timing_Update(void);
FAST_CODE void PWM_Update(float DutyA, float DutyB, float DutyC);
void PWM_Enable(void);
FAST_CODE void PWM_Disable(void);

#endif /* MOTOR_PWM_H */
