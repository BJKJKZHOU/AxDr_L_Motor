/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Motor_Type.h"
#include "Identification.h"

void Motor_Control(void);
Motor_Fast_Mode_e Motor_Fast_Run(float *Theta_e,
                                 float *Id_Ref,
                                 float *Iq_Ref,
                                 float *Ualpha,
                                 float *Ubeta);

Motor_State_e Motor_State_Get(void);
Motor_Mode_e Motor_Mode_Get(void);
float Motor_I_Limit_Effective_Get(void);
float Motor_Wm_Limit_Effective_Get(void);
float Motor_Wm_Ref_Get(void);
Motor_Position_T Motor_Position_Ref_Get(void);
bool Motor_Encoder_Required(void);

void Motor_Enable(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_Disable(void);

bool Motor_Ident_Start(Ident_Mode_e Mode);
bool Motor_Ident_Abort(void);
bool Motor_Ident_Apply(void);

/* User mechanical coordinate feedback. Internal FOC state remains in Motor_Run. */
float Motor_Wm_Get(void);
void Motor_Position_Get(int32_t *Turn, float *Theta);

#endif /* MOTOR_CONTROL_H */
