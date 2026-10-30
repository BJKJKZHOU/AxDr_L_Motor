/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Fast_Memory.h"

#include "Motor_Type.h"
#include "Identification.h"
#include "Enc_Control.h"

void Motor_Control(void);
FAST_CODE Motor_Fast_Mode_e Motor_Fast_Run(float Encoder_Theta_e,
                                          bool Encoder_Park_Valid,
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha,
                                          float *Ubeta);

FAST_CODE Motor_State_e Motor_State_Get(void);
FAST_CODE float Motor_I_Limit_Effective_Get(void);
float Motor_Wm_Limit_Effective_Get(void);
float Motor_Wm_Ref_Get(void);
Motor_Position_T Motor_Position_Ref_Get(void);
bool Motor_Encoder_Required(void);

/* Existing speed/current PI handover used by encoder calibration. */
void Motor_Cal_Decel_Begin(void);

/* 动作执行结果，由命令入口转换为通信应答。 */
typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERR_STATE,
    MOTOR_ERR_CONFIG,

} Motor_Status_e;

Motor_Status_e Motor_Enable(void);
Motor_Status_e Motor_Start(void);
Motor_Status_e Motor_Enc_Start(Enc_Mode_e Mode);
void Motor_Stop(void);
void Motor_Disable(void);

/* Internal output/state transition; keep the encoder sequence active.
 * User disable and protection must use Motor_Disable() to cancel it. */
void Motor_Disable_Apply(void);

Motor_Status_e Motor_Ident_Start(Ident_Mode_e Mode);
Motor_Status_e Motor_Ident_Apply(void);
Motor_Status_e Motor_Signal_Start(void);

/* User mechanical coordinate feedback. Internal FOC state remains in Motor_Run. */
float Motor_Wm_Get(void);
void Motor_Position_Get(int32_t *Turn, float *Theta);

/* Coherent 2 kHz active speed-controller sample, held for the 20 kHz FAST plot. */
typedef struct
{
    float Ref;
    float Fbk;
    float Iq;
    float Inj;
} Motor_Frf_Speed_T;

extern volatile Motor_Frf_Speed_T Motor_Frf_Speed;

/* NORMAL Plot shadows in user mechanical coordinates. */
extern volatile float Motor_Plot_Wm;
extern volatile float Motor_Plot_Wm_Ref;
extern volatile float Motor_Plot_Position;
extern volatile float Motor_Plot_Position_Ref;
void Motor_Plot_Normal_Update(void);

#endif /* MOTOR_CONTROL_H */
