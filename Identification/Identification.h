/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#include "Flux_Observer.h"
#include "Motor_Type.h"
#include "PLL.h"

typedef enum
{
    IDENT_NONE = 0,
    IDENT_RS_LS,
    IDENT_FLUX,

} Ident_Mode_e;

typedef enum
{
    IDENT_IDLE = 0,
    IDENT_RUNNING,
    IDENT_DONE,
    IDENT_FAILED,

} Ident_State_e;

typedef enum
{
    IDENT_FAIL_NONE = 0,
    IDENT_FAIL_PHASE_CURRENT,
    IDENT_FAIL_FLUX_INTERNAL,
    IDENT_FAIL_START_CONFIG,

} Ident_Fail_Reason_e;

typedef struct
{
    float I_Max;
    float U_Available;
    float U_Max;

} Ident_Envelope_T;

/* Identification-only sensorless runtime.
 *
 * Shared by identification workflows that require sensorless motion
 * (Flux and future J/B identification). These instances are owned by the
 * Identification layer and are separate from normal Sensorless-mode runtime.
 * Each identification workflow owns its own configuration/reset policy. */
extern Flux_Observer_T Ident_Observer;
extern PLL_T Ident_PLL;

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target);
void Identification_Abort(void);
void Identification_Control(void);
bool Identification_Apply(void);

bool Identification_Active(void);
bool Identification_Result_Valid(void);
Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float Ic_A,
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V);

/* Internal lifecycle accessors. These are not Host-visible protocol state. */
Ident_Mode_e Identification_Mode_Get(void);
Ident_State_e Identification_State_Get(void);
const Ident_Envelope_T *Identification_Envelope_Get(void);

/* Stable final-result accessors used by the Parameter layer. */
uint8_t Identification_Rs_Ls_Valid_Get(void);
float Identification_Rs_Get(void);
float Identification_Ls_Get(void);
uint8_t Identification_Flux_Valid_Get(void);
float Identification_Flux_Get(void);
uint8_t Identification_Fail_Reason_Get(void);

#endif /* IDENTIFICATION_H */
