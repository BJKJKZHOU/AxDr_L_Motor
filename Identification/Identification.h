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
    IDENT_JB,

} Ident_Mode_e;

typedef enum
{
    IDENT_IDLE = 0,
    IDENT_RUNNING,
    IDENT_DONE,
    IDENT_FAILED,
    IDENT_FINISH,

} Ident_State_e;

typedef enum
{
    IDENT_FAIL_NONE = 0,
    IDENT_FAIL_PHASE_CURRENT, /* Retired; retain the Host-visible reason values. */
    IDENT_FAIL_FLUX_INTERNAL,
    IDENT_FAIL_START_CONFIG,
    IDENT_FAIL_JB_INTERNAL,

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
 * (Flux and J/B identification). These instances are owned by the
 * Identification layer and are separate from normal Sensorless-mode runtime.
 * Each identification workflow owns its own configuration/reset policy. */
extern Flux_Observer_T Ident_Observer;
extern PLL_T Ident_PLL;

/* Host-configured J/B experiment settings. The work-point speed remains an
 * internal automatic policy derived from the validated Flux 0.30 ratio. */
extern volatile float Ident_JB_Excite_Ratio;
extern volatile float Ident_JB_Excite_Hz;
extern volatile uint8_t Ident_Fail_Reason;
extern volatile Ident_State_e Ident_State; /* Updated by Identification only. */
/* Limits written by Identification; identification algorithms only read them. */
extern Ident_Envelope_T Ident_Envelope;

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target);
void Identification_Stop(void);
void Identification_Report(bool Valid);
void Identification_Control(void);
bool Identification_Apply(void);

Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V);

#endif /* IDENTIFICATION_H */
