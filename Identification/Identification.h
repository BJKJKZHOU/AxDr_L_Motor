/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#include "Motor_Type.h"

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

bool Identification_Start(Ident_Mode_e Mode);
void Identification_Abort(void);
void Identification_Control(void);
bool Identification_Apply(void);

bool Identification_Active(void);
Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float Ic_A,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V);

Ident_Mode_e Identification_Mode_Get(void);
Ident_State_e Identification_State_Get(void);
uint8_t Identification_Stage_Get(void);

#endif /* IDENTIFICATION_H */
