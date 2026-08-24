/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Identification.h"

#include "Current_Loop.h"
#include "Flux.h"
#include "Rs_Ls.h"

#define IDENT_I_MAX_A 2.2f

static volatile Ident_Mode_e Ident_Mode = IDENT_NONE;
static volatile Ident_State_e Ident_State = IDENT_IDLE;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target)
{
    if (Ident_State == IDENT_RUNNING)
    {
        return false;
    }

    if (Mode == IDENT_RS_LS)
    {
        Rs_Ls_Start();
    }
    else if (Mode == IDENT_FLUX)
    {
        Flux_Start(Wm_Target);
    }
    else
    {
        return false;
    }

    Ident_Mode = Mode;
    Ident_State = IDENT_RUNNING;

    return true;
}

void Identification_Abort(void)
{
    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Reset();
    }
    else if (Ident_Mode == IDENT_FLUX)
    {
        Flux_Reset();
    }

    Ident_Mode = IDENT_NONE;
    Ident_State = IDENT_IDLE;
}

void Identification_Control(void)
{
    const Rs_Ls_Result_T *Rs_Ls_Result;
    const Flux_Result_T *Flux_Result;

    if (Ident_State != IDENT_RUNNING)
    {
        return;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        if (Rs_Ls_Active())
        {
            return;
        }

        Rs_Ls_Result = Rs_Ls_Result_Get();
        Ident_State = Rs_Ls_Result->Valid ? IDENT_DONE : IDENT_FAILED;
    }
    else if (Ident_Mode == IDENT_FLUX)
    {
        Flux_Control();

        if (Flux_Active())
        {
            return;
        }

        Flux_Result = Flux_Result_Get();
        Ident_State = Flux_Result->Valid ? IDENT_DONE : IDENT_FAILED;
    }
}

bool Identification_Apply(void)
{
    const Rs_Ls_Result_T *Rs_Ls_Result;
    const Flux_Result_T *Flux_Result;

    if (Ident_State != IDENT_DONE)
    {
        return false;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Result = Rs_Ls_Result_Get();

        if (!Rs_Ls_Result->Valid)
        {
            return false;
        }

        Motor_Para.Rs = Rs_Ls_Result->Rs_Ohm;
        Motor_Para.Ld = Rs_Ls_Result->Ls_H;
        Motor_Para.Lq = Rs_Ls_Result->Ls_H;
        Current_Loop_Para_Update();
        return true;
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        Flux_Result = Flux_Result_Get();

        if (!Flux_Result->Valid)
        {
            return false;
        }

        Motor_Para.Flux = Flux_Result->Flux_Wb;
        return true;
    }

    return false;
}

bool Identification_Active(void)
{
    return Ident_State == IDENT_RUNNING;
}

Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float Ic_A,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V)
{
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (Ident_State != IDENT_RUNNING)
    {
        return FAST_OFF;
    }

    if ((Abs_Value(Ia_A) > IDENT_I_MAX_A) || (Abs_Value(Ib_A) > IDENT_I_MAX_A) || (Abs_Value(Ic_A) > IDENT_I_MAX_A))
    {
        if (Ident_Mode == IDENT_RS_LS)
        {
            Rs_Ls_Fail();
        }
        else if (Ident_Mode == IDENT_FLUX)
        {
            Flux_Fail();
        }

        return FAST_OFF;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Run(Ia_A, Ualpha_V, Ubeta_V);
        return FAST_VOLTAGE;
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return Flux_Fast_Run(Ia_A, Ib_A, Ic_A, Id_Ref, Iq_Ref);
    }

    return FAST_OFF;
}

Ident_Mode_e Identification_Mode_Get(void)
{
    return Ident_Mode;
}

Ident_State_e Identification_State_Get(void)
{
    return Ident_State;
}

uint8_t Identification_Stage_Get(void)
{
    if (Ident_Mode == IDENT_RS_LS)
    {
        return (uint8_t)Rs_Ls_State_Get();
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return (uint8_t)Flux_State_Get();
    }

    return 0U;
}
