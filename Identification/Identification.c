/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Identification.h"

#include "Flux.h"
#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Rs_Ls.h"
#include "control_params.h"

#define IDENT_U_MAX_RATIO  0.80f
#define IDENT_I_TRIP_COUNT 5U

Flux_Observer_T Ident_Observer = { 0 };
PLL_T Ident_PLL = { 0 };
volatile float Ident_IF_Current_A = 1.0f;

static volatile Ident_Mode_e Ident_Mode = IDENT_NONE;
static volatile Ident_State_e Ident_State = IDENT_IDLE;
static volatile Ident_Fail_Reason_e Ident_Fail_Reason = IDENT_FAIL_NONE;
static Ident_Envelope_T Ident_Envelope = { 0 };
static uint8_t Current_Over_Cnt = 0U;

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static bool Current_Over_Trip(float Ia_A, float Ib_A, float Ic_A)
{
    float I_Peak;

    if (!__builtin_isfinite(Ia_A) || !__builtin_isfinite(Ib_A) ||
        !__builtin_isfinite(Ic_A) || !__builtin_isfinite(Ident_Envelope.I_Max) ||
        (Ident_Envelope.I_Max <= 0.0f))
    {
        return true;
    }

    I_Peak = Abs_Value(Ia_A);
    if (Abs_Value(Ib_A) > I_Peak)
    {
        I_Peak = Abs_Value(Ib_A);
    }
    if (Abs_Value(Ic_A) > I_Peak)
    {
        I_Peak = Abs_Value(Ic_A);
    }

    if (I_Peak >= Ident_Envelope.I_Max)
    {
        if (Current_Over_Cnt < IDENT_I_TRIP_COUNT)
        {
            Current_Over_Cnt++;
        }
    }
    else
    {
        Current_Over_Cnt = 0U;
    }

    return Current_Over_Cnt >= IDENT_I_TRIP_COUNT;
}

static void Envelope_Voltage_Update(void)
{
    Ident_Envelope.U_Available = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    if (Ident_Envelope.U_Available < 0.0f)
    {
        Ident_Envelope.U_Available = 0.0f;
    }

    Ident_Envelope.U_Max = IDENT_U_MAX_RATIO * Ident_Envelope.U_Available;
}

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target)
{
    Ident_Fail_Reason = IDENT_FAIL_NONE;
    Current_Over_Cnt = 0U;

    if (Ident_State == IDENT_RUNNING)
    {
        return false;
    }

    if ((Mode != IDENT_RS_LS) && (Mode != IDENT_FLUX))
    {
        Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
        return false;
    }

    if (Ident_State != IDENT_IDLE)
    {
        Identification_Abort();
    }

    Ident_Envelope.I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    if (Ident_Envelope.I_Max < 0.0f)
    {
        Ident_Envelope.I_Max = 0.0f;
    }

    Envelope_Voltage_Update();
    if ((Ident_Envelope.I_Max <= 0.0f) || (Ident_Envelope.U_Max <= 0.0f))
    {
        Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
        return false;
    }

    if (Mode == IDENT_RS_LS)
    {
        Rs_Ls_Start();
    }
    else if (!Flux_Start(Wm_Target))
    {
        Ident_Fail_Reason = IDENT_FAIL_START_CONFIG;
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
        Rs_Ls_Abort();
    }

    Ident_Mode = IDENT_NONE;
    Ident_State = IDENT_IDLE;
    Current_Over_Cnt = 0U;
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
        if (Flux_Result->Valid)
        {
            Ident_State = IDENT_DONE;
        }
        else
        {
            if (Ident_Fail_Reason == IDENT_FAIL_NONE)
            {
                Ident_Fail_Reason = IDENT_FAIL_FLUX_INTERNAL;
            }
            Ident_State = IDENT_FAILED;
        }
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
        Motor_Para_Update();
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
        Motor_Para_Update();
        return true;
    }

    return false;
}

bool Identification_Active(void)
{
    return Ident_State == IDENT_RUNNING;
}

bool Identification_Result_Valid(void)
{
    if (Ident_State != IDENT_DONE)
    {
        return false;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        return Rs_Ls_Result_Get()->Valid;
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return Flux_Result_Get()->Valid;
    }

    return false;
}

Motor_Fast_Mode_e Identification_Fast_Run(float Ia_A,
                                          float Ib_A,
                                          float Ic_A,
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V)
{
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (Ident_State != IDENT_RUNNING)
    {
        return FAST_OFF;
    }

    Envelope_Voltage_Update();
    if (Current_Over_Trip(Ia_A, Ib_A, Ic_A))
    {
        if (Ident_Mode == IDENT_RS_LS)
        {
            Rs_Ls_Abort();
        }

        Ident_Fail_Reason = IDENT_FAIL_PHASE_CURRENT;
        Ident_State = IDENT_FAILED;
        return FAST_OFF;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        return Rs_Ls_Run(Ia_A, Theta_e, Id_Ref, Iq_Ref, Ualpha_V, Ubeta_V);
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return Flux_Fast_Run(Ia_A,
                             Ib_A,
                             Ic_A,
                             Theta_e,
                             Id_Ref,
                             Iq_Ref,
                             Ualpha_V,
                             Ubeta_V);
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

const Ident_Envelope_T *Identification_Envelope_Get(void)
{
    return &Ident_Envelope;
}

uint8_t Identification_Rs_Ls_Valid_Get(void)
{
    return Rs_Ls_Result_Get()->Valid ? 1U : 0U;
}

float Identification_Rs_Get(void)
{
    return Rs_Ls_Result_Get()->Rs_Ohm;
}

float Identification_Ls_Get(void)
{
    return Rs_Ls_Result_Get()->Ls_H;
}

uint8_t Identification_Flux_Valid_Get(void)
{
    return Flux_Result_Get()->Valid ? 1U : 0U;
}

float Identification_Flux_Get(void)
{
    return Flux_Result_Get()->Flux_Wb;
}

uint8_t Identification_Fail_Reason_Get(void)
{
    return (uint8_t)Ident_Fail_Reason;
}
