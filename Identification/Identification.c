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

#define IDENT_PROBE_I_RATIO       0.10f
#define IDENT_MEASURE_I_RATIO     0.15f
#define IDENT_MEASURE_I_MAX_RATIO 0.20f
#define IDENT_ALIGN_I_RATIO       0.20f
#define IDENT_ALIGN_I_MAX_RATIO   0.50f
#define IDENT_I_MIN_RATIO         0.02f
#define IDENT_U_SOFT_RATIO        0.25f
#define IDENT_U_HARD_RATIO        0.80f

static volatile Ident_Mode_e Ident_Mode = IDENT_NONE;
static volatile Ident_State_e Ident_State = IDENT_IDLE;
static Ident_Envelope_T Ident_Envelope = { 0 };

static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}

static float Current_Limit_Get(void)
{
    float I_Max;

    I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    return (I_Max > 0.0f) ? I_Max : 0.0f;
}

static void Envelope_Voltage_Update(void)
{
    Ident_Envelope.U_Available_V = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    if (Ident_Envelope.U_Available_V < 0.0f)
    {
        Ident_Envelope.U_Available_V = 0.0f;
    }

    Ident_Envelope.U_Soft_V = IDENT_U_SOFT_RATIO * Ident_Envelope.U_Available_V;
    Ident_Envelope.U_Hard_V = IDENT_U_HARD_RATIO * Ident_Envelope.U_Available_V;
}

static bool Envelope_Build(void)
{
    Ident_Envelope.I_Safe_A = Current_Limit_Get();
    Ident_Envelope.I_Min_A = IDENT_I_MIN_RATIO * Ident_Envelope.I_Safe_A;

    Ident_Envelope.I_Probe_A = IDENT_PROBE_I_RATIO * Ident_Envelope.I_Safe_A;
    if (Ident_Envelope.I_Probe_A < Ident_Envelope.I_Min_A)
    {
        Ident_Envelope.I_Probe_A = Ident_Envelope.I_Min_A;
    }

    Ident_Envelope.I_Measure_A = IDENT_MEASURE_I_RATIO * Ident_Envelope.I_Safe_A;
    if (Ident_Envelope.I_Measure_A < Ident_Envelope.I_Min_A)
    {
        Ident_Envelope.I_Measure_A = Ident_Envelope.I_Min_A;
    }

    Ident_Envelope.I_Measure_Max_A = IDENT_MEASURE_I_MAX_RATIO * Ident_Envelope.I_Safe_A;
    Ident_Envelope.I_Align_A = IDENT_ALIGN_I_RATIO * Ident_Envelope.I_Safe_A;
    Ident_Envelope.I_Align_Max_A = IDENT_ALIGN_I_MAX_RATIO * Ident_Envelope.I_Safe_A;

    Envelope_Voltage_Update();

    Ident_Envelope.Valid = (Ident_Envelope.I_Safe_A > 0.0f) && (Ident_Envelope.U_Hard_V > 0.0f) &&
                           (Ident_Envelope.I_Probe_A <= Ident_Envelope.I_Measure_Max_A);
    return Ident_Envelope.Valid;
}

bool Identification_Start(Ident_Mode_e Mode, float Wm_Target)
{
    if (Ident_State == IDENT_RUNNING)
    {
        return false;
    }

    if ((Mode != IDENT_RS_LS) && (Mode != IDENT_FLUX))
    {
        return false;
    }

    if (!Envelope_Build())
    {
        return false;
    }

    if (Mode == IDENT_RS_LS)
    {
        Rs_Ls_Start();
    }
    else if (!Flux_Start(Wm_Target))
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
        Motor_Para_Changed(MOTOR_PARA_RL);
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
        Motor_Para_Changed(MOTOR_PARA_FLUX);
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
                                          float *Theta_e,
                                          float *Id_Ref,
                                          float *Iq_Ref,
                                          float *Ualpha_V,
                                          float *Ubeta_V)
{
    float I_Max;

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
    I_Max = Ident_Envelope.I_Safe_A;

    if ((I_Max <= 0.0f) || (Abs_Value(Ia_A) > I_Max) || (Abs_Value(Ib_A) > I_Max) || (Abs_Value(Ic_A) > I_Max))
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
        return Rs_Ls_Run(Ia_A, Theta_e, Id_Ref, Iq_Ref, Ualpha_V, Ubeta_V);
    }

    if (Ident_Mode == IDENT_FLUX)
    {
        return Flux_Fast_Run(Ia_A, Ib_A, Ic_A, Theta_e, Id_Ref, Iq_Ref);
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

const Ident_Envelope_T *Identification_Envelope_Get(void)
{
    return &Ident_Envelope;
}
