/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Control.h"

#include "Math.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Flux.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Cal.h"
#include "Motor_Config.h"
#include "Motor_Para.h"
#include "Motor_PWM.h"
#include "Motion_Loop.h"
#include "Open_Loop.h"
#include "Ramp.h"
#include "Sensorless.h"
#include "Servo_Phase.h"
#include "Trapezoid.h"
#include "control_params.h"
#include "motor_params.h"

typedef struct
{
    float Id;
    float Iq;

} Current_Ref_T;

Motor_Cmd_T Motor_Cmd = { 0 };
uint8_t Motor_Mode = (uint8_t)TORQUE;
Motor_Cal_T Motor_Cal = MOTOR_CAL_DEFAULT;
const Motor_Limit_T Motor_Lim = MOTOR_LIM_DEFAULT;
Motor_Limit_T User_Lim = USER_LIM_DEFAULT;
Motor_Run_T Motor_Run = { 0 };

volatile float Motor_Plot_Wm = 0.0f;
volatile float Motor_Plot_Wm_Ref = 0.0f;
volatile float Motor_Plot_Position = 0.0f;
volatile float Motor_Plot_Position_Ref = 0.0f;

static volatile Motor_State_e Motor_State = DISABLED;

static Current_Ref_T Current_Ref = { 0 };
static Motion_Ref_T Motion_Ref = { 0 };
static float Wm_Ref = 0.0f;
static float We_Ref = 0.0f;
static uint32_t Pos_Div = 0U;
static bool Stop_Pending = false;

static FAST_CODE void Motor_Limit_Get(Motor_Limit_T *Lim)
{
    Lim->I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    Lim->Wm_Max = (Motor_Lim.Wm_Max < User_Lim.Wm_Max) ? Motor_Lim.Wm_Max : User_Lim.Wm_Max;

    if (Motion_Config.Wm_Max < Lim->Wm_Max)
    {
        Lim->Wm_Max = Motion_Config.Wm_Max;
    }

    if (Lim->I_Max < 0.0f)
    {
        Lim->I_Max = 0.0f;
    }

    if (Lim->Wm_Max < 0.0f)
    {
        Lim->Wm_Max = 0.0f;
    }
}

static void Iq_Limit_Calc(float *Iq_Min, float *Iq_Max)
{
    float We;
    float U_Lim;
    float Wlq;
    float Bemf;
    float A;
    float B;
    float C;
    float D;
    float Sqrt_D;

    We = (float)Motor_Para.Pp * Motor_Run.Wm;
    U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    Wlq = We * Motor_Para.Lq;
    Bemf = We * Motor_Para.Flux;

    A = Motor_Para.Rs * Motor_Para.Rs + Wlq * Wlq;
    B = 2.0f * Motor_Para.Rs * Bemf;
    C = Bemf * Bemf - U_Lim * U_Lim;

    if (A <= 0.0f)
    {
        *Iq_Min = 0.0f;
        *Iq_Max = 0.0f;
        return;
    }

    D = B * B - 4.0f * A * C;

    if (D < 0.0f)
    {
        *Iq_Min = 0.0f;
        *Iq_Max = 0.0f;
        return;
    }

    Sqrt_D = __builtin_sqrtf(D);

    *Iq_Min = (-B - Sqrt_D) / (2.0f * A);
    *Iq_Max = (-B + Sqrt_D) / (2.0f * A);
}

static void Motion_State_Reset(void)
{
    Speed_Loop_State_Reset((float)Motor_Para.Pp * Mechanical_ESO_Wm_Get());
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Trapezoid_Reset(&Motion_Ref, Motor_Run.Turn, Motor_Run.Theta_m, 0.0f);
    Pos_Div = 0U;
}

static FAST_CODE bool Servo_Mode(void)
{
    return (Motor_Mode == TORQUE) || (Motor_Mode == SPEED) || (Motor_Mode == POSITION);
}

static FAST_CODE float Encoder_Theta_e(void)
{
    return Angle_Wrap((float)Motor_Para.Pp * Motor_Run.Theta_m + Motor_Cal.Theta_Off);
}

static void Disable_Apply(void)
{
    Stop_Pending = false;

    if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_Stop();
    }

    PWM_Disable();
    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Motor_State = DISABLED;
}

void Motor_Control(void)
{
    int32_t Pos_Turn_Target;
    float Pos_Theta_Target;
    float Kt;
    float Te_Max;
    float Te_Ref;
    float Wm_Target;
    float We_Fbk;
    float Iq_Min;
    float Iq_Max;
    float Wm_Corr;
    Motor_Limit_T Lim;

    if (Motor_State == DISABLED)
    {
        return;
    }

    if (Motor_Mode == PHASE_SEARCH)
    {
        if ((Motor_State == RUN) && !Servo_Phase_Active())
        {
            Disable_Apply();
            (void)Servo_Phase_Apply();
        }

        return;
    }

    if (Motor_Mode == IDENT)
    {
        if (Motor_State == RUN)
        {
            Identification_Control();

            if (!Identification_Active())
            {
                Current_Ref.Id = 0.0f;
                Current_Ref.Iq = 0.0f;
                PWM_Disable();
                Motor_State = ENABLED;
            }
        }

        return;
    }

    Motor_Limit_Get(&Lim);

    if ((Motor_Mode == SPEED) || (Motor_Mode == OPEN_LOOP) || (Motor_Mode == SENSORLESS_SPEED))
    {
        if (Stop_Pending)
        {
            Wm_Target = 0.0f;
        }
        else
        {
            Wm_Target = (Motor_State == RUN) ? Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
        }

        Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
        Wm_Ref = Motion_Ref.Wm;
        We_Ref = (float)Motor_Para.Pp * Wm_Ref;

        if (Stop_Pending && (Motion_Ref.Wm == 0.0f))
        {
            Stop_Pending = false;

            if (Motor_Mode == SENSORLESS_SPEED)
            {
                Sensorless_Stop();
                Current_Ref.Id = 0.0f;
                Current_Ref.Iq = 0.0f;
                PWM_Disable();
            }

            Motor_State = ENABLED;
        }
    }

    if (Motor_Mode == OPEN_LOOP)
    {
        Current_Ref.Id = 0.0f;
        Current_Ref.Iq = 0.0f;
        return;
    }

    if (Motor_Mode == SENSORLESS_SPEED)
    {
        Current_Ref.Id = 0.0f;
        Current_Ref.Iq = 0.0f;

        if (Motor_State == RUN)
        {
            Sensorless_Control(We_Ref, -Lim.I_Max, Lim.I_Max);
        }

        return;
    }

    Current_Ref.Id = 0.0f;

    Iq_Limit_Calc(&Iq_Min, &Iq_Max);

    if (Iq_Min < -Lim.I_Max)
    {
        Iq_Min = -Lim.I_Max;
    }

    if (Iq_Max > Lim.I_Max)
    {
        Iq_Max = Lim.I_Max;
    }

    if (Iq_Min > Iq_Max)
    {
        Iq_Min = 0.0f;
        Iq_Max = 0.0f;
    }

    We_Fbk = (float)Motor_Para.Pp * Mechanical_ESO_Wm_Get();

    switch (Motor_Mode)
    {
        case TORQUE:
            if (Motor_State == RUN)
            {
                Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;

                if (!__builtin_isfinite(Kt) || (Kt <= 0.0f))
                {
                    Current_Ref.Iq = 0.0f;
                    break;
                }

                Te_Max = Kt * Lim.I_Max;
                Te_Ref = Motor_User_To_Internal(Motor_Cmd.Te_Target);
                Limit_Value(&Te_Ref, -Te_Max, Te_Max);
                Current_Ref.Iq = Te_Ref / Kt;
            }
            else
            {
                Current_Ref.Iq = 0.0f;
            }
            break;

        case SPEED:
            Current_Ref.Iq = Speed_Loop(We_Ref, We_Fbk, Iq_Min, Iq_Max);
            break;

        case POSITION:
            if (Pos_Div == 0U)
            {
                if (Stop_Pending)
                {
                    if (Trapezoid_Stop(&Motion_Ref, Motion_Config.Wm_Dec, POS_TS))
                    {
                        Stop_Pending = false;
                        Motor_State = ENABLED;
                    }
                }
                else if (Motor_State == RUN)
                {
                    Motor_Position_User_To_Internal(Motor_Cmd.Position_Target.Turn,
                                                    Motor_Cmd.Position_Target.Theta,
                                                    &Pos_Turn_Target,
                                                    &Pos_Theta_Target);
                    Trapezoid_Run(&Motion_Ref,
                                  Pos_Turn_Target,
                                  Pos_Theta_Target,
                                  Lim.Wm_Max,
                                  Motion_Config.Wm_Acc,
                                  Motion_Config.Wm_Dec,
                                  POS_TS);
                }

                Wm_Corr = Position_Loop(Motion_Ref.Turn, Motion_Ref.Theta, -Lim.Wm_Max, Lim.Wm_Max);
                Wm_Ref = Motion_Ref.Wm + Wm_Corr;
                Limit_Value(&Wm_Ref, -Lim.Wm_Max, Lim.Wm_Max);
            }

            Pos_Div ^= 1U;
            We_Ref = (float)Motor_Para.Pp * Wm_Ref;
            Current_Ref.Iq = Speed_Loop(We_Ref, We_Fbk, Iq_Min, Iq_Max);
            break;

        default:
            Current_Ref.Iq = 0.0f;
            break;
    }

    Limit_Value(&Current_Ref.Iq, Iq_Min, Iq_Max);
}

Motor_Fast_Mode_e Motor_Fast_Run(float *Theta_e,
                                 float *Id_Ref,
                                 float *Iq_Ref,
                                 float *Ualpha,
                                 float *Ubeta)
{
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha = 0.0f;
    *Ubeta = 0.0f;

    if (Motor_State == DISABLED)
    {
        return FAST_OFF;
    }

    switch (Motor_Mode)
    {
        case TORQUE:
        case SPEED:
        case POSITION:
            *Theta_e = Encoder_Theta_e();
            *Id_Ref = Current_Ref.Id;
            *Iq_Ref = Current_Ref.Iq;
            return FAST_CURRENT;

        case OPEN_LOOP:
            if (Motor_State != RUN)
            {
                return FAST_OFF;
            }

            Open_Loop(We_Ref, Theta_e, Id_Ref, Iq_Ref);
            return FAST_CURRENT;

        case IDENT:
            if (Motor_State != RUN)
            {
                return FAST_OFF;
            }

            return Identification_Fast_Run(ADC.Ia_A,
                                           ADC.Ib_A,
                                           ADC.Ic_A,
                                           Theta_e,
                                           Id_Ref,
                                           Iq_Ref,
                                           Ualpha,
                                           Ubeta);

        case SENSORLESS_SPEED:
            if (Motor_State != RUN)
            {
                return FAST_OFF;
            }

            Sensorless_Run(ADC.Ia_A, ADC.Ib_A, We_Ref, Theta_e, Id_Ref, Iq_Ref);
            return FAST_CURRENT;

        case PHASE_SEARCH:
            if (Motor_State != RUN)
            {
                return FAST_OFF;
            }

            return Servo_Phase_Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);

        default:
            return FAST_OFF;
    }
}

Motor_State_e Motor_State_Get(void)
{
    return Motor_State;
}

Motor_Mode_e Motor_Mode_Get(void)
{
    return (Motor_Mode_e)Motor_Mode;
}

float Motor_I_Limit_Effective_Get(void)
{
    Motor_Limit_T Lim;

    Motor_Limit_Get(&Lim);
    return Lim.I_Max;
}

float Motor_Wm_Limit_Effective_Get(void)
{
    Motor_Limit_T Lim;

    Motor_Limit_Get(&Lim);
    return Lim.Wm_Max;
}

float Motor_Wm_Ref_Get(void)
{
    return Motor_Internal_To_User(Wm_Ref);
}

Motor_Position_T Motor_Position_Ref_Get(void)
{
    Motor_Position_T Position;

    Motor_Position_Internal_To_User(Motion_Ref.Turn,
                                    Motion_Ref.Theta,
                                    &Position.Turn,
                                    &Position.Theta);
    return Position;
}

bool Motor_Encoder_Required(void)
{
    return Servo_Mode() || (Motor_Mode == PHASE_SEARCH);
}

bool Motor_Mechanical_ESO_Required(void)
{
    return Servo_Mode() &&
           (Motor_State != DISABLED) &&
           (Motor_Cal.Valid != 0U) &&
           (Encoder.Ready != 0U) &&
           (Encoder.Fault == 0U);
}

void Motor_Enable(void)
{
    float Kt;

    Stop_Pending = false;

    if (Motor_State != DISABLED)
    {
        return;
    }

    if (Motor_Encoder_Required() && ((Encoder.Ready == 0U) || (Encoder.Fault != 0U)))
    {
        return;
    }

    if (Servo_Mode() && (Motor_Cal.Valid == 0U))
    {
        return;
    }

    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;

    if (Servo_Mode())
    {
        Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
        if (!Mechanical_ESO_Config(Motor_Para.J,
                                   Motor_Para.B,
                                   Kt,
                                   TWO_PI_F * Mechanical_ESO_Bw_Hz))
        {
            return;
        }

        Mechanical_ESO_Reset(Encoder_Position_Get(), Motor_Run.Wm);
        Motion_State_Reset();
        Current_Loop_State_Reset();
    }

    Motor_Run.Ud = 0.0f;
    Motor_Run.Uq = 0.0f;
    Motor_Run.Ualpha = 0.0f;
    Motor_Run.Ubeta = 0.0f;

    if (Servo_Mode())
    {
        PWM_Enable();
    }

    Motor_State = ENABLED;
}

void Motor_Start(void)
{
    if (Motor_State != ENABLED)
    {
        return;
    }

    /* Identification is entered only through Motor_Ident_Start(). */
    if (Motor_Mode == IDENT)
    {
        return;
    }

    if (Motor_Mode == OPEN_LOOP)
    {
        Current_Loop_State_Reset();
        Open_Loop_Reset();
    }
    else if (Motor_Mode == SENSORLESS_SPEED)
    {
        if (!Sensorless_Begin())
        {
            return;
        }
    }
    else if (Motor_Mode == PHASE_SEARCH)
    {
        Current_Loop_State_Reset();

        if (!Servo_Phase_Start())
        {
            return;
        }
    }

    if (!Servo_Mode())
    {
        PWM_Enable();
    }

    Stop_Pending = false;
    Motor_State = RUN;
}

void Motor_Stop(void)
{
    if (Motor_State != RUN)
    {
        return;
    }

    if ((Motor_Mode == SPEED) || (Motor_Mode == POSITION) || (Motor_Mode == SENSORLESS_SPEED))
    {
        if (Motor_Mode == SENSORLESS_SPEED)
        {
            Sensorless_Stop_Request();
        }

        Stop_Pending = true;
        return;
    }

    if (Motor_Mode == OPEN_LOOP)
    {
        Open_Loop_Reset();
    }
    else if (Motor_Mode == IDENT)
    {
        if (Identification_Active())
        {
            (void)Motor_Ident_Abort();
        }
        else
        {
            PWM_Disable();
            Motor_State = ENABLED;
        }
        return;
    }
    else if (Motor_Mode == PHASE_SEARCH)
    {
        Servo_Phase_Abort();
    }

    Current_Ref.Id = 0.0f;

    if (Motor_Mode == TORQUE)
    {
        Current_Ref.Iq = 0.0f;
    }

    if (!Servo_Mode())
    {
        PWM_Disable();
    }

    Motor_State = ENABLED;
}

void Motor_Disable(void)
{
    if (Motor_State == RUN)
    {
        Motor_Stop();
    }

    if ((Motor_Mode == IDENT) && (Identification_State_Get() == IDENT_FAILED))
    {
        Identification_Abort();
    }

    Disable_Apply();
}

bool Motor_Ident_Start(Ident_Mode_e Mode)
{
    if ((Motor_State != ENABLED) ||
        (Motor_Mode != IDENT) ||
        ((Mode != IDENT_RS_LS) && (Mode != IDENT_FLUX) && (Mode != IDENT_JB)))
    {
        return false;
    }

    if (!Identification_Start(Mode, Motor_Cmd.Wm_Target))
    {
        return false;
    }

    PWM_Enable();
    Motor_State = RUN;
    return true;
}

bool Motor_Ident_Abort(void)
{
    if ((Motor_State != RUN) ||
        (Motor_Mode != IDENT) ||
        !Identification_Active())
    {
        return false;
    }

    Identification_Abort();
    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    PWM_Disable();
    Motor_State = ENABLED;
    return true;
}

bool Motor_Ident_Apply(void)
{
    if ((Motor_State == RUN) || (Motor_Mode != IDENT))
    {
        return false;
    }

    return Identification_Apply();
}

float Motor_Wm_Get(void)
{
    /* The ESO stops updating while disabled; do not publish its held speed. */
    if (Motor_State == DISABLED)
    {
        return 0.0f;
    }

    if (Servo_Mode() && (Mechanical_ESO.Para.Valid != 0U))
    {
        return Motor_Internal_To_User(Mechanical_ESO_Wm_Get());
    }

    return Motor_Internal_To_User(Motor_Run.Wm);
}

void Motor_Position_Get(int32_t *Turn, float *Theta)
{
    Motor_Position_Internal_To_User(Motor_Run.Turn, Motor_Run.Theta_m, Turn, Theta);
}

void Motor_Plot_Normal_Update(void)
{
    Motor_Position_T Ref;
    int32_t Turn;
    float Theta;

    Motor_Plot_Wm = Motor_Wm_Get();
    Motor_Plot_Wm_Ref = Motor_Wm_Ref_Get();

    Motor_Position_Get(&Turn, &Theta);
    Motor_Plot_Position = (float)Turn + Theta / TWO_PI_F;

    Ref = Motor_Position_Ref_Get();
    Motor_Plot_Position_Ref = (float)Ref.Turn + Ref.Theta / TWO_PI_F;
}
