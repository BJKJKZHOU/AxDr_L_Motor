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
#include "SCurve.h"
#include "Sensorless.h"
#include "Servo_Phase.h"
#include "Signal_Injection.h"
#include "Trapezoid.h"
#include "control_params.h"
#include "motor_params.h"

#define SENSORLESS_STOP_DELAY_S     0.10f
#define SENSORLESS_STOP_DELAY_TICKS ((uint32_t)(SENSORLESS_STOP_DELAY_S / SPD_TS + 0.5f))

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
static SCurve_T SCurve = { 0 };
static float Wm_Ref = 0.0f;
static float We_Ref = 0.0f;
static float Sensorless_We_Target = 0.0f;
static uint32_t Pos_Div = 0U;
static bool Stop_Pending = false;
static bool Sensorless_Profile_Active = false;
static uint32_t Sensorless_Stop_Delay_Cnt = 0U;

static FAST_CODE void Motor_Limit_Get(Motor_Limit_T *Lim)
{
    Lim->I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    Lim->Wm_Max = (Motor_Lim.Wm_Max < User_Lim.Wm_Max) ? Motor_Lim.Wm_Max : User_Lim.Wm_Max;

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

    We = (float)Motor_Para.Pp * Mechanical_ESO_Wm_Get();
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
    SCurve = (SCurve_T){ 0 };
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

static void NonServo_Enabled_Apply(void)
{
    if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_Stop();
    }

    Stop_Pending = false;
    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Sensorless_We_Target = 0.0f;
    Sensorless_Profile_Active = false;
    Sensorless_Stop_Delay_Cnt = 0U;
    Motor_Run.Ud = 0.0f;
    Motor_Run.Uq = 0.0f;
    Motor_Run.Ualpha = 0.0f;
    Motor_Run.Ubeta = 0.0f;
    PWM_Disable();
    Motor_State = ENABLED;
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
    Sensorless_We_Target = 0.0f;
    Sensorless_Profile_Active = false;
    Sensorless_Stop_Delay_Cnt = 0U;
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
    float Iq_Target;
    float Iq_Step;
    Motor_Limit_T Lim;

    if (Motor_State == DISABLED)
    {
        return;
    }

    if (Motor_Mode == PHASE_SEARCH)
    {
        if ((Motor_State == RUN) && !Servo_Phase_Active())
        {
            NonServo_Enabled_Apply();
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
                NonServo_Enabled_Apply();
            }
        }

        return;
    }

    Motor_Limit_Get(&Lim);

    switch (Motor_Mode)
    {
        case OPEN_LOOP:
            Wm_Target = ((Motor_State == RUN) && !Stop_Pending) ?
                        Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
            Wm_Ref = Motion_Ref.Wm;
            We_Ref = (float)Motor_Para.Pp * Wm_Ref;

            Current_Ref.Id = 0.0f;
            Current_Ref.Iq = 0.0f;
            return;

        case SENSORLESS_SPEED:
            if (Sensorless_State_Get() == SL_FAILED)
            {
                NonServo_Enabled_Apply();
                return;
            }

            Wm_Target = ((Motor_State == RUN) && !Stop_Pending) ?
                        Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            Sensorless_We_Target = (float)Motor_Para.Pp * Wm_Target;

            if (Sensorless_Profile_Active && !Sensorless_Speed_Control_Active())
            {
                Sensorless_Profile_Active = false;
            }

            if (!Sensorless_Profile_Active && Sensorless_Speed_Control_Active())
            {
                Wm_Ref = Sensorless_Wm_Get();
                Ramp_Reset(&Motion_Ref, Wm_Ref);
                Sensorless_Profile_Active = true;
            }

            if (Sensorless_Profile_Active)
            {
                Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
                Wm_Ref = Motion_Ref.Wm;
            }
            else
            {
                Wm_Ref = Sensorless_Wm_Get();
            }

            We_Ref = (float)Motor_Para.Pp * Wm_Ref;

            if (Stop_Pending && (Wm_Ref == 0.0f))
            {
                if (Sensorless_Stop_Delay_Cnt < SENSORLESS_STOP_DELAY_TICKS)
                {
                    Sensorless_Stop_Delay_Cnt++;
                }

                if (Sensorless_Stop_Delay_Cnt >= SENSORLESS_STOP_DELAY_TICKS)
                {
                    NonServo_Enabled_Apply();
                }
            }
            else
            {
                Sensorless_Stop_Delay_Cnt = 0U;
            }

            Current_Ref.Id = 0.0f;
            Current_Ref.Iq = 0.0f;

            if (Motor_State == RUN)
            {
                Sensorless_Control(We_Ref, -Lim.I_Max, Lim.I_Max);
            }
            return;

        default:
            break;
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
            Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
            if ((Motor_State == RUN) && __builtin_isfinite(Kt) && (Kt > 0.0f))
            {
                Te_Max = Kt * Lim.I_Max;
                Te_Ref = Stop_Pending ? 0.0f : Motor_User_To_Internal(Motor_Cmd.Te_Target);
                Limit_Value(&Te_Ref, -Te_Max, Te_Max);
                Iq_Target = Te_Ref / Kt;
                if (Motion_Config.Te_Rate > 0.0f)
                {
                    /* Continue from the current-limited reference, without a second
                     * torque state that could run ahead during saturation. */
                    Iq_Step = Motion_Config.Te_Rate * SPD_TS / Kt;
                    Limit_Value(&Iq_Target, Current_Ref.Iq - Iq_Step, Current_Ref.Iq + Iq_Step);
                }
                Current_Ref.Iq = Iq_Target;
            }
            else
            {
                Current_Ref.Iq = 0.0f;
            }

            /* Stop completes on the reference actually passed to the current loop. */
            Limit_Value(&Current_Ref.Iq, Iq_Min, Iq_Max);
            if (Stop_Pending && (Current_Ref.Iq == 0.0f))
            {
                Stop_Pending = false;
                Motor_State = ENABLED;
            }
            return;

        case SPEED:
            Wm_Target = ((Motor_State == RUN) && !Stop_Pending) ?
                        Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            if (Motion_Config.Profile == MOTION_TRAPEZOID)
            {
                Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
            }
            else
            {
                SCurve_Speed(&SCurve, &Motion_Ref, Wm_Target,
                             Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, Motion_Config.Profile, SPD_TS);
            }
            Wm_Ref = Motion_Ref.Wm;
            We_Ref = (float)Motor_Para.Pp * Wm_Ref;

            if (Stop_Pending && (Motion_Ref.Wm == 0.0f) &&
                ((Motion_Config.Profile == MOTION_TRAPEZOID) || (SCurve.Count == 0U)))
            {
                Stop_Pending = false;
                Motor_State = ENABLED;
            }

            Current_Ref.Iq = Speed_Loop(We_Ref, We_Fbk, Iq_Min, Iq_Max);
            break;

        case POSITION:
            if (Pos_Div == 0U)
            {
                if (Stop_Pending)
                {
                    bool Stopped;

                    if (Motion_Config.Profile == MOTION_TRAPEZOID)
                    {
                        Stopped = Trapezoid_Stop(&Motion_Ref, Motion_Config.Wm_Dec, POS_TS);
                    }
                    else
                    {
                        SCurve_Speed(&SCurve, &Motion_Ref, 0.0f,
                                     Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, Motion_Config.Profile, POS_TS);
                        Stopped = (SCurve.Count == 0U) && (Motion_Ref.Wm == 0.0f);
                    }
                    if (Stopped)
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
                    /* Position determines direction; the shared speed command
                     * supplies magnitude. Zero brakes the trajectory in place. */
                    Wm_Target = __builtin_fabsf(Motor_Cmd.Wm_Target);
                    Limit_Value(&Wm_Target, 0.0f, Lim.Wm_Max);
                    if (Motion_Config.Profile == MOTION_TRAPEZOID)
                    {
                        Trapezoid_Run(&Motion_Ref,
                                      Pos_Turn_Target,
                                      Pos_Theta_Target,
                                      Wm_Target,
                                      Motion_Config.Wm_Acc,
                                      Motion_Config.Wm_Dec,
                                      POS_TS);
                    }
                    else
                    {
                        SCurve_Position(&SCurve, &Motion_Ref, Pos_Turn_Target, Pos_Theta_Target,
                                        Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec,
                                        Motion_Config.Profile, POS_TS);
                    }
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

    /* Sensored modes retain current control while ENABLED for servo holding. */
    if (Servo_Mode())
    {
        *Theta_e = Encoder_Theta_e();
        *Id_Ref = Current_Ref.Id;
        *Iq_Ref = Current_Ref.Iq;
        return FAST_CURRENT;
    }

    if (Motor_State != RUN)
    {
        return FAST_OFF;
    }

    switch (Motor_Mode)
    {
        case OPEN_LOOP:
            Open_Loop(We_Ref, Theta_e, Id_Ref, Iq_Ref);
            return FAST_CURRENT;

        case IDENT:
            return Identification_Fast_Run(ADC.Ia_A,
                                           ADC.Ib_A,
                                           ADC.Ic_A,
                                           Theta_e,
                                           Id_Ref,
                                           Iq_Ref,
                                           Ualpha,
                                           Ubeta);

        case SENSORLESS_SPEED:
            Sensorless_Run(ADC.Ia_A, ADC.Ib_A, Sensorless_We_Target, Theta_e, Id_Ref, Iq_Ref);
            return FAST_CURRENT;

        case PHASE_SEARCH:
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

void Motor_Enable(void)
{
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
        if (Mechanical_ESO.Para.Valid == 0U)
        {
            return;
        }

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

    Signal_Injection_Stop();

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
        Sensorless_Profile_Active = false;
        Sensorless_We_Target = 0.0f;
        Ramp_Reset(&Motion_Ref, 0.0f);
        Wm_Ref = 0.0f;
        We_Ref = 0.0f;

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
    Sensorless_Stop_Delay_Cnt = 0U;
    Motor_State = RUN;
}

void Motor_Stop(void)
{
    /* Stop cancels test excitation even when the servo is holding ENABLED. */
    Signal_Injection_Stop();

    if (Motor_State != RUN)
    {
        return;
    }

    if ((Motor_Mode == TORQUE) || (Motor_Mode == SPEED) ||
        (Motor_Mode == POSITION) || (Motor_Mode == SENSORLESS_SPEED))
    {
        if (Motor_Mode == SENSORLESS_SPEED)
        {
            Sensorless_Stop_Request();
            Sensorless_Stop_Delay_Cnt = 0U;
        }

        Stop_Pending = true;
        return;
    }

    if (Motor_Mode == OPEN_LOOP)
    {
        Open_Loop_Reset();
        NonServo_Enabled_Apply();
        return;
    }

    if (Motor_Mode == IDENT)
    {
        if (Identification_Active())
        {
            (void)Motor_Ident_Abort();
        }
        else
        {
            NonServo_Enabled_Apply();
        }
        return;
    }

    if (Motor_Mode == PHASE_SEARCH)
    {
        Servo_Phase_Abort();
        NonServo_Enabled_Apply();
        return;
    }

    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    Motor_State = ENABLED;
}

void Motor_Disable(void)
{
    Signal_Injection_Stop();

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
    NonServo_Enabled_Apply();
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
    if ((Motor_Mode == SENSORLESS_SPEED) && (Motor_State == RUN))
    {
        return Motor_Internal_To_User(Sensorless_Wm_Get());
    }

    if ((Encoder.Ready != 0U) && (Encoder.Fault == 0U) &&
        (Mechanical_ESO.Para.Valid != 0U))
    {
        return Motor_Internal_To_User(Mechanical_ESO_Wm_Get());
    }

    return 0.0f;
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
