/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Control.h"

#include "Math.h"
#include "Current_Loop.h"
#include "Flux.h"
#include "Motor_ADC.h"
#include "Motor_Para.h"
#include "Motor_PWM.h"
#include "Motion_Loop.h"
#include "Open_Loop.h"
#include "Sensorless.h"
#include "control_params.h"
#include "motor_params.h"

typedef struct
{
    float Id;
    float Iq;

} Current_Ref_T;

typedef Motor_Fast_Mode_e (*Fast_Run_T)(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta);

Motor_Cal_T Motor_Cal = MOTOR_CAL_DEFAULT;
const Motor_Limit_T Motor_Lim = MOTOR_LIM_DEFAULT;
Motor_Limit_T User_Lim = USER_LIM_DEFAULT;
Motor_Run_T Motor_Run = { 0 };

static Motor_Cmd_T Motor_Cmd = { 0 };
static volatile Motor_State_e Motor_State = DISABLED;
static Motor_Mode_e Motor_Mode = TORQUE;
static Ident_Mode_e Ident_Mode = IDENT_RS_LS;

static Current_Ref_T Current_Ref = { 0 };
static float Wm_Ref = 0.0f;
static float We_Ref = 0.0f;
static int32_t Pos_Ref_Turn = 0;
static float Pos_Ref_Theta = 0.0f;
static uint32_t Pos_Div = 0U;

static Motor_Fast_Mode_e Fast_Off_Run(float *Theta_e,
                                      float *Id_Ref,
                                      float *Iq_Ref,
                                      float *Ualpha,
                                      float *Ubeta);
static Motor_Fast_Mode_e Servo_Fast_Run(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta);
static Motor_Fast_Mode_e Open_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta);
static Motor_Fast_Mode_e Ident_Fast_Run(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta);
static Motor_Fast_Mode_e Sensorless_Fast_Run(float *Theta_e,
                                             float *Id_Ref,
                                             float *Iq_Ref,
                                             float *Ualpha,
                                             float *Ubeta);

static Fast_Run_T Fast_Run = Fast_Off_Run;

static void Motor_Limit_Get(Motor_Limit_T *Lim)
{
    Lim->I_Max = (Motor_Lim.I_Max < User_Lim.I_Max) ? Motor_Lim.I_Max : User_Lim.I_Max;
    Lim->Te_Max = (Motor_Lim.Te_Max < User_Lim.Te_Max) ? Motor_Lim.Te_Max : User_Lim.Te_Max;
    Lim->Wm_Max = (Motor_Lim.Wm_Max < User_Lim.Wm_Max) ? Motor_Lim.Wm_Max : User_Lim.Wm_Max;

    if (Lim->I_Max < 0.0f)
    {
        Lim->I_Max = 0.0f;
    }

    if (Lim->Te_Max < 0.0f)
    {
        Lim->Te_Max = 0.0f;
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
    Speed_Ctrl.State.Int = 0.0f;
    Speed_Ctrl.State.Fbk_Pre = (float)Motor_Para.Pp * Motor_Run.Wm;
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Pos_Ref_Turn = Motor_Run.Turn;
    Pos_Ref_Theta = Motor_Run.Theta_m;
    Pos_Div = 0U;
}

static bool Motion_Mode_Active(void)
{
    return (Motor_Mode == TORQUE) || (Motor_Mode == SPEED) || (Motor_Mode == POSITION);
}

static float Encoder_Theta_e(void)
{
    return Angle_Wrap((float)Motor_Para.Pp * Motor_Run.Theta_m + Motor_Cal.Theta_Off);
}

static Motor_Fast_Mode_e Fast_Off_Run(float *Theta_e,
                                      float *Id_Ref,
                                      float *Iq_Ref,
                                      float *Ualpha,
                                      float *Ubeta)
{
    (void)Theta_e;
    (void)Id_Ref;
    (void)Iq_Ref;
    (void)Ualpha;
    (void)Ubeta;

    return FAST_OFF;
}

static Motor_Fast_Mode_e Servo_Fast_Run(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta)
{
    (void)Ualpha;
    (void)Ubeta;

    *Theta_e = Encoder_Theta_e();
    *Id_Ref = Current_Ref.Id;
    *Iq_Ref = Current_Ref.Iq;

    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Open_Fast_Run(float *Theta_e,
                                       float *Id_Ref,
                                       float *Iq_Ref,
                                       float *Ualpha,
                                       float *Ubeta)
{
    (void)Ualpha;
    (void)Ubeta;

    if (Motor_State != RUN)
    {
        return FAST_OFF;
    }

    Open_Loop(We_Ref, Theta_e, Id_Ref, Iq_Ref);
    return FAST_CURRENT;
}

static Motor_Fast_Mode_e Ident_Fast_Run(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta)
{
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
}

static Motor_Fast_Mode_e Sensorless_Fast_Run(float *Theta_e,
                                             float *Id_Ref,
                                             float *Iq_Ref,
                                             float *Ualpha,
                                             float *Ubeta)
{
    (void)Ualpha;
    (void)Ubeta;

    if (Motor_State != RUN)
    {
        return FAST_OFF;
    }

    (void)Sensorless_Run(ADC.Ia_A, ADC.Ib_A, We_Ref, Theta_e, Id_Ref, Iq_Ref);
    return FAST_CURRENT;
}

static void Fast_Path_Bind(void)
{
    switch (Motor_Mode)
    {
        case TORQUE:
        case SPEED:
        case POSITION:
            Fast_Run = Servo_Fast_Run;
            break;

        case OPEN_LOOP:
            Fast_Run = Open_Fast_Run;
            break;

        case IDENT:
            Fast_Run = Ident_Fast_Run;
            break;

        case SENSORLESS_SPEED:
            Fast_Run = Sensorless_Fast_Run;
            break;

        default:
            Fast_Run = Fast_Off_Run;
            break;
    }
}

void Motor_Control(void)
{
    float Kt;
    float Te_Ref;
    float Wm_Target;
    float We_Fbk;
    float Iq_Min;
    float Iq_Max;
    Motor_Limit_T Lim;

    if (Motor_State == DISABLED)
    {
        Current_Ref.Id = 0.0f;
        Current_Ref.Iq = 0.0f;
        Wm_Ref = 0.0f;
        We_Ref = 0.0f;
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
        if (Motor_State == RUN)
        {
            Wm_Target = Motor_Cmd.Wm_Target;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            Wm_Ref = Speed_Profile(Wm_Target, Wm_Ref, MOTION_ACC_RAD_S2, MOTION_DEC_RAD_S2);
        }
        else
        {
            Wm_Ref = 0.0f;
        }

        We_Ref = (float)Motor_Para.Pp * Wm_Ref;
    }

    if ((Motor_Mode == OPEN_LOOP) || (Motor_Mode == SENSORLESS_SPEED))
    {
        Current_Ref.Id = 0.0f;
        Current_Ref.Iq = 0.0f;
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

    We_Fbk = (float)Motor_Para.Pp * Motor_Run.Wm;

    switch (Motor_Mode)
    {
        case TORQUE:
            if (Motor_State == RUN)
            {
                Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
                Te_Ref = Motor_Cmd.Te_Target;
                Limit_Value(&Te_Ref, -Lim.Te_Max, Lim.Te_Max);
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
            if (Motor_State == RUN)
            {
                Pos_Ref_Turn = Motor_Cmd.Pos_Turn;
                Pos_Ref_Theta = Motor_Cmd.Pos_Theta;
            }

            if (Pos_Div == 0U)
            {
                Wm_Ref = Position_Loop(Pos_Ref_Turn, Pos_Ref_Theta, -Lim.Wm_Max, Lim.Wm_Max);
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

    return Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);
}

Motor_State_e Motor_State_Get(void)
{
    return Motor_State;
}

Motor_Mode_e Motor_Mode_Get(void)
{
    return Motor_Mode;
}

void Motor_Enable(void)
{
    if (Motor_State != DISABLED)
    {
        return;
    }

    Fast_Path_Bind();

    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;

    if (Motion_Mode_Active())
    {
        Motion_State_Reset();
        Current_Loop_State_Reset();
    }

    Motor_Run.Ud = 0.0f;
    Motor_Run.Uq = 0.0f;
    Motor_Run.Ualpha = 0.0f;
    Motor_Run.Ubeta = 0.0f;

    if (Motion_Mode_Active())
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

    if (Motor_Mode == OPEN_LOOP)
    {
        Current_Loop_State_Reset();
        Open_Loop_Reset();
    }
    else if (Motor_Mode == IDENT)
    {
        if (!Identification_Start(Ident_Mode, Motor_Cmd.Wm_Target))
        {
            return;
        }
    }
    else if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_Begin();
    }

    if (!Motion_Mode_Active())
    {
        PWM_Enable();
    }

    Motor_State = RUN;
}

void Motor_Stop(void)
{
    if (Motor_State != RUN)
    {
        return;
    }

    if (Motor_Mode == OPEN_LOOP)
    {
        Open_Loop_Reset();
    }
    else if ((Motor_Mode == IDENT) && Identification_Active())
    {
        Identification_Abort();
    }
    else if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_Stop();
    }
    else if (Motor_Mode == POSITION)
    {
        Pos_Ref_Turn = Motor_Run.Turn;
        Pos_Ref_Theta = Motor_Run.Theta_m;
        Pos_Div = 0U;
    }

    Current_Ref.Id = 0.0f;

    if (Motor_Mode == TORQUE)
    {
        Current_Ref.Iq = 0.0f;
    }

    if (!Motion_Mode_Active())
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

    PWM_Disable();
    Fast_Run = Fast_Off_Run;
    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Motor_State = DISABLED;
}

void Motor_Mode_Set(Motor_Mode_e Mode)
{
    if ((Motor_State == DISABLED) && (Mode <= SENSORLESS_SPEED))
    {
        Motor_Mode = Mode;
    }
}

void Motor_Ident_Mode_Set(Ident_Mode_e Mode)
{
    if ((Motor_State == DISABLED) && (Mode >= IDENT_RS_LS) && (Mode <= IDENT_FLUX))
    {
        Ident_Mode = Mode;
    }
}

bool Motor_Ident_Apply(void)
{
    if ((Motor_State == RUN) || (Motor_Mode != IDENT))
    {
        return false;
    }

    return Identification_Apply();
}

bool User_I_Limit_Set(float I_Max)
{
    if ((Motor_State != DISABLED) || !__builtin_isfinite(I_Max) || (I_Max <= 0.0f) || (I_Max > Motor_Lim.I_Max))
    {
        return false;
    }

    User_Lim.I_Max = I_Max;
    return true;
}

bool Motor_Pp_Set(uint8_t Pp)
{
    if ((Motor_State != DISABLED) || (Pp == 0U))
    {
        return false;
    }

    Motor_Para.Pp = Pp;
    Motor_Para_Changed(MOTOR_PARA_PP);
    return true;
}

void Torque_Target_Set(float Te)
{
    Motor_Cmd.Te_Target = Te;
}

void Speed_Target_Set(float Wm)
{
    Motor_Cmd.Wm_Target = Wm;
}

void Position_Target_Set(int32_t Turn, float Theta)
{
    Motor_Cmd.Pos_Turn = Turn;
    Motor_Cmd.Pos_Theta = Theta;
}
