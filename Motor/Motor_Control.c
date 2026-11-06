/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motor_Control.h"

#include "main.h"

#include "Math.h"
#include "Current_Loop.h"
#include "Encoder.h"
#include "Enc_Cal.h"
#include "Enc_Control.h"
#include "Flux.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Para.h"
#include "Motor_PWM.h"
#include "motor_work.h"
#include "Motion_Loop.h"
#include "Open_Loop.h"
#include "Protection.h"
#include "Ramp.h"
#include "SCurve.h"
#include "Sensorless.h"
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
volatile Motor_Frf_Speed_T Motor_Frf_Speed = { 0 };

volatile float Motor_Plot_Wm = 0.0f;
volatile float Motor_Plot_Wm_Ref = 0.0f;
volatile float Motor_Plot_Position = 0.0f;
volatile float Motor_Plot_Position_Ref = 0.0f;

static volatile Motor_State_e Motor_State = DISABLED;

/* The 2 kHz loop owns these references. The first VF-to-FOC ADC cycle
 * seeds them from fresh dq feedback until the next speed-loop update. */
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

static void Iq_Limit_Calc(float I_Max, float *Iq_Min, float *Iq_Max)
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

    We = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;
    U_Lim = ADC.Vbus_V * INV_SQRT3_F * VOLT_MOD_MAX;

    Wlq = We * Motor_Para.Lq;
    Bemf = We * Motor_Para.Flux;

    A = Motor_Para.Rs * Motor_Para.Rs + Wlq * Wlq;
    B = 2.0f * Motor_Para.Rs * Bemf;
    C = Bemf * Bemf - U_Lim * U_Lim;

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

    if (*Iq_Min < -I_Max)
    {
        *Iq_Min = -I_Max;
    }
    if (*Iq_Max > I_Max)
    {
        *Iq_Max = I_Max;
    }
    if (*Iq_Min > *Iq_Max)
    {
        *Iq_Min = 0.0f;
        *Iq_Max = 0.0f;
    }
}

/* Resume the speed trajectory from VF feedback. The first DECEL ADC cycle
 * seeds the current controller from that cycle's voltage and current. */
void Motor_Cal_Decel_Begin(void)
{
    float Wm = Mechanical_ESO.State.Wm;
    float Iq = Motor_Run.Iq;
    float Iq_Min;
    float Iq_Max;

    Motion_Ref.Wm = Wm;
    Motion_Ref.Am = 0.0f;
    SCurve = (SCurve_T){ 0 };
    Wm_Ref = Wm;
    We_Ref = (float)Motor_Para.Pp * Wm;
    Current_Ref.Id = Motor_Run.Id;
    Current_Ref.Iq = Iq;

    Iq_Limit_Calc(User_Lim.I_Max, &Iq_Min, &Iq_Max);
    Speed_Loop_Track(We_Ref, We_Ref, Iq, Iq_Min, Iq_Max);
}

static FAST_CODE bool Servo_Mode(void)
{
    return (Motor_Mode == TORQUE) || (Motor_Mode == SPEED) || (Motor_Mode == POSITION);
}

/* References shared by the non-servo stop and power-stage Disable paths. */
static void Motor_Refs_Clear(void)
{
    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;
    Wm_Ref = 0.0f;
    We_Ref = 0.0f;
    Sensorless_We_Target = 0.0f;
    Sensorless_Profile_Active = false;
    Sensorless_Stop_Delay_Cnt = 0U;
}

static void NonServo_Enabled_Apply(void)
{
    if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_State = SL_IDLE;
    }

    Stop_Pending = false;
    Motor_Refs_Clear();
    Motor_Run.Ud = 0.0f;
    Motor_Run.Uq = 0.0f;
    Motor_Run.Ualpha = 0.0f;
    Motor_Run.Ubeta = 0.0f;
    PWM_Disable();
    Motor_State = ENABLED;
}

void Motor_Disable_Apply(void)
{
    Stop_Pending = false;
    if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_State = SL_IDLE;
    }

    PWM_Disable();
    Motor_Refs_Clear();
    Motor_State = DISABLED;
}

void Motor_Control(void)
{
    bool Cal_Speed;
    Motor_Mode_e Mode;
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
    float Speed_Inj;
    uint32_t Primask;
    Motor_Limit_T Lim;

    Cal_Speed = (Enc_Step == ENC_CAL_RUN) &&
                ((Enc_Cal.Step == CAL_SPEED) || (Enc_Cal.Step == CAL_DECEL));
    /* 动作只推进流程；回零继续执行 POSITION，校准速度阶段在闭环更新后推进。 */
    if (Enc_Control_Active() && !Cal_Speed)
    {
        Enc_Control_Run();
    }

    if (Motor_State == DISABLED)
    {
        return;
    }

    Mode = Cal_Speed ? SPEED : Motor_Mode;

    if (Mode == ENCODER_CAL)
    {
        return;
    }

    if (Mode == IDENT)
    {
        if (Motor_State == RUN)
        {
            Identification_Control();

            if ((Ident_State != IDENT_RUNNING) && (Ident_State != IDENT_FINISH))
            {
                NonServo_Enabled_Apply();
            }
        }

        return;
    }

    Lim.I_Max = User_Lim.I_Max;
    Lim.Wm_Max = User_Lim.Wm_Max;

    switch (Mode)
    {
        case OPEN_LOOP:
            Wm_Target = (Motor_State == RUN) ?
                        Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            if (Open_Loop_Method == OPEN_LOOP_IF)
            {
                Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
                Wm_Target = Motion_Ref.Wm;
            }
            Wm_Ref = Wm_Target;
            We_Ref = (float)Motor_Para.Pp * Wm_Target;
            return;

        case SENSORLESS_SPEED:
            if (Sensorless_State == SL_FAILED)
            {
                NonServo_Enabled_Apply();
                return;
            }

            Wm_Target = ((Motor_State == RUN) && !Stop_Pending) ?
                        Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f;
            Limit_Value(&Wm_Target, -Lim.Wm_Max, Lim.Wm_Max);
            Sensorless_We_Target = (float)Motor_Para.Pp * Wm_Target;

            if (Sensorless_Speed_Control_Active())
            {
                if (!Sensorless_Profile_Active)
                {
                    Wm_Ref = Sensorless_Wm_Get();
                    Motion_Ref.Wm = Wm_Ref;
                    Motion_Ref.Am = 0.0f;
                    Sensorless_Profile_Active = true;
                }
                Ramp_Run(&Motion_Ref, Wm_Target, Motion_Config.Wm_Acc, Motion_Config.Wm_Dec, SPD_TS);
                Wm_Ref = Motion_Ref.Wm;
            }
            else
            {
                Sensorless_Profile_Active = false;
                Wm_Ref = Sensorless_Wm_Get();
            }

            We_Ref = (float)Motor_Para.Pp * Wm_Ref;

            if (Stop_Pending && (Wm_Ref == 0.0f))
            {
                Sensorless_Stop_Delay_Cnt++;

                if (Sensorless_Stop_Delay_Cnt >= SENSORLESS_STOP_DELAY_TICKS)
                {
                    NonServo_Enabled_Apply();
                }
            }
            else
            {
                Sensorless_Stop_Delay_Cnt = 0U;
            }

            if (Motor_State == RUN)
            {
                Sensorless_Control(We_Ref, -Lim.I_Max, Lim.I_Max);
            }
            return;

        default:
            break;
    }

    Current_Ref.Id = 0.0f;

    Iq_Limit_Calc(Lim.I_Max, &Iq_Min, &Iq_Max);

    We_Fbk = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;

    switch (Mode)
    {
        case TORQUE:
            Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
            if (Motor_State == RUN)
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
            Wm_Target = Cal_Speed ?
                        ((Enc_Cal.Step == CAL_DECEL) ? 0.0f : ENC_CAL_WM) :
                        (((Motor_State == RUN) && !Stop_Pending) ?
                         Motor_User_To_Internal(Motor_Cmd.Wm_Target) : 0.0f);
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

            /* Stop keeps RUN while decelerating, so a later FRF Start can
             * still pass the Action gate. Cancel it before adding excitation. */
            Speed_Inj = 0.0f;
            if ((Signal_Injection.State.Active != 0U) &&
                (Signal_Injection.State.Target == SIGNAL_SPEED))
            {
                if (Stop_Pending)
                {
                    Signal_Injection_Stop();
                }
                else
                {
                    Speed_Inj = (float)Motor_Para.Pp * Signal_Injection_Run();
                    We_Ref += Speed_Inj;
                }
            }

            if ((Motor_Mode == SPEED) && Stop_Pending && (Motion_Ref.Wm == 0.0f) &&
                ((Motion_Config.Profile == MOTION_TRAPEZOID) || (SCurve.Count == 0U)))
            {
                Stop_Pending = false;
                Motor_State = ENABLED;
            }

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
            break;

        default:
            Current_Ref.Iq = 0.0f;
            goto finish;
    }

    if (!Cal_Speed && (Speed_Controller == SPEED_CTRL_LADRC))
    {
        Current_Ref.Iq = Speed_LADRC(We_Ref, Iq_Min, Iq_Max);
    }
    else
    {
        Current_Ref.Iq = Speed_Loop(We_Ref, We_Fbk, Iq_Min, Iq_Max);
    }

    if (Mode == SPEED)
    {
        /* 2 kHz 的四个观测量一起发布，供 FAST Plot 读取。 */
        Primask = __get_PRIMASK();
        __disable_irq();
        Motor_Frf_Speed.Ref = Speed_Ctrl.Sig.Ref;
        Motor_Frf_Speed.Fbk = Speed_Ctrl.Sig.Fbk;
        Motor_Frf_Speed.Iq = Speed_Ctrl.Sig.Out;
        Motor_Frf_Speed.Inj = Speed_Inj;
        __set_PRIMASK(Primask);
    }

finish:
    /* The handoff ADC cycle can reseed Current_Ref after Speed_Loop returns. */
    Limit_Value(&Current_Ref.Iq, Iq_Min, Iq_Max);

    if (Cal_Speed)
    {
        /* Preserve motion update -> experiment transition -> next ADC handoff. */
        Enc_Control_Run();
    }
}

Motor_Fast_Mode_e Motor_Fast_Run(float Ialpha, float Ibeta, float Encoder_Theta_e,
                                 float *Theta_e,
                                 float *Id_Ref,
                                 float *Iq_Ref,
                                 float *Ualpha,
                                 float *Ubeta)
{
    bool Cal = (Enc_Step == ENC_CAL_RUN);

    /* 快环输出在分派前统一置零，分支只更新需要的分量。 */
    *Theta_e = 0.0f;
    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;
    *Ualpha = 0.0f;
    *Ubeta = 0.0f;

    if ((Motor_State == DISABLED) || ((Motor_Mode == ENCODER_CAL) && (Motor_State != RUN)))
    {
        return FAST_OFF;
    }

    /* ENCODER_CAL 接管快环，速度升降阶段复用有感闭环；回零沿用 POSITION。 */
    if ((Motor_Mode == ENCODER_CAL) &&
        (!Cal || ((Enc_Cal.Step != CAL_SPEED) && (Enc_Cal.Step != CAL_DECEL))))
    {
        return Enc_Control_Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);
    }

    /* Sensored modes retain current control while ENABLED for servo holding. */
    if (Cal || Servo_Mode())
    {
        bool Handoff = Cal && (Enc_Cal.Step == CAL_DECEL) && Enc_Cal_Handoff_Ready;

        if (Handoff)
        {
            /* ADC has published this sample in the encoder frame. Keep VF
             * rotating once more; ADC tracks that voltage before running FOC. */
            (void)Enc_Cal_Fast_Run(Theta_e, Ualpha, Ubeta);
            Current_Ref.Id = Motor_Run.Id;
            Current_Ref.Iq = Motor_Run.Iq;
        }

        *Theta_e = Encoder_Theta_e;
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
            return Open_Loop(We_Ref, Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);

        case IDENT:
            return Identification_Fast_Run(Ialpha,
                                           Ibeta,
                                           Theta_e,
                                           Id_Ref,
                                           Iq_Ref,
                                           Ualpha,
                                           Ubeta);

        case SENSORLESS_SPEED:
            Sensorless_Run(Ialpha, Ibeta, Sensorless_We_Target, Theta_e, Id_Ref, Iq_Ref);
            return FAST_CURRENT;

        default:
            return FAST_OFF;
    }
}

Motor_State_e Motor_State_Get(void)
{
    return Motor_State;
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
    return Servo_Mode() || (Motor_Mode == ENCODER_CAL);
}

Motor_Status_e Motor_Enable(void)
{
    bool Servo = (Enc_Step == ENC_CAL_RUN) || Servo_Mode();

    if ((Motor_State != DISABLED) || (Protection.Fault != 0U) ||
        ((Protection.Warning & PROT_UNDERVOLTAGE) != 0U) ||
        (Motor_Work_State != MOTOR_WORK_IDLE))
    {
        return MOTOR_ERR_STATE;
    }

    Stop_Pending = false;

    if (Motor_Encoder_Required() && ((Encoder.Ready == 0U) || (Encoder.Fault != 0U)))
    {
        return MOTOR_ERR_CONFIG;
    }

    if (Servo && (Motor_Cal.Valid == 0U))
    {
        return MOTOR_ERR_CONFIG;
    }

    Current_Ref.Id = 0.0f;
    Current_Ref.Iq = 0.0f;

    if (Servo)
    {
        int32_t Turn;
        float Theta;

        if (Mechanical_ESO.Para.Valid == 0U)
        {
            return MOTOR_ERR_CONFIG;
        }

        Speed_Ctrl.State.Int = 0.0f;
        Speed_Ctrl.State.Fbk_Pre = (float)Motor_Para.Pp * Mechanical_ESO.State.Wm;
        Wm_Ref = 0.0f;
        We_Ref = 0.0f;
        Encoder_Position_Read(&Turn, &Theta);
        Motion_Ref = (Motion_Ref_T){ .Turn = Turn, .Theta = Theta };
        SCurve = (SCurve_T){ 0 };
        Pos_Div = 0U;
        Current_Loop_State_Reset();
    }

    Motor_Run.Ud = 0.0f;
    Motor_Run.Uq = 0.0f;
    Motor_Run.Ualpha = 0.0f;
    Motor_Run.Ubeta = 0.0f;

    if (Servo && !PWM_Enable())
    {
        return MOTOR_ERR_CONFIG;
    }
    Motor_State = ENABLED;
    return MOTOR_OK;
}

Motor_Status_e Motor_Start(void)
{
    bool Enc = Enc_Control_Active();
    bool Cal = (Enc_Step == ENC_CAL_RUN);

    if ((Motor_State != ENABLED) || (Protection.Fault != 0U) ||
        ((Protection.Warning & PROT_UNDERVOLTAGE) != 0U))
    {
        return MOTOR_ERR_STATE;
    }

    Signal_Injection_Stop();

    /* 辨识、寻相和校准由专用命令启动。 */
    if (((Motor_Mode == IDENT) || (Motor_Mode == ENCODER_CAL)) && !Enc)
    {
        return MOTOR_ERR_STATE;
    }

    if (Enc_Step == ENC_PHASE_RUN)
    {
        Current_Loop_State_Reset();
    }
    else if (Motor_Mode == OPEN_LOOP)
    {
        Current_Loop_State_Reset();
        Open_Loop_Reset();
    }
    else if (Motor_Mode == SENSORLESS_SPEED)
    {
        Sensorless_Profile_Active = false;
        Sensorless_We_Target = 0.0f;
        Motion_Ref.Wm = 0.0f;
        Motion_Ref.Am = 0.0f;
        Wm_Ref = 0.0f;
        We_Ref = 0.0f;

        if (!Sensorless_Begin())
        {
            return MOTOR_ERR_CONFIG;
        }
    }

    if (!Servo_Mode() && !Cal)
    {
        if (!PWM_Enable())
        {
            Motor_Disable();
            return MOTOR_ERR_CONFIG;
        }
    }

    Stop_Pending = false;
    Sensorless_Stop_Delay_Cnt = 0U;
    Motor_State = RUN;
    return MOTOR_OK;
}

Motor_Status_e Motor_Enc_Start(Enc_Mode_e Mode)
{
    Motor_Status_e Status;

    if ((Motor_State != DISABLED) ||
        (Motor_Mode != ((Mode == ENC_HOME) ? POSITION : ENCODER_CAL)) ||
        (Protection.Fault != 0U) || ((Protection.Warning & PROT_UNDERVOLTAGE) != 0U) ||
        (Motor_Work_State != MOTOR_WORK_IDLE) ||
        Enc_Control_Active())
    {
        return MOTOR_ERR_STATE;
    }
    if (!Enc_Control_Start(Mode))
    {
        return MOTOR_ERR_CONFIG;
    }

    Status = Motor_Enable();
    if (Status == MOTOR_OK)
    {
        Status = Motor_Start();
    }
    if (Status != MOTOR_OK)
    {
        Enc_Control_Abort();
        Motor_Disable_Apply();
    }
    return Status;
}

void Motor_Stop(void)
{
    /* Stop cancels test excitation even when the servo is holding ENABLED. */
    Signal_Injection_Stop();

    if (Enc_Control_Active())
    {
        Enc_Control_Abort();
        Motor_Disable_Apply();
        return;
    }

    if (Motor_State != RUN)
    {
        return;
    }

    if ((Motor_Mode == TORQUE) || (Motor_Mode == SPEED) ||
        (Motor_Mode == POSITION) || (Motor_Mode == SENSORLESS_SPEED))
    {
        if (Motor_Mode == SENSORLESS_SPEED)
        {
            Sensorless_Stop_Requested = true;
            Sensorless_Stop_Delay_Cnt = 0U;
        }

        Stop_Pending = true;
        return;
    }

    if (Motor_Mode == IDENT)
    {
        Identification_Stop();
    }
    NonServo_Enabled_Apply();
}

void Motor_Disable(void)
{
    Signal_Injection_Stop();

    if (Enc_Control_Active())
    {
        Enc_Control_Abort();
    }
    else if ((Motor_State == RUN) && ((Motor_Mode == OPEN_LOOP) || (Motor_Mode == IDENT)))
    {
        if (Motor_Mode == IDENT)
        {
            Identification_Stop();
        }

        Motor_Run.Ud = 0.0f;
        Motor_Run.Uq = 0.0f;
        Motor_Run.Ualpha = 0.0f;
        Motor_Run.Ubeta = 0.0f;
    }
    Motor_Disable_Apply();
}

Motor_Status_e Motor_Signal_Start(void)
{
    if (Enc_Control_Active() || (Protection.Fault != 0U) ||
        ((Protection.Warning & PROT_UNDERVOLTAGE) != 0U) ||
        ((Signal_Injection.Para.Target == SIGNAL_SPEED) ?
         ((Motor_Mode != SPEED) || ((Motor_State != ENABLED) && (Motor_State != RUN))) :
         ((Motor_Mode != TORQUE) || (Motor_State != ENABLED))))
    {
        return MOTOR_ERR_STATE;
    }
    if (!Signal_Injection_Start(User_Lim.I_Max, User_Lim.Wm_Max))
    {
        return MOTOR_ERR_CONFIG;
    }
    return MOTOR_OK;
}

Motor_Status_e Motor_Ident_Start(Ident_Mode_e Mode)
{
    if ((Motor_State != ENABLED) ||
        (Motor_Mode != IDENT) || (Protection.Fault != 0U) ||
        ((Protection.Warning & PROT_UNDERVOLTAGE) != 0U))
    {
        return MOTOR_ERR_STATE;
    }

    if (!Identification_Start(Mode, Motor_Cmd.Wm_Target))
    {
        return MOTOR_ERR_CONFIG;
    }

    if (!PWM_Enable())
    {
        Identification_Stop();
        return MOTOR_ERR_CONFIG;
    }
    Motor_State = RUN;
    return MOTOR_OK;
}

Motor_Status_e Motor_Ident_Apply(void)
{
    if ((Motor_State == RUN) || (Motor_Mode != IDENT) || (Ident_State != IDENT_DONE))
    {
        return MOTOR_ERR_STATE;
    }

    return Identification_Apply() ? MOTOR_OK : MOTOR_ERR_CONFIG;
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
        return Motor_Internal_To_User(Mechanical_ESO.State.Wm);
    }

    return 0.0f;
}

Motor_Position_T Motor_Position_Get(void)
{
    Motor_Position_T Position;
    int32_t Turn_Int;
    float Theta_Int;

    Encoder_Position_Read(&Turn_Int, &Theta_Int);
    Motor_Position_Internal_To_User(Turn_Int, Theta_Int, &Position.Turn, &Position.Theta);
    return Position;
}

bool Motor_Position_Plan_Done(void)
{
    Motor_Position_T Target;

    Motor_Position_User_To_Internal(Motor_Cmd.Position_Target.Turn,
                                    Motor_Cmd.Position_Target.Theta,
                                    &Target.Turn, &Target.Theta);
    return (Motion_Ref.Turn == Target.Turn) && (Motion_Ref.Theta == Target.Theta) &&
           (Motion_Ref.Wm == 0.0f) &&
           ((Motion_Config.Profile == MOTION_TRAPEZOID) || (SCurve.Count == 0U));
}

void Motor_Plot_Normal_Update(void)
{
    Motor_Position_T Ref;
    Motor_Position_T Position;

    Motor_Plot_Wm = Motor_Wm_Get();
    Motor_Plot_Wm_Ref = Motor_Wm_Ref_Get();

    Position = Motor_Position_Get();
    Motor_Plot_Position = (float)Position.Turn + Position.Theta / TWO_PI_F;

    Ref = Motor_Position_Ref_Get();
    Motor_Plot_Position_Ref = (float)Ref.Turn + Ref.Theta / TWO_PI_F;
}
