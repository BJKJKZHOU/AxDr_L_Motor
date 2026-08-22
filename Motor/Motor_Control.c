#include "Motor_Control.h"

#include "Math.h"
#include "Current_Loop.h"
#include "Motor_ADC.h"
#include "Motor_PWM.h"
#include "Motion_Loop.h"
#include "control_params.h"
#include "motor_params.h"
#include "tim.h"


typedef struct
{
    float Id;
    float Iq;

} Current_Ref_T;


Motor_Cal_T Motor_Cal = MOTOR_CAL_DEFAULT;
Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;
const Motor_Limit_T Motor_Lim = MOTOR_LIM_DEFAULT;
Motor_Limit_T User_Lim = USER_LIM_DEFAULT;
Motor_Run_T Motor_Run = {0};


static Motor_Cmd_T Motor_Cmd = {0};
static volatile Servo_State_e Servo_State = SERVO_DISABLED;
static Servo_State_e State_Pre = SERVO_DISABLED;
static Ctrl_Mode_e Ctrl_Mode = CTRL_TORQUE;

static Current_Ref_T Current_Ref = {0};
static float Wm_Ref = 0.0f;
static int32_t Pos_Ref_Turn = 0;
static float Pos_Ref_Theta = 0.0f;
static uint32_t Pos_Div = 0U;


static void Motor_Limit_Get(Motor_Limit_T *Lim)
{
    Lim->I_Max = (Motor_Lim.I_Max < User_Lim.I_Max)
               ? Motor_Lim.I_Max
               : User_Lim.I_Max;
    Lim->Te_Max = (Motor_Lim.Te_Max < User_Lim.Te_Max)
                ? Motor_Lim.Te_Max
                : User_Lim.Te_Max;
    Lim->Wm_Max = (Motor_Lim.Wm_Max < User_Lim.Wm_Max)
                ? Motor_Lim.Wm_Max
                : User_Lim.Wm_Max;

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


void Motor_Control(void)
{
    float Kt;
    float Te_Ref;
    float Iq_Min;
    float Iq_Max;
    Motor_Limit_T Lim;

    if (Servo_State != State_Pre)
    {
        switch (Servo_State)
        {
            case SERVO_DISABLED:
                Current_Ref.Id = 0.0f;
                Current_Ref.Iq = 0.0f;
                Wm_Ref = 0.0f;
                break;

            case SERVO_ENABLED:
                Speed_Ctrl.State.Int = 0.0f;
                Speed_Ctrl.State.Fbk_Pre = Motor_Run.Wm;

                switch (Ctrl_Mode)
                {
                    case CTRL_TORQUE:
                    case CTRL_OPEN_LOOP:
                        Current_Ref.Id = 0.0f;
                        Current_Ref.Iq = 0.0f;
                        break;

                    case CTRL_SPEED:
                        if (State_Pre == SERVO_DISABLED)
                        {
                            Wm_Ref = 0.0f;
                        }
                        break;

                    case CTRL_POSITION:
                        Pos_Ref_Turn = Motor_Run.Turn;
                        Pos_Ref_Theta = Motor_Run.Theta_m;
                        Pos_Div = 0U;
                        break;

                    default:
                        Current_Ref.Id = 0.0f;
                        Current_Ref.Iq = 0.0f;
                        break;
                }
                break;

            case SERVO_RUN:
            default:
                break;
        }

        State_Pre = Servo_State;
    }

    Current_Ref.Id = 0.0f;

    if (Servo_State == SERVO_DISABLED)
    {
        Current_Ref.Iq = 0.0f;
        return;
    }

    if (Ctrl_Mode == CTRL_OPEN_LOOP)
    {
        Current_Ref.Iq = 0.0f;
        return;
    }

    Motor_Limit_Get(&Lim);
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

    switch (Ctrl_Mode)
    {
        case CTRL_TORQUE:
            if (Servo_State == SERVO_ENABLED)
            {
                Current_Ref.Iq = 0.0f;
            }
            else
            {
                Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
                Te_Ref = Motor_Cmd.Te_Target;
                Limit_Value(&Te_Ref, -Lim.Te_Max, Lim.Te_Max);
                Current_Ref.Iq = Te_Ref / Kt;
            }
            break;

        case CTRL_SPEED:
            if (Servo_State == SERVO_ENABLED)
            {
                Wm_Ref = 0.0f;
            }
            else
            {
                Wm_Ref = Motor_Cmd.Wm_Target;
                Limit_Value(&Wm_Ref, -Lim.Wm_Max, Lim.Wm_Max);
            }

            Current_Ref.Iq = Speed_Loop(Wm_Ref, Iq_Min, Iq_Max);
            break;

        case CTRL_POSITION:
            if (Servo_State == SERVO_RUN)
            {
                Pos_Ref_Turn = Motor_Cmd.Pos_Turn;
                Pos_Ref_Theta = Motor_Cmd.Pos_Theta;
            }

            if (Pos_Div == 0U)
            {
                Wm_Ref = Position_Loop(Pos_Ref_Turn, Pos_Ref_Theta,
                                       -Lim.Wm_Max, Lim.Wm_Max);
            }

            Pos_Div ^= 1U;

            Current_Ref.Iq = Speed_Loop(Wm_Ref, Iq_Min, Iq_Max);
            break;

        case CTRL_OPEN_LOOP:
        default:
            Current_Ref.Iq = 0.0f;
            break;
    }

    Limit_Value(&Current_Ref.Iq, Iq_Min, Iq_Max);
}


void Current_Ref_Get(float *Id_Ref, float *Iq_Ref)
{
    *Id_Ref = Current_Ref.Id;
    *Iq_Ref = Current_Ref.Iq;
}


Servo_State_e Servo_State_Get(void)
{
    return Servo_State;
}


Ctrl_Mode_e Ctrl_Mode_Get(void)
{
    return Ctrl_Mode;
}


void Servo_Enable(void)
{
    if (Servo_State == SERVO_DISABLED)
    {
        Current_Ref.Id = 0.0f;
        Current_Ref.Iq = 0.0f;

        Id_Ctrl.State.Int = 0.0f;
        Iq_Ctrl.State.Int = 0.0f;

        Id_Ctrl.Sig.Out = 0.0f;
        Iq_Ctrl.Sig.Out = 0.0f;

        Motor_Run.Ud = 0.0f;
        Motor_Run.Uq = 0.0f;

        PWM_Enable();
        Servo_State = SERVO_ENABLED;
    }
}


void Servo_Run(void)
{
    if (Servo_State == SERVO_ENABLED)
    {
        Servo_State = SERVO_RUN;
    }
}


void Servo_Stop(void)
{
    if (Servo_State == SERVO_RUN)
    {
        Servo_State = SERVO_ENABLED;
    }
}


void Servo_Disable(void)
{
    PWM_Disable();
    Servo_State = SERVO_DISABLED;
}


void Ctrl_Mode_Set(Ctrl_Mode_e Mode)
{
    if (Servo_State == SERVO_DISABLED)
    {
        Ctrl_Mode = Mode;

        if (Mode == CTRL_OPEN_LOOP)
        {
            CLEAR_BIT(TIM1->DIER, TIM_DIER_CC4IE);
        }
        else
        {
            SET_BIT(TIM1->DIER, TIM_DIER_CC4IE);
        }
    }
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
