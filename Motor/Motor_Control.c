#include "Motor_Control.h"

#include "motor_params.h"


typedef struct
{
    float Id;
    float Iq;

} Current_Ref_T;


Motor_Para_T Motor_Para = MOTOR_PARA_DEFAULT;
Motor_Run_T Motor_Run = {0};


static Motor_Cmd_T Motor_Cmd = {0};
static Servo_State_e Servo_State = SERVO_DISABLED;
static Servo_State_e State_Pre = SERVO_DISABLED;
static Ctrl_Mode_e Ctrl_Mode = CTRL_TORQUE;

static Current_Ref_T Current_Ref = {0};
static float Wm_Ref = 0.0f;
static int32_t Pos_Ref_Turn = 0;
static float Pos_Ref_Theta = 0.0f;


void Motor_Control(void)
{
    float Kt;

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
                switch (Ctrl_Mode)
                {
                    case CTRL_TORQUE:
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
                Current_Ref.Iq = Motor_Cmd.Te_Target / Kt;
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
            }

            /* Speed loop will generate Iq when implemented. */
            Current_Ref.Iq = 0.0f;
            break;

        case CTRL_POSITION:
            /*
             * ENABLED keeps the position captured on entry.
             * RUN will update Pos_Ref through the position trajectory when implemented.
             */
            (void)Pos_Ref_Turn;
            (void)Pos_Ref_Theta;
            Current_Ref.Iq = 0.0f;
            break;

        default:
            Current_Ref.Iq = 0.0f;
            break;
    }
}


void Servo_Enable(void)
{
    if (Servo_State == SERVO_DISABLED)
    {
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
    Servo_State = SERVO_DISABLED;
}


void Ctrl_Mode_Set(Ctrl_Mode_e Mode)
{
    if (Servo_State == SERVO_DISABLED)
    {
        Ctrl_Mode = Mode;
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
