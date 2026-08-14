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
static Ctrl_Mode_e Ctrl_Mode = CTRL_TORQUE;
static Current_Ref_T Current_Ref = {0};


void Motor_Control(void)
{
    float Kt;

    Current_Ref.Id = 0.0f;

    switch (Servo_State)
    {
        case SERVO_DISABLED:
        case SERVO_ENABLED:
            Current_Ref.Iq = 0.0f;
            break;

        case SERVO_RUN:
            switch (Ctrl_Mode)
            {
                case CTRL_TORQUE:
                    Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
                    Current_Ref.Iq = Motor_Cmd.Te_Target / Kt;
                    break;

                case CTRL_SPEED:
                case CTRL_POSITION:
                default:
                    Current_Ref.Iq = 0.0f;
                    break;
            }
            break;

        default:
            Current_Ref.Iq = 0.0f;
            break;
    }
}
