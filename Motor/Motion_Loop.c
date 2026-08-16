#include "Motion_Loop.h"

#include "Math.h"
#include "Motor_Control.h"
#include "control_params.h"


PID_T Pos_Ctrl = POSITION_CTRL_DEFAULT;
PID_T Speed_Ctrl = SPEED_CTRL_DEFAULT;


float Position_Loop(int32_t Turn_Ref, float Theta_Ref,
                    float Wm_Min, float Wm_Max)
{
    int32_t Turn_Err;

    /* Calculate the turn difference before converting to float. */
    Turn_Err = Turn_Ref - Motor_Run.Turn;
    Pos_Ctrl.Sig.Ref = (float)Turn_Err * TWO_PI_F + Theta_Ref;
    Pos_Ctrl.Sig.Fbk = Motor_Run.Theta_m;

    Pos_Ctrl.Para.Out_Min = Wm_Min;
    Pos_Ctrl.Para.Out_Max = Wm_Max;

    PID_Run(&Pos_Ctrl, SPD_TS);

    return Pos_Ctrl.Sig.Out;
}


float Speed_Loop(float Wm_Ref, float Iq_Min, float Iq_Max)
{
    Speed_Ctrl.Sig.Ref = Wm_Ref;
    Speed_Ctrl.Sig.Fbk = Motor_Run.Wm;

    Speed_Ctrl.Para.Out_Min = Iq_Min;
    Speed_Ctrl.Para.Out_Max = Iq_Max;
    Speed_Ctrl.Para.Int_Min = Iq_Min;
    Speed_Ctrl.Para.Int_Max = Iq_Max;

    PID_Run(&Speed_Ctrl, SPD_TS);

    return Speed_Ctrl.Sig.Out;
}
