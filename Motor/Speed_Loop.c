#include "Speed_Loop.h"

#include "Motor_Control.h"
#include "control_params.h"


PID_T Speed_Ctrl = SPEED_CTRL_DEFAULT;


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
