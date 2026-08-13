#include "PID.h"


void PID_Run(PID_T *Pid, float Ts)
{
    float P;
    float D;
    float Out;

    Pid->Sig.Err = Pid->Sig.Ref - Pid->Sig.Fbk;

    P = Pid->Para.Kp * Pid->Sig.Err;

    Pid->State.Int +=
        Pid->Para.Ki * Pid->Sig.Err * Ts;

    if (Pid->State.Int > Pid->Para.Int_Max)
    {
        Pid->State.Int = Pid->Para.Int_Max;
    }
    else if (Pid->State.Int < Pid->Para.Int_Min)
    {
        Pid->State.Int = Pid->Para.Int_Min;
    }

    D = -Pid->Para.Kd
      * (Pid->Sig.Fbk - Pid->State.Fbk_Pre)
      / Ts;

    Out = P + Pid->State.Int + D;

    if (Out > Pid->Para.Out_Max)
    {
        Out = Pid->Para.Out_Max;
    }
    else if (Out < Pid->Para.Out_Min)
    {
        Out = Pid->Para.Out_Min;
    }

    Pid->State.Fbk_Pre = Pid->Sig.Fbk;
    Pid->Sig.Out = Out;
}