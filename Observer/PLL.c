#include "PLL.h"

#include "Math.h"
#include "Sin_LUT.h"


void PLL_Reset(PLL_T *Pll,
               float Theta,
               float We)
{
    Pll->State.Theta = Angle_Wrap(Theta);
    Pll->State.We = We;
    Pll->State.Err = 0.0f;
}


void PLL_Run(PLL_T *Pll,
             float X,
             float Y,
             float Mag_Ref,
             float Ts)
{
    float Sin;
    float Cos;

    if ((Mag_Ref <= 0.0f) || (Ts <= 0.0f))
    {
        Pll->State.Err = 0.0f;
        return;
    }

    SinCos(Pll->State.Theta, &Sin, &Cos);

    Pll->State.Err = (Y * Cos - X * Sin) / Mag_Ref;

    Pll->State.We += Pll->Para.Ki * Pll->State.Err * Ts;
    Pll->State.Theta +=
        (Pll->State.We + Pll->Para.Kp * Pll->State.Err) * Ts;
    Pll->State.Theta = Angle_Wrap(Pll->State.Theta);
}
