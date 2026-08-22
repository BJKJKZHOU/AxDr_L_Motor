#include "Flux_Observer.h"

#include "Math.h"
#include "Sin_LUT.h"


void Flux_Observer_Reset(Flux_Observer_T *Obs,
                         float Theta_e,
                         float Ialpha,
                         float Ibeta)
{
    float Sin;
    float Cos;

    SinCos(Angle_Wrap(Theta_e), &Sin, &Cos);

    Obs->State.PsiAlpha = Obs->Para.Flux * Cos;
    Obs->State.PsiBeta = Obs->Para.Flux * Sin;

    Obs->State.LambdaAlpha = Obs->Para.Ls * Ialpha
                           + Obs->State.PsiAlpha;
    Obs->State.LambdaBeta = Obs->Para.Ls * Ibeta
                          + Obs->State.PsiBeta;

    Obs->State.Flux_Err = 0.0f;
}


void Flux_Observer_Run(Flux_Observer_T *Obs,
                       float Ualpha,
                       float Ubeta,
                       float Ialpha,
                       float Ibeta,
                       float Ts)
{
    float Corr;
    float LambdaAlpha_Dot;
    float LambdaBeta_Dot;

    Obs->State.PsiAlpha = Obs->State.LambdaAlpha
                        - Obs->Para.Ls * Ialpha;
    Obs->State.PsiBeta = Obs->State.LambdaBeta
                       - Obs->Para.Ls * Ibeta;

    Obs->State.Flux_Err = Obs->Para.Flux * Obs->Para.Flux
                        - Obs->State.PsiAlpha * Obs->State.PsiAlpha
                        - Obs->State.PsiBeta * Obs->State.PsiBeta;

    Corr = 0.5f * Obs->Para.Gamma * Obs->State.Flux_Err;

    LambdaAlpha_Dot = Ualpha
                    - Obs->Para.Rs * Ialpha
                    + Corr * Obs->State.PsiAlpha;
    LambdaBeta_Dot = Ubeta
                   - Obs->Para.Rs * Ibeta
                   + Corr * Obs->State.PsiBeta;

    Obs->State.LambdaAlpha += LambdaAlpha_Dot * Ts;
    Obs->State.LambdaBeta += LambdaBeta_Dot * Ts;

    Obs->State.PsiAlpha = Obs->State.LambdaAlpha
                        - Obs->Para.Ls * Ialpha;
    Obs->State.PsiBeta = Obs->State.LambdaBeta
                       - Obs->Para.Ls * Ibeta;
}
