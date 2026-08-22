#ifndef PLL_H
#define PLL_H


typedef struct
{
    float Kp;
    float Ki;

} PLL_Para_T;


typedef struct
{
    float Theta;
    float We;
    float Err;

} PLL_State_T;


typedef struct
{
    PLL_Para_T Para;
    PLL_State_T State;

} PLL_T;


void PLL_Reset(PLL_T *Pll,
               float Theta,
               float We);

void PLL_Run(PLL_T *Pll,
             float X,
             float Y,
             float Mag_Ref,
             float Ts);


#endif /* PLL_H */
