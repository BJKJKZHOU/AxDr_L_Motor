#include "Identification.h"

#include "Current_Loop.h"
#include "Motor_Type.h"
#include "Rs_Ls.h"


#define IDENT_I_MAX_A    2.0f


static volatile Ident_Mode_e Ident_Mode = IDENT_NONE;
static volatile Ident_State_e Ident_State = IDENT_IDLE;


static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}


bool Identification_Start(Ident_Mode_e Mode)
{
    if (Ident_State == IDENT_RUNNING)
    {
        return false;
    }

    if (Mode == IDENT_RS_LS)
    {
        Rs_Ls_Start();
    }
    else
    {
        return false;
    }

    Ident_Mode = Mode;
    Ident_State = IDENT_RUNNING;

    return true;
}


void Identification_Abort(void)
{
    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Reset();
    }

    Ident_Mode = IDENT_NONE;
    Ident_State = IDENT_IDLE;
}


void Identification_Update(void)
{
    const Rs_Ls_Result_T *Result;

    if (Ident_State != IDENT_RUNNING)
    {
        return;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        if (Rs_Ls_Active())
        {
            return;
        }

        Result = Rs_Ls_Result_Get();
        Ident_State = Result->Valid ? IDENT_DONE : IDENT_FAILED;
    }
}


bool Identification_Apply(void)
{
    const Rs_Ls_Result_T *Result;

    if ((Ident_Mode != IDENT_RS_LS) || (Ident_State != IDENT_DONE))
    {
        return false;
    }

    Result = Rs_Ls_Result_Get();

    if (!Result->Valid)
    {
        return false;
    }

    Motor_Para.Rs = Result->Rs_Ohm;
    Motor_Para.Ld = Result->Ls_H;
    Motor_Para.Lq = Result->Ls_H;
    Current_Loop_Para_Update();

    return true;
}


bool Identification_Active(void)
{
    return Ident_State == IDENT_RUNNING;
}


void Identification_Fast_Run(float Ia_A,
                             float Ib_A,
                             float Ic_A,
                             float *Ualpha_V,
                             float *Ubeta_V)
{
    *Ualpha_V = 0.0f;
    *Ubeta_V = 0.0f;

    if (Ident_State != IDENT_RUNNING)
    {
        return;
    }

    if ((Abs_Value(Ia_A) > IDENT_I_MAX_A) ||
        (Abs_Value(Ib_A) > IDENT_I_MAX_A) ||
        (Abs_Value(Ic_A) > IDENT_I_MAX_A))
    {
        if (Ident_Mode == IDENT_RS_LS)
        {
            Rs_Ls_Fail();
        }

        return;
    }

    if (Ident_Mode == IDENT_RS_LS)
    {
        Rs_Ls_Run(Ia_A, Ualpha_V, Ubeta_V);
    }
}


Ident_Mode_e Identification_Mode_Get(void)
{
    return Ident_Mode;
}


Ident_State_e Identification_State_Get(void)
{
    return Ident_State;
}


uint8_t Identification_Stage_Get(void)
{
    if (Ident_Mode == IDENT_RS_LS)
    {
        return (uint8_t)Rs_Ls_State_Get();
    }

    return 0U;
}
