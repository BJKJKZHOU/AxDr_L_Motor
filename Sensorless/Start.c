#include "Start.h"

#include "Align.h"
#include "Current_Loop.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motor_Type.h"
#include "control_params.h"


static Sensorless_Start_State_e Start_State = SENSORLESS_ALIGN;
static int8_t Start_Dir = 1;
static volatile bool Start_Active = false;
static volatile bool Start_Ready = false;


void Sensorless_Start_Begin(int8_t Dir)
{
    Start_State = SENSORLESS_ALIGN;
    Start_Dir = (Dir >= 0) ? 1 : -1;
    Start_Ready = false;

    Align_Reset();
    Current_Loop_State_Reset();
    Start_Active = true;
}


void Sensorless_Start_Stop(void)
{
    Start_Active = false;
    Start_Ready = false;
}


bool Sensorless_Start_Active(void)
{
    return Start_Active;
}


bool Sensorless_Start_Ready(void)
{
    return Start_Ready;
}


bool Sensorless_Start_Run(float *Id_Ref, float *Iq_Ref)
{
    if (!Start_Active)
    {
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return false;
    }

    if (Start_State == SENSORLESS_ALIGN)
    {
        Motor_Run.Theta_e = 0.0f;

        if (Align_Current(IF_ALIGN_ID_A,
                          IF_ALIGN_CNT,
                          Id_Ref,
                          Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Start_Reset(-(float)Start_Dir * (0.5f * PI_F), Start_Dir);
            Start_State = SENSORLESS_IF;
        }

        return false;
    }

    Start_Ready = IF_Start_Run(&Motor_Run.Theta_e, Id_Ref, Iq_Ref);
    return Start_Ready;
}


Sensorless_Start_State_e Sensorless_Start_State_Get(void)
{
    return Start_State;
}
