#include "Open_Loop.h"

#include "Align.h"
#include "Math.h"
#include "Motor_Control.h"
#include "Motor_Type.h"
#include "control_params.h"

typedef enum
{
    OPEN_ALIGN = 0,
    OPEN_RUN,

} Open_State_e;

static Open_State_e Open_State = OPEN_ALIGN;
static float Theta_Open = 0.0f;

void Open_Loop_Reset(void)
{
    Open_State = OPEN_ALIGN;
    Theta_Open = 0.0f;
    Align_Reset();
}

void Open_Loop(float *Id_Ref, float *Iq_Ref)
{
    if ((Motor_State_Get() != RUN) || (Motor_Mode_Get() != OPEN_LOOP))
    {
        Open_Loop_Reset();

        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        Motor_Run.Theta_e = Theta_Open;
        return;
    }

    if (Open_State == OPEN_ALIGN)
    {
        Theta_Open = 0.0f;

        if (Align_Current(OPEN_ALIGN_ID_A, OPEN_ALIGN_CNT, Id_Ref, Iq_Ref))
        {
            Open_State = OPEN_RUN;
        }
    }
    else
    {
        Theta_Open += OPEN_WE_RAD_S * CUR_TS;
        Theta_Open = Angle_Wrap(Theta_Open);

        *Id_Ref = 0.0f;
        *Iq_Ref = OPEN_IQ_A;
    }

    Motor_Run.Theta_e = Theta_Open;
}
