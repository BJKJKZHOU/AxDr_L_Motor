#include "IF_Start.h"

#include "Math.h"
#include "control_params.h"

static IF_State_e IF_State = IF_ACCEL;
static int8_t IF_Dir = 1;
static uint32_t IF_Hold_Cnt = 0U;
static float IF_Theta_e = 0.0f;
static float IF_We_Abs = 0.0f;
static float IF_We_Target = IF_WE_TARGET_RAD_S;

void IF_Start_Reset(float Theta_Start, int8_t Dir)
{
    IF_State = IF_ACCEL;
    IF_Dir = (Dir >= 0) ? 1 : -1;
    IF_Hold_Cnt = 0U;
    IF_Theta_e = Angle_Wrap(Theta_Start);
    IF_We_Abs = 0.0f;
    IF_We_Target = IF_WE_TARGET_RAD_S;
}

void IF_Start_Target_Set(float We_Target)
{
    if (We_Target < 0.0f)
    {
        We_Target = -We_Target;
    }

    IF_We_Target = We_Target;
    IF_Hold_Cnt = 0U;

    if (IF_We_Abs == IF_We_Target)
    {
        IF_State = IF_HOLD;
    }
    else
    {
        IF_State = IF_ACCEL;
    }
}

bool IF_Start_Run(float *Theta_e, float *Id_Ref, float *Iq_Ref)
{
    float Ratio;
    float Iq_Abs;
    float We_Step;

    Ratio = IF_We_Abs / IF_WE_TARGET_RAD_S;

    if (Ratio > 1.0f)
    {
        Ratio = 1.0f;
    }

    Iq_Abs = IF_IQ_START_A + (IF_IQ_TARGET_A - IF_IQ_START_A) * Ratio;

    *Theta_e = IF_Theta_e;
    *Id_Ref = 0.0f;
    *Iq_Ref = (float)IF_Dir * Iq_Abs;

    if (IF_State == IF_ACCEL)
    {
        We_Step = IF_ACC_RAD_S2 * CUR_TS;

        if (IF_We_Abs < IF_We_Target)
        {
            IF_We_Abs += We_Step;

            if (IF_We_Abs >= IF_We_Target)
            {
                IF_We_Abs = IF_We_Target;
                IF_Hold_Cnt = 0U;
                IF_State = IF_HOLD;
            }
        }
        else
        {
            IF_We_Abs -= We_Step;

            if (IF_We_Abs <= IF_We_Target)
            {
                IF_We_Abs = IF_We_Target;
                IF_Hold_Cnt = 0U;
                IF_State = IF_HOLD;
            }
        }
    }
    else if (IF_Hold_Cnt < IF_HOLD_CNT)
    {
        IF_Hold_Cnt++;
    }

    IF_Theta_e += (float)IF_Dir * IF_We_Abs * CUR_TS;
    IF_Theta_e = Angle_Wrap(IF_Theta_e);

    return (IF_State == IF_HOLD) && (IF_Hold_Cnt >= IF_HOLD_CNT);
}

IF_State_e IF_Start_State_Get(void)
{
    return IF_State;
}

float IF_Start_We_Get(void)
{
    return (float)IF_Dir * IF_We_Abs;
}
