#include "Start.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Fast_Profile.h"
#include "Flux_Observer.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "Sin_LUT.h"
#include "control_params.h"
#include "main.h"


#define SENSORLESS_FLUX_OBS_BW_HZ      200.0f
#define SENSORLESS_PLL_BW_HZ            50.0f
#define SENSORLESS_PLL_DAMPING          0.707f
#define SENSORLESS_PLL_WN               (TWO_PI_F * SENSORLESS_PLL_BW_HZ)
#define SENSORLESS_PLL_KP               (2.0f * SENSORLESS_PLL_DAMPING * SENSORLESS_PLL_WN)
#define SENSORLESS_PLL_KI               (SENSORLESS_PLL_WN * SENSORLESS_PLL_WN)

#define SENSORLESS_OBS_WAIT_TIME_S      0.20f
#define SENSORLESS_OBS_WAIT_CNT         ((uint32_t)(SENSORLESS_OBS_WAIT_TIME_S / CUR_TS + 0.5f))
#define SENSORLESS_BLEND_TIME_S         0.15f
#define SENSORLESS_BLEND_CNT            ((uint32_t)(SENSORLESS_BLEND_TIME_S / CUR_TS + 0.5f))
#define SENSORLESS_ID_RAMP_A_S          5.0f
#define SENSORLESS_ID_RAMP_STEP_A       (SENSORLESS_ID_RAMP_A_S * CUR_TS)

#define SENSORLESS_OBS_WE_FILT_TAU_S    0.020f
#define SENSORLESS_OBS_WE_FILT_ALPHA    (CUR_TS / (SENSORLESS_OBS_WE_FILT_TAU_S + CUR_TS))
#define SENSORLESS_WE_ERR_MAX_RAD_S     5.0f
#define SENSORLESS_PLL_ERR_MAX          0.08f
#define SENSORLESS_FLUX_MIN_RATIO2      0.64f
#define SENSORLESS_FLUX_MAX_RATIO2      1.44f


static Sensorless_Start_State_e Start_State = SENSORLESS_ALIGN;
static int8_t Start_Dir = 1;
static volatile bool Start_Active = false;
static volatile bool Start_Ready = false;

Flux_Observer_T Flux_Obs = {0};
PLL_T Flux_PLL = {0};

volatile float Sensorless_Theta_IF = 0.0f;
volatile float Sensorless_Theta_Use = 0.0f;
volatile float Sensorless_Id_Ref = 0.0f;
volatile float Sensorless_Iq_Ref = 0.0f;
volatile float Sensorless_Blend = 0.0f;
volatile float Sensorless_We_Obs_F = 0.0f;

static bool Flux_Obs_U_Valid = false;
static bool Profile_Requested = false;
static uint32_t Obs_Wait_Cnt = 0U;
static uint32_t Blend_Cnt = 0U;
static float Obs_Id_Ref = 0.0f;
static float Obs_Iq_Ref = 0.0f;


static float Angle_Diff(float A, float B)
{
    float Diff;

    Diff = A - B;

    while (Diff > PI_F)
    {
        Diff -= TWO_PI_F;
    }

    while (Diff < -PI_F)
    {
        Diff += TWO_PI_F;
    }

    return Diff;
}


static float Abs_F(float X)
{
    return (X >= 0.0f) ? X : -X;
}


static bool Observer_Stable(void)
{
    float We_Err;
    float Flux2;
    float Flux_Ref2;

    Sensorless_We_Obs_F += SENSORLESS_OBS_WE_FILT_ALPHA *
                           (Flux_PLL.State.We - Sensorless_We_Obs_F);
    We_Err = Sensorless_We_Obs_F - IF_Start_We_Get();
    Flux2 = Flux_Obs.State.PsiAlpha * Flux_Obs.State.PsiAlpha
          + Flux_Obs.State.PsiBeta * Flux_Obs.State.PsiBeta;
    Flux_Ref2 = Flux_Obs.Para.Flux * Flux_Obs.Para.Flux;

    if (Flux_Ref2 <= 0.0f)
    {
        return false;
    }

    if (Abs_F(We_Err) > SENSORLESS_WE_ERR_MAX_RAD_S)
    {
        return false;
    }

    if (Abs_F(Flux_PLL.State.Err) > SENSORLESS_PLL_ERR_MAX)
    {
        return false;
    }

    if ((Flux2 < SENSORLESS_FLUX_MIN_RATIO2 * Flux_Ref2) ||
        (Flux2 > SENSORLESS_FLUX_MAX_RATIO2 * Flux_Ref2))
    {
        return false;
    }

    return true;
}


static void Current_Ref_Rotate(float Theta_IF,
                               float Theta_Use,
                               float Id_IF,
                               float Iq_IF,
                               float *Id_Ref,
                               float *Iq_Ref)
{
    float Diff;
    float Sin;
    float Cos;

    Diff = Angle_Diff(Theta_IF, Theta_Use);
    SinCos(Angle_Wrap(Diff), &Sin, &Cos);

    *Id_Ref = Id_IF * Cos - Iq_IF * Sin;
    *Iq_Ref = Id_IF * Sin + Iq_IF * Cos;
}


static float Ramp_Zero(float X, float Step)
{
    if (X > Step)
    {
        return X - Step;
    }

    if (X < -Step)
    {
        return X + Step;
    }

    return 0.0f;
}


void Sensorless_Start_Begin(int8_t Dir)
{
    Start_State = SENSORLESS_ALIGN;
    Start_Dir = (Dir >= 0) ? 1 : -1;
    Start_Ready = false;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;

    Sensorless_Theta_IF = 0.0f;
    Sensorless_Theta_Use = 0.0f;
    Sensorless_Id_Ref = 0.0f;
    Sensorless_Iq_Ref = 0.0f;
    Sensorless_Blend = 0.0f;
    Sensorless_We_Obs_F = 0.0f;

    Align_Reset();
    Current_Loop_State_Reset();
    Start_Active = true;
}


void Sensorless_Start_Stop(void)
{
    Start_Active = false;
    Start_Ready = false;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;
    Obs_Wait_Cnt = 0U;
    Blend_Cnt = 0U;
    Obs_Id_Ref = 0.0f;
    Obs_Iq_Ref = 0.0f;
    Sensorless_Blend = 0.0f;
}


bool Sensorless_Start_Active(void)
{
    return Start_Active;
}


bool Sensorless_Start_Ready(void)
{
    return Start_Ready;
}


bool Sensorless_Start_Run(float Ia_A,
                          float Ib_A,
                          float *Id_Ref,
                          float *Iq_Ref)
{
    float Ialpha;
    float Ibeta;
    float Theta_Start;
    float Theta_IF;
    float Theta_Obs;
    float Theta_Err;
    float Theta_Use;
    float Id_IF;
    float Iq_IF;
    float Blend;
    bool IF_Ready;
    uint32_t T0;

    if (!Start_Active)
    {
        *Id_Ref = 0.0f;
        *Iq_Ref = 0.0f;
        return false;
    }

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;

    if (Start_State == SENSORLESS_ALIGN)
    {
        Motor_Run.Theta_e = 0.0f;

        if (Align_Current(IF_ALIGN_ID_A,
                          IF_ALIGN_CNT,
                          Id_Ref,
                          Iq_Ref))
        {
            Current_Loop_State_Reset();

            Theta_Start = -(float)Start_Dir * (0.5f * PI_F);
            IF_Start_Reset(Theta_Start, Start_Dir);

            Flux_Obs.Para.Rs = Motor_Para.Rs;
            Flux_Obs.Para.Ls = Motor_Para.Ld;
            Flux_Obs.Para.Flux = Motor_Para.Flux;

            if (Motor_Para.Flux > 0.0f)
            {
                Flux_Obs.Para.Gamma =
                    TWO_PI_F * SENSORLESS_FLUX_OBS_BW_HZ /
                    (Motor_Para.Flux * Motor_Para.Flux);
            }
            else
            {
                Flux_Obs.Para.Gamma = 0.0f;
            }

            Flux_PLL.Para.Kp = SENSORLESS_PLL_KP;
            Flux_PLL.Para.Ki = SENSORLESS_PLL_KI;

            Flux_Observer_Reset(&Flux_Obs,
                                Theta_Start,
                                Ialpha,
                                Ibeta);
            PLL_Reset(&Flux_PLL, Theta_Start, 0.0f);
            Flux_Obs_U_Valid = false;

            Start_State = SENSORLESS_IF;
        }

        return false;
    }

    if (Flux_Obs_U_Valid)
    {
        if (Fast_Profile.Run != 0U)
        {
            T0 = DWT->CYCCNT;
        }

        Flux_Observer_Run(&Flux_Obs,
                          Motor_Run.Ualpha,
                          Motor_Run.Ubeta,
                          Ialpha,
                          Ibeta,
                          CUR_TS);

        if (Fast_Profile.Run != 0U)
        {
            Fast_Profile_Add(&Fast_Profile.Flux_Observer,
                             DWT->CYCCNT - T0);
            T0 = DWT->CYCCNT;
        }

        PLL_Run(&Flux_PLL,
                Flux_Obs.State.PsiAlpha,
                Flux_Obs.State.PsiBeta,
                Flux_Obs.Para.Flux,
                CUR_TS);

        if (Fast_Profile.Run != 0U)
        {
            Fast_Profile_Add(&Fast_Profile.PLL,
                             DWT->CYCCNT - T0);
        }
    }

    if (Fast_Profile.Run != 0U)
    {
        T0 = DWT->CYCCNT;
    }

    IF_Ready = IF_Start_Run(&Theta_IF, &Id_IF, &Iq_IF);

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.IF_Start,
                         DWT->CYCCNT - T0);
    }

    Flux_Obs_U_Valid = true;
    Theta_Obs = Flux_PLL.State.Theta;

    if (IF_Ready && !Profile_Requested)
    {
        Fast_Profile_Request();
        Profile_Requested = true;
    }

    if (Start_State == SENSORLESS_IF)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (IF_Ready)
        {
            Obs_Wait_Cnt = 0U;
            Sensorless_We_Obs_F = Flux_PLL.State.We;
            Start_State = SENSORLESS_OBS_WAIT;
        }
    }
    else if (Start_State == SENSORLESS_OBS_WAIT)
    {
        Theta_Use = Theta_IF;
        *Id_Ref = Id_IF;
        *Iq_Ref = Iq_IF;

        if (Observer_Stable())
        {
            if (Obs_Wait_Cnt < SENSORLESS_OBS_WAIT_CNT)
            {
                Obs_Wait_Cnt++;
            }

            if (Obs_Wait_Cnt >= SENSORLESS_OBS_WAIT_CNT)
            {
                Blend_Cnt = 0U;
                Sensorless_Blend = 0.0f;
                Start_State = SENSORLESS_BLEND;
            }
        }
        else
        {
            Obs_Wait_Cnt = 0U;
        }
    }
    else if (Start_State == SENSORLESS_BLEND)
    {
        if (SENSORLESS_BLEND_CNT > 0U)
        {
            Blend = (float)(Blend_Cnt + 1U) /
                    (float)SENSORLESS_BLEND_CNT;
        }
        else
        {
            Blend = 1.0f;
        }

        if (Blend > 1.0f)
        {
            Blend = 1.0f;
        }

        Theta_Err = Angle_Diff(Theta_Obs, Theta_IF);
        Theta_Use = Angle_Wrap(Theta_IF + Blend * Theta_Err);

        Current_Ref_Rotate(Theta_IF,
                           Theta_Use,
                           Id_IF,
                           Iq_IF,
                           Id_Ref,
                           Iq_Ref);

        Sensorless_Blend = Blend;

        if (Blend_Cnt < SENSORLESS_BLEND_CNT)
        {
            Blend_Cnt++;
        }

        if (Blend_Cnt >= SENSORLESS_BLEND_CNT)
        {
            Obs_Id_Ref = *Id_Ref;
            Obs_Iq_Ref = *Iq_Ref;
            Start_Ready = true;
            Sensorless_Blend = 1.0f;
            Start_State = SENSORLESS_OBS_HOLD;
        }
    }
    else if (Start_State == SENSORLESS_OBS_HOLD)
    {
        Theta_Use = Theta_Obs;
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;
        Start_Ready = true;
        Sensorless_Blend = 1.0f;
        Start_State = SENSORLESS_OBS_CURRENT_TRANS;
    }
    else if (Start_State == SENSORLESS_OBS_CURRENT_TRANS)
    {
        Theta_Use = Theta_Obs;
        Obs_Id_Ref = Ramp_Zero(Obs_Id_Ref, SENSORLESS_ID_RAMP_STEP_A);
        *Id_Ref = Obs_Id_Ref;
        *Iq_Ref = Obs_Iq_Ref;
        Start_Ready = true;
        Sensorless_Blend = 1.0f;

        if (Obs_Id_Ref == 0.0f)
        {
            Start_State = SENSORLESS_RUN;
        }
    }
    else
    {
        Theta_Use = Theta_Obs;
        *Id_Ref = 0.0f;
        *Iq_Ref = Obs_Iq_Ref;
        Start_Ready = true;
        Sensorless_Blend = 1.0f;
    }

    Motor_Run.Theta_e = Theta_Use;

    Sensorless_Theta_IF = Theta_IF;
    Sensorless_Theta_Use = Theta_Use;
    Sensorless_Id_Ref = *Id_Ref;
    Sensorless_Iq_Ref = *Iq_Ref;

    return Start_Ready;
}


Sensorless_Start_State_e Sensorless_Start_State_Get(void)
{
    return Start_State;
}
