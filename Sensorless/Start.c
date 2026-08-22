#include "Start.h"

#include "Align.h"
#include "Current_Loop.h"
#include "Fast_Profile.h"
#include "Flux_Observer.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motor_Type.h"
#include "PLL.h"
#include "control_params.h"
#include "main.h"


#define SENSORLESS_FLUX_OBS_BW_HZ      200.0f
#define SENSORLESS_PLL_BW_HZ            50.0f
#define SENSORLESS_PLL_DAMPING          0.707f
#define SENSORLESS_PLL_WN               (TWO_PI_F * SENSORLESS_PLL_BW_HZ)
#define SENSORLESS_PLL_KP               (2.0f * SENSORLESS_PLL_DAMPING * SENSORLESS_PLL_WN)
#define SENSORLESS_PLL_KI               (SENSORLESS_PLL_WN * SENSORLESS_PLL_WN)


static Sensorless_Start_State_e Start_State = SENSORLESS_ALIGN;
static int8_t Start_Dir = 1;
static volatile bool Start_Active = false;
static volatile bool Start_Ready = false;

Flux_Observer_T Flux_Obs = {0};
PLL_T Flux_PLL = {0};
static bool Flux_Obs_U_Valid = false;
static bool Profile_Requested = false;


void Sensorless_Start_Begin(int8_t Dir)
{
    Start_State = SENSORLESS_ALIGN;
    Start_Dir = (Dir >= 0) ? 1 : -1;
    Start_Ready = false;
    Flux_Obs_U_Valid = false;
    Profile_Requested = false;

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

    Start_Ready = IF_Start_Run(&Motor_Run.Theta_e, Id_Ref, Iq_Ref);

    if (Fast_Profile.Run != 0U)
    {
        Fast_Profile_Add(&Fast_Profile.IF_Start,
                         DWT->CYCCNT - T0);
    }

    Flux_Obs_U_Valid = true;

    if (Start_Ready && !Profile_Requested)
    {
        Fast_Profile_Request();
        Profile_Requested = true;
    }

    return Start_Ready;
}


Sensorless_Start_State_e Sensorless_Start_State_Get(void)
{
    return Start_State;
}
