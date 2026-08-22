#include "Flux.h"

#include "Align.h"
#include "Current_Loop.h"
#include "IF_Start.h"
#include "Math.h"
#include "Motor_Type.h"
#include "Sin_LUT.h"
#include "control_params.h"


#define FLUX_POINT_NUM                 4U
#define FLUX_IDENT_WE_1_RAD_S          120.0f
#define FLUX_IDENT_WE_2_RAD_S          160.0f
#define FLUX_IDENT_WE_3_RAD_S          200.0f
#define FLUX_IDENT_WE_4_RAD_S          240.0f
#define FLUX_IDENT_SETTLE_TIME_S       0.30f
#define FLUX_IDENT_MEASURE_TIME_S      0.20f
#define FLUX_IDENT_SETTLE_CNT          ((uint32_t)(FLUX_IDENT_SETTLE_TIME_S / CUR_TS + 0.5f))
#define FLUX_IDENT_MEASURE_CNT         ((uint32_t)(FLUX_IDENT_MEASURE_TIME_S / CUR_TS + 0.5f))
#define FLUX_IDENT_FINISH_IQ_DEC_A_S   20.0f
#define FLUX_IDENT_FINISH_I_A          0.20f
#define FLUX_IDENT_FINISH_TIME_S       0.002f
#define FLUX_IDENT_FINISH_CNT          ((uint32_t)(FLUX_IDENT_FINISH_TIME_S / CUR_TS + 0.5f))


static const float Flux_We_Point[FLUX_POINT_NUM] =
{
    FLUX_IDENT_WE_1_RAD_S,
    FLUX_IDENT_WE_2_RAD_S,
    FLUX_IDENT_WE_3_RAD_S,
    FLUX_IDENT_WE_4_RAD_S,
};

static volatile Flux_State_e Flux_State = FLUX_IDLE;
static Flux_Result_T Flux_Result = {0};
static uint8_t Flux_Point = 0U;
static uint32_t Flux_Cnt = 0U;
static uint32_t Flux_Meas_Cnt = 0U;
static float Flux_E_Sum = 0.0f;
static float Flux_We_Mean[FLUX_POINT_NUM] = {0};
static float Flux_E_Mean[FLUX_POINT_NUM] = {0};
static float Flux_Theta_Pre = 0.0f;
static float Flux_Finish_Iq_Ref = 0.0f;
static uint8_t Flux_U_Valid = 0U;
static uint8_t Flux_Finish_Init = 0U;


static float Abs_Value(float Value)
{
    return (Value >= 0.0f) ? Value : -Value;
}


static void Flux_Measure(float Ia_A, float Ib_A)
{
    float Ialpha;
    float Ibeta;
    float Sin;
    float Cos;
    float Ualpha;
    float Ubeta;
    float Ealpha;
    float Ebeta;
    float E;

    if (Flux_U_Valid == 0U)
    {
        return;
    }

    SinCos(Flux_Theta_Pre, &Sin, &Cos);

    Ualpha = Motor_Run.Ud * Cos - Motor_Run.Uq * Sin;
    Ubeta = Motor_Run.Ud * Sin + Motor_Run.Uq * Cos;

    Ialpha = Ia_A;
    Ibeta = (Ia_A + 2.0f * Ib_A) * INV_SQRT3_F;

    Ealpha = Ualpha - Motor_Para.Rs * Ialpha;
    Ebeta = Ubeta - Motor_Para.Rs * Ibeta;
    E = __builtin_sqrtf(Ealpha * Ealpha + Ebeta * Ebeta);

    Flux_E_Sum += E;
    Flux_Meas_Cnt++;
}


void Flux_Start(void)
{
    Flux_Reset();
    Align_Reset();
    Current_Loop_State_Reset();
    Flux_State = FLUX_ALIGN;
}


void Flux_Reset(void)
{
    Flux_State = FLUX_IDLE;
    Flux_Result.Flux_Wb = 0.0f;
    Flux_Result.V_Offset_V = 0.0f;
    Flux_Result.Fit_R2 = 0.0f;
    Flux_Result.Valid = false;
    Flux_Point = 0U;
    Flux_Cnt = 0U;
    Flux_Meas_Cnt = 0U;
    Flux_E_Sum = 0.0f;
    Flux_Theta_Pre = 0.0f;
    Flux_Finish_Iq_Ref = 0.0f;
    Flux_U_Valid = 0U;
    Flux_Finish_Init = 0U;

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Flux_We_Mean[n] = 0.0f;
        Flux_E_Mean[n] = 0.0f;
    }
}


void Flux_Fail(void)
{
    Flux_State = FLUX_FAILED;
    Flux_Result.Valid = false;
}


void Flux_Control(void)
{
    float Sum_X = 0.0f;
    float Sum_Y = 0.0f;
    float Sum_XX = 0.0f;
    float Sum_XY = 0.0f;
    float Y_Mean;
    float SS_Tot = 0.0f;
    float SS_Err = 0.0f;
    float Den;
    float Y_Est;

    if (Flux_State != FLUX_CALC)
    {
        return;
    }

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Sum_X += Flux_We_Mean[n];
        Sum_Y += Flux_E_Mean[n];
        Sum_XX += Flux_We_Mean[n] * Flux_We_Mean[n];
        Sum_XY += Flux_We_Mean[n] * Flux_E_Mean[n];
    }

    Den = (float)FLUX_POINT_NUM * Sum_XX - Sum_X * Sum_X;

    if (Den <= 0.0f)
    {
        Flux_Result.Valid = false;
        Flux_Cnt = 0U;
        Flux_Finish_Init = 0U;
        Flux_State = FLUX_FINISH;
        return;
    }

    Flux_Result.Flux_Wb =
        ((float)FLUX_POINT_NUM * Sum_XY - Sum_X * Sum_Y) / Den;
    Flux_Result.V_Offset_V =
        (Sum_Y - Flux_Result.Flux_Wb * Sum_X) / (float)FLUX_POINT_NUM;

    Y_Mean = Sum_Y / (float)FLUX_POINT_NUM;

    for (uint8_t n = 0U; n < FLUX_POINT_NUM; n++)
    {
        Y_Est = Flux_Result.Flux_Wb * Flux_We_Mean[n]
              + Flux_Result.V_Offset_V;
        SS_Tot += (Flux_E_Mean[n] - Y_Mean)
                * (Flux_E_Mean[n] - Y_Mean);
        SS_Err += (Flux_E_Mean[n] - Y_Est)
                * (Flux_E_Mean[n] - Y_Est);
    }

    Flux_Result.Fit_R2 = (SS_Tot > 0.0f)
                       ? (1.0f - SS_Err / SS_Tot)
                       : 0.0f;

    /* First hardware version keeps validity deliberately minimal.
     * Fit_R2 is reported for tuning; a quality threshold will be set from data. */
    Flux_Result.Valid = (Flux_Result.Flux_Wb > 0.0f);
    Flux_Cnt = 0U;
    Flux_Finish_Init = 0U;
    Flux_State = FLUX_FINISH;
}


bool Flux_Active(void)
{
    return (Flux_State != FLUX_IDLE) &&
           (Flux_State != FLUX_DONE) &&
           (Flux_State != FLUX_FAILED);
}


Motor_Fast_Mode_e Flux_Fast_Run(float Ia_A,
                                float Ib_A,
                                float Ic_A,
                                float *Id_Ref,
                                float *Iq_Ref)
{
    float Theta_e;
    float Iq_Step;

    *Id_Ref = 0.0f;
    *Iq_Ref = 0.0f;

    if (!Flux_Active())
    {
        return FAST_OFF;
    }

    if (Flux_State == FLUX_ALIGN)
    {
        Motor_Run.Theta_e = 0.0f;

        if (Align_Current(IF_ALIGN_ID_A,
                          IF_ALIGN_CNT,
                          Id_Ref,
                          Iq_Ref))
        {
            Current_Loop_State_Reset();
            IF_Start_Reset(-0.5f * PI_F, 1);
            IF_Start_Target_Set(Flux_We_Point[0]);
            Flux_State = FLUX_ACCEL;
        }

        return FAST_CURRENT;
    }

    if (Flux_State == FLUX_FINISH)
    {
        (void)IF_Start_Run(&Theta_e, Id_Ref, Iq_Ref);
        Motor_Run.Theta_e = Theta_e;

        if (Flux_Finish_Init == 0U)
        {
            Flux_Finish_Iq_Ref = *Iq_Ref;
            Flux_Finish_Init = 1U;
        }

        Iq_Step = FLUX_IDENT_FINISH_IQ_DEC_A_S * CUR_TS;

        if (Flux_Finish_Iq_Ref > Iq_Step)
        {
            Flux_Finish_Iq_Ref -= Iq_Step;
        }
        else if (Flux_Finish_Iq_Ref < -Iq_Step)
        {
            Flux_Finish_Iq_Ref += Iq_Step;
        }
        else
        {
            Flux_Finish_Iq_Ref = 0.0f;
        }

        *Id_Ref = 0.0f;
        *Iq_Ref = Flux_Finish_Iq_Ref;

        if ((Flux_Finish_Iq_Ref == 0.0f) &&
            (Abs_Value(Ia_A) <= FLUX_IDENT_FINISH_I_A) &&
            (Abs_Value(Ib_A) <= FLUX_IDENT_FINISH_I_A) &&
            (Abs_Value(Ic_A) <= FLUX_IDENT_FINISH_I_A))
        {
            if (++Flux_Cnt >= FLUX_IDENT_FINISH_CNT)
            {
                Flux_State = Flux_Result.Valid ? FLUX_DONE : FLUX_FAILED;
            }
        }
        else
        {
            Flux_Cnt = 0U;
        }

        return FAST_CURRENT;
    }

    if (Flux_State == FLUX_MEASURE)
    {
        Flux_Measure(Ia_A, Ib_A);

        if (Flux_Meas_Cnt >= FLUX_IDENT_MEASURE_CNT)
        {
            Flux_We_Mean[Flux_Point] = IF_Start_We_Get();
            Flux_E_Mean[Flux_Point] = Flux_E_Sum / (float)Flux_Meas_Cnt;

            Flux_Point++;

            if (Flux_Point >= FLUX_POINT_NUM)
            {
                Flux_State = FLUX_CALC;
            }
            else
            {
                Flux_E_Sum = 0.0f;
                Flux_Meas_Cnt = 0U;
                Flux_Cnt = 0U;
                IF_Start_Target_Set(Flux_We_Point[Flux_Point]);
                Flux_State = FLUX_ACCEL;
            }
        }
    }

    (void)IF_Start_Run(&Theta_e, Id_Ref, Iq_Ref);
    Motor_Run.Theta_e = Theta_e;
    Flux_Theta_Pre = Theta_e;
    Flux_U_Valid = 1U;

    if (Flux_State == FLUX_ACCEL)
    {
        if (IF_Start_State_Get() == IF_HOLD)
        {
            Flux_Cnt = 0U;
            Flux_State = FLUX_SETTLE;
        }
    }
    else if (Flux_State == FLUX_SETTLE)
    {
        if (++Flux_Cnt >= FLUX_IDENT_SETTLE_CNT)
        {
            Flux_Cnt = 0U;
            Flux_E_Sum = 0.0f;
            Flux_Meas_Cnt = 0U;
            Flux_State = FLUX_MEASURE;
        }
    }

    return FAST_CURRENT;
}


Flux_State_e Flux_State_Get(void)
{
    return Flux_State;
}


const Flux_Result_T *Flux_Result_Get(void)
{
    return &Flux_Result;
}
