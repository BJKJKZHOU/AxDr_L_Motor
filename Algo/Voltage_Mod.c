#include "Voltage_Mod.h"

#include "Math.h"


void SVPWM_Calc(float Ualpha,
                 float Ubeta,
                 float Vbus,
                 float *DutyA,
                 float *DutyB,
                 float *DutyC)
{
    float Ua;
    float Ub;
    float Uc;
    float Umax;
    float Umin;
    float Uoff;
    float Da;
    float Db;
    float Dc;

    if (Vbus <= 0.0f)
    {
        *DutyA = 0.5f;
        *DutyB = 0.5f;
        *DutyC = 0.5f;
        return;
    }

    Ua = Ualpha;
    Ub = -0.5f * Ualpha + SQRT3_HALF_F * Ubeta;
    Uc = -0.5f * Ualpha - SQRT3_HALF_F * Ubeta;

    Umax = Ua;
    if (Ub > Umax)
    {
        Umax = Ub;
    }
    if (Uc > Umax)
    {
        Umax = Uc;
    }

    Umin = Ua;
    if (Ub < Umin)
    {
        Umin = Ub;
    }
    if (Uc < Umin)
    {
        Umin = Uc;
    }

    Uoff = -0.5f * (Umax + Umin);

    Da = 0.5f + (Ua + Uoff) / Vbus;
    Db = 0.5f + (Ub + Uoff) / Vbus;
    Dc = 0.5f + (Uc + Uoff) / Vbus;

    Limit_Value(&Da, 0.0f, 1.0f);
    Limit_Value(&Db, 0.0f, 1.0f);
    Limit_Value(&Dc, 0.0f, 1.0f);

    *DutyA = Da;
    *DutyB = Db;
    *DutyC = Dc;
}
