#include "Math.h"


float Limit_Value(float In, float Min, float Max)
{
    if (In > Max)
    {
        return Max;
    }

    if (In < Min)
    {
        return Min;
    }

    return In;
}


void Vector2_Limit(float *X, float *Y, float Lim)
{
    float Mag2;
    float Lim2;
    float Scale;

    if (Lim <= 0.0f)
    {
        *X = 0.0f;
        *Y = 0.0f;
        return;
    }

    Mag2 = (*X * *X) + (*Y * *Y);
    Lim2 = Lim * Lim;

    if (Mag2 <= Lim2)
    {
        return;
    }

    Scale = Lim / __builtin_sqrtf(Mag2);

    *X *= Scale;
    *Y *= Scale;
}


float Angle_Wrap(float Theta)
{
    while (Theta >= TWO_PI_F)
    {
        Theta -= TWO_PI_F;
    }

    while (Theta < 0.0f)
    {
        Theta += TWO_PI_F;
    }

    return Theta;
}
