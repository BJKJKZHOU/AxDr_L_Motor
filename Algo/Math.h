#ifndef MATH_H
#define MATH_H


#define PI_F            3.14159265358979323846f
#define TWO_PI_F        6.28318530717958647692f
#define SQRT3_F         1.73205080756887729353f
#define INV_SQRT3_F     0.57735026918962576451f
#define SQRT3_HALF_F    0.86602540378443864676f


float Limit_Value(float In, float Min, float Max);
void Vector2_Limit(float *X, float *Y, float Lim);
float Angle_Wrap(float Theta);

#endif
