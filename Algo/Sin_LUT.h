#ifndef SIN_LUT_H
#define SIN_LUT_H

/*
 * Lookup-table sine/cosine for electrical angle in radians.
 * Theta is expected in [0, 2pi); Motor_Run.Theta_e already follows this range.
 */
void SinCos(float Theta, float *Sin, float *Cos);

#endif /* SIN_LUT_H */
