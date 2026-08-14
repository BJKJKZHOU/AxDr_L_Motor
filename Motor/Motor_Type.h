#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H

#include <stdint.h>


/*
 * Mechanical positive direction convention:
 * Viewed from the motor output-shaft side toward the motor body, CCW is positive.
 * Mechanical angle, speed and torque signs must follow this convention.
 *
 * Enc_Dir maps the encoder raw direction into this mechanical coordinate system.
 * After direction correction, Theta_m increases in the positive mechanical direction.
 * Theta_Off is the calibrated electrical angle offset used by:
 * Theta_e = wrap(Pp * Theta_m + Theta_Off).
 */
typedef struct
{
    int8_t Enc_Dir;       /* +1 / -1 */
    float Theta_Off;      /* rad */

} Motor_Cal_T;


typedef struct
{
    uint8_t Pp;        /* Pole pairs */

    float Rs;          /* Ohm */
    float Ld;          /* H */
    float Lq;          /* H */
    float Flux;        /* Wb, Te = 1.5 * Pp * Flux * Iq */

    float J;           /* kg*m^2 */
    float B;           /* N*m/(rad/s) */

} Motor_Para_T;


/*
 * Operating limits, not fault thresholds.
 *
 * Motor_Lim: firmware-defined product operating limits. Users must not
 *            configure commands beyond these limits.
 * User_Lim:  user-configurable operating limits for a more conservative
 *            application range.
 *
 * Effective control limits use the smaller value of Motor_Lim and User_Lim.
 * Reaching or clipping at these limits is normal control behavior and must
 * not be treated as a Warning/Fault/Disable condition.
 *
 * Protection thresholds and hardware trip limits are separate from this type.
 */
typedef struct
{
    float I_Max;       /* A */
    float Te_Max;      /* N*m */
    float Wm_Max;      /* rad/s */

} Motor_Limit_T;


#endif /* MOTOR_TYPE_H */
