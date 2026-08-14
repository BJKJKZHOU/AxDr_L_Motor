#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H

#include <stdint.h>


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
