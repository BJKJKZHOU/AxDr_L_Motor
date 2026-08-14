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


#endif /* MOTOR_TYPE_H */
