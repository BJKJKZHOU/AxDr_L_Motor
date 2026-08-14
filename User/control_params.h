#ifndef USER_CONTROL_PARAMS_H
#define USER_CONTROL_PARAMS_H

#include "Math.h"
#include "PID.h"
#include "motor_params.h"


/*
 * Current-loop PI design.
 *
 * PMSM dq electrical plant:
 *
 *             1
 * G(s) = -------------
 *          L*s + Rs
 *
 * Choose the current-loop bandwidth Fc and place the PI zero at the
 * electrical pole:
 *
 * Wc = 2*pi*Fc
 * Kp = L*Wc
 * Ki = Rs*Wc
 *
 * Id uses Ld and Iq uses Lq.
 */
#define CUR_BW_HZ_DEFAULT      1000.0f
#define CUR_WC_DEFAULT         (TWO_PI_F * CUR_BW_HZ_DEFAULT)

#define ID_KP_DEFAULT          (MOTOR_LD_DEFAULT * CUR_WC_DEFAULT)
#define ID_KI_DEFAULT          (MOTOR_RS_DEFAULT * CUR_WC_DEFAULT)

#define IQ_KP_DEFAULT          (MOTOR_LQ_DEFAULT * CUR_WC_DEFAULT)
#define IQ_KI_DEFAULT          (MOTOR_RS_DEFAULT * CUR_WC_DEFAULT)


#define ID_CTRL_DEFAULT                \
{                                      \
    .Para =                            \
    {                                  \
        .Kp = ID_KP_DEFAULT,           \
        .Ki = ID_KI_DEFAULT,           \
        .Kd = 0.0f,                    \
    },                                 \
}

#define IQ_CTRL_DEFAULT                \
{                                      \
    .Para =                            \
    {                                  \
        .Kp = IQ_KP_DEFAULT,           \
        .Ki = IQ_KI_DEFAULT,           \
        .Kd = 0.0f,                    \
    },                                 \
}


#endif /* USER_CONTROL_PARAMS_H */
