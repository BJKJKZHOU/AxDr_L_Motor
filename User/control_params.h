#ifndef USER_CONTROL_PARAMS_H
#define USER_CONTROL_PARAMS_H

#include "Math.h"
#include "PID.h"
#include "motor_params.h"


#define CUR_FREQ_HZ_DEFAULT    20000.0f
#define CUR_TS                 (1.0f / CUR_FREQ_HZ_DEFAULT)
#define VOLT_MOD_MAX           0.95f


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


/*
 * Speed-loop PI design.
 *
 * Mechanical plant from Iq to mechanical speed:
 *
 *                  Kt
 * G(s) = ---------------------
 *             J*s + B
 *
 * Kt = 1.5 * Pp * Flux
 *
 * Choose speed-loop bandwidth Fs and place the PI zero at the
 * mechanical pole:
 *
 * Ws = 2*pi*Fs
 * Kp = J*Ws/Kt
 * Ki = B*Ws/Kt
 *
 * Speed PI output is Iq_Ref directly.
 */
#define SPD_FREQ_HZ_DEFAULT    2000.0f
#define SPD_TS                 (1.0f / SPD_FREQ_HZ_DEFAULT)
#define SPD_FBK_ALPHA_DEFAULT  0.38586955f /* 200 Hz LPF at 2 kHz */

#define SPD_BW_HZ_DEFAULT      50.0f
#define SPD_WC_DEFAULT         (TWO_PI_F * SPD_BW_HZ_DEFAULT)

#define SPD_KP_DEFAULT         (MOTOR_J_DEFAULT * SPD_WC_DEFAULT / MOTOR_KT_DEFAULT)
#define SPD_KI_DEFAULT         (MOTOR_B_DEFAULT * SPD_WC_DEFAULT / MOTOR_KT_DEFAULT)


#define SPEED_CTRL_DEFAULT             \
{                                      \
    .Para =                            \
    {                                  \
        .Kp = SPD_KP_DEFAULT,          \
        .Ki = SPD_KI_DEFAULT,          \
        .Kd = 0.0f,                    \
    },                                 \
}


#endif /* USER_CONTROL_PARAMS_H */
