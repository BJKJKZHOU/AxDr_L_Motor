/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef USER_CONTROL_PARAMS_H
#define USER_CONTROL_PARAMS_H

#include "Math.h"
#include "PID.h"
#include "motor_params.h"

/* Fast-loop timing source. Current control runs once per PWM period. */
#define PWM_FREQ_HZ_DEFAULT 20000.0f
#define CUR_FREQ_HZ_DEFAULT PWM_FREQ_HZ_DEFAULT
#define CUR_TS              (1.0f / CUR_FREQ_HZ_DEFAULT)
#define VOLT_MOD_MAX        0.95f

/*
 * TIM1 CH4/CH5 timing is expressed as physical time, not fixed CCR values.
 *
 * Current 20 kHz measured baseline at TIM1 = 160 MHz:
 *   ARR  = 4000
 *   CCR4 = 2240
 *   CCR5 = 3665
 *
 * Therefore:
 *   PWM center -> ADC trigger = (4000 - 3665) / 160 MHz = 2.09375 us
 *   Encoder trigger -> ADC trigger = (3665 - 2240) / 160 MHz = 8.90625 us
 *
 * PWM_Timing_Update() converts these time constraints back to timer counts.
 * Changing PWM_FREQ_HZ_DEFAULT therefore moves ARR/CCR4/CCR5 together while
 * preserving the measured peripheral timing in seconds.
 */
#define ADC_TRIG_CENTER_S 2.09375e-6f
#define ENC_TRIG_LEAD_S   8.90625e-6f

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
#define CUR_BW_HZ_DEFAULT 1000.0f
#define CUR_WC_DEFAULT    (TWO_PI_F * CUR_BW_HZ_DEFAULT)

#define ID_KP_DEFAULT (MOTOR_LD_DEFAULT * CUR_WC_DEFAULT)
#define ID_KI_DEFAULT (MOTOR_RS_DEFAULT * CUR_WC_DEFAULT)

#define IQ_KP_DEFAULT (MOTOR_LQ_DEFAULT * CUR_WC_DEFAULT)
#define IQ_KI_DEFAULT (MOTOR_RS_DEFAULT * CUR_WC_DEFAULT)

#define ID_CTRL_DEFAULT                                                                                                \
    {                                      \
    .Para =                            \
    {                                  \
        .Kp = ID_KP_DEFAULT,           \
        .Ki = ID_KI_DEFAULT,           \
        .Kd = 0.0f,                    \
    },                                 \
}

#define IQ_CTRL_DEFAULT                                                                                                \
    {                                      \
    .Para =                            \
    {                                  \
        .Kp = IQ_KP_DEFAULT,           \
        .Ki = IQ_KI_DEFAULT,           \
        .Kd = 0.0f,                    \
    },                                 \
}

/* Open-loop current strategy. Speed comes from the signed mechanical command. */
#define OPEN_ALIGN_ID_A   0.2f
#define OPEN_ALIGN_TIME_S 0.5f
#define OPEN_ALIGN_CNT    ((uint32_t)(OPEN_ALIGN_TIME_S / CUR_TS + 0.5f))
#define OPEN_IQ_A         0.2f

/* Sensorless I/F startup and low-speed operation. */
#define IF_ALIGN_ID_A      1.0f
#define IF_ALIGN_TIME_S    1.0f
#define IF_ALIGN_CNT       ((uint32_t)(IF_ALIGN_TIME_S / CUR_TS + 0.5f))
#define IF_IQ_START_A      1.0f
#define IF_IQ_TARGET_A     1.6f
#define IF_IQ_SLEW_A_S     20.0f
#define IF_WE_TARGET_RAD_S 120.0f
#define IF_ACC_RAD_S2      15.0f
#define IF_HOLD_TIME_S     0.3f
#define IF_HOLD_CNT        ((uint32_t)(IF_HOLD_TIME_S / CUR_TS + 0.5f))

/*
 * Mechanical-speed reference profile.
 * Acc/Dec are positive magnitudes in mechanical rad/s^2. Reversal always
 * decelerates Wm_Ref to zero before accelerating in the opposite direction.
 * These are first-version motion-policy defaults and require hardware tuning.
 */
#define MOTION_ACC_RAD_S2 100.0f
#define MOTION_DEC_RAD_S2 100.0f

/*
 * Speed-loop PI design.
 *
 * Controller input is electrical speed We. User/communication speed remains
 * mechanical speed Wm and is converted with We = Pp * Wm before this loop.
 *
 * Mechanical plant:
 *
 *                  Kt
 * Wm(s)/Iq(s) = ---------
 *                 J*s+B
 *
 * Since We = Pp * Wm:
 *
 *                  Pp*Kt
 * We(s)/Iq(s) = ---------
 *                  J*s+B
 *
 * Choose speed-loop bandwidth Fs and place the PI zero at the mechanical
 * pole:
 *
 * Ws = 2*pi*Fs
 * Kp = J*Ws/(Pp*Kt)
 * Ki = B*Ws/(Pp*Kt)
 *
 * Speed PI output is Iq_Ref directly.
 */
#define SPD_FREQ_HZ_DEFAULT   2000.0f
#define SPD_TS                (1.0f / SPD_FREQ_HZ_DEFAULT)
#define SPD_FBK_ALPHA_DEFAULT 0.38586955f /* 200 Hz LPF at 2 kHz */

#define SPD_BW_HZ_DEFAULT 50.0f
#define SPD_WC_DEFAULT    (TWO_PI_F * SPD_BW_HZ_DEFAULT)

#define SPD_KP_DEFAULT (MOTOR_J_DEFAULT * SPD_WC_DEFAULT / ((float)MOTOR_PP_DEFAULT * MOTOR_KT_DEFAULT))
#define SPD_KI_DEFAULT (MOTOR_B_DEFAULT * SPD_WC_DEFAULT / ((float)MOTOR_PP_DEFAULT * MOTOR_KT_DEFAULT))

#define POS_FREQ_HZ_DEFAULT 1000.0f
#define POS_TS              (1.0f / POS_FREQ_HZ_DEFAULT)
#define POS_KP_DEFAULT      5.0f /* (rad/s)/rad */

#define POSITION_CTRL_DEFAULT                                                                                          \
    {                                      \
    .Para =                            \
    {                                  \
        .Kp = POS_KP_DEFAULT,          \
        .Ki = 0.0f,                    \
        .Kd = 0.0f,                    \
    },                                 \
}

#define SPEED_CTRL_DEFAULT                                                                                             \
    {                                      \
    .Para =                            \
    {                                  \
        .Kp = SPD_KP_DEFAULT,          \
        .Ki = SPD_KI_DEFAULT,          \
        .Kd = 0.0f,                    \
    },                                 \
}

#endif /* USER_CONTROL_PARAMS_H */
