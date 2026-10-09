/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef USER_MOTOR_PARAMS_H
#define USER_MOTOR_PARAMS_H

#include "Encoder.h"
#include "Motor_Type.h"

/* Current 32-pole outer-rotor motor current-loop parameters. */
#define MOTOR_PP_DEFAULT 16U
#define MOTOR_RS_DEFAULT 0.08471736f
#define MOTOR_LD_DEFAULT 0.000017836f
#define MOTOR_LQ_DEFAULT 0.000017836f

/* Inertia and damping still require identification for this motor. */
#define MOTOR_FLUX_DEFAULT 0.0031835556f
#define MOTOR_J_DEFAULT    9.08865259e-05f
#define MOTOR_B_DEFAULT    0.000188353f

#define MOTOR_ALIGN_CURRENT_DEFAULT 1.0f
#define MOTOR_IF_CURRENT_DEFAULT    1.0f
#define MOTOR_RL_I_PEAK_DEFAULT     1.0f

/* Default motor torque constant is only used to initialize compile-time controller tuning. */
#define MOTOR_KT_DEFAULT (1.5f * (float)MOTOR_PP_DEFAULT * MOTOR_FLUX_DEFAULT)

/* MT6816 absolute speed capability; this is the system hardware speed ceiling. */
#define ENC_MAX_RPM 25000.0f
#define ENC_WM_MAX  (ENC_MAX_RPM * 6.283185307f / 60.0f)

/* First-version phase-current command ceiling based on the board 10 A operating range. */
#define MOTOR_I_MAX_DEFAULT  10.0f
#define MOTOR_WM_MAX_DEFAULT ENC_WM_MAX

#define USER_I_MAX_DEFAULT  10.0f
#define USER_WM_MAX_DEFAULT 1256.637061f /* 12000 rpm */

/*
 * AxDr_L V1.3 fixed board-level DC-bus protection limits.
 * These safety thresholds are not user-configurable.
 */
#define VBUS_UV_FAULT_V          5.0f
#define VBUS_OV_FAULT_V         52.0f
#define VBUS_UV_DEBOUNCE_TICKS  20U /* 10 ms at the 2 kHz Motor thread */
#define VBUS_OV_DEBOUNCE_TICKS   1U /* 0.5 ms at the 2 kHz Motor thread */

/*
 * Fixed AxDr_L V1.3 software overcurrent policy.
 * MOTOR_I_MAX_DEFAULT remains the normal command ceiling; these thresholds
 * protect actual measured current and are not Host-configurable.
 */
#define PROT_CURRENT_OVERLOAD_A       15.0f
#define PROT_CURRENT_OVERLOAD_CYCLES  5000U /* 250 ms at 20 kHz */
#define PROT_CURRENT_FAST_A           25.0f
#define PROT_CURRENT_FAST_CYCLES         5U /* 250 us at 20 kHz */

#define MOTOR_ENC_DIR_DEFAULT   1
#define MOTOR_THETA_OFF_DEFAULT 0.0f
#define ENCODER_PROTOCOL_DEFAULT ENC_PROTOCOL_SPI
#define ENCODER_SPI_TYPE_DEFAULT ENC_SPI_MT6816

#define ENCODER_CONFIG_DEFAULT                                                                                         \
    {                                                                                                                  \
        .Protocol = ENCODER_PROTOCOL_DEFAULT,                                                                          \
        .SPI_Type = ENCODER_SPI_TYPE_DEFAULT,                                                                          \
    }

#define MOTOR_CAL_DEFAULT                                                                                              \
    {                                                                                                                  \
        .Enc_Dir = MOTOR_ENC_DIR_DEFAULT,                                                                              \
        .Theta_Off = MOTOR_THETA_OFF_DEFAULT,                                                                          \
        .Valid = 0U,                                                                                                   \
    }

#define MOTOR_PARA_DEFAULT                                                                                             \
    {                                                                                                                  \
        .Pp = MOTOR_PP_DEFAULT,                                                                                        \
        .Rs = MOTOR_RS_DEFAULT,                                                                                        \
        .Ld = MOTOR_LD_DEFAULT,                                                                                        \
        .Lq = MOTOR_LQ_DEFAULT,                                                                                        \
        .Flux = MOTOR_FLUX_DEFAULT,                                                                                    \
        .J = MOTOR_J_DEFAULT,                                                                                          \
        .B = MOTOR_B_DEFAULT,                                                                                          \
    }

#define MOTOR_LIM_DEFAULT                                                                                              \
    {                                                                                                                  \
        .I_Max = MOTOR_I_MAX_DEFAULT,                                                                                  \
        .Wm_Max = MOTOR_WM_MAX_DEFAULT,                                                                                \
    }

#define USER_LIM_DEFAULT                                                                                               \
    {                                                                                                                  \
        .I_Max = USER_I_MAX_DEFAULT,                                                                                   \
        .Wm_Max = USER_WM_MAX_DEFAULT,                                                                                 \
    }

#endif /* USER_MOTOR_PARAMS_H */
