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

#define MOTOR_KT_DEFAULT (1.5f * (float)MOTOR_PP_DEFAULT * MOTOR_FLUX_DEFAULT)

/* MT6816 absolute speed capability; this is the system hardware speed ceiling. */
#define ENC_MAX_RPM 25000.0f
#define ENC_WM_MAX  (ENC_MAX_RPM * 6.283185307f / 60.0f)

/* First-version phase-current command ceiling based on the board 10 A operating range. */
#define MOTOR_I_MAX_DEFAULT  10.0f
#define MOTOR_TE_MAX_DEFAULT (MOTOR_KT_DEFAULT * MOTOR_I_MAX_DEFAULT)
#define MOTOR_WM_MAX_DEFAULT ENC_WM_MAX

#define USER_I_MAX_DEFAULT  5.0f
#define USER_TE_MAX_DEFAULT (MOTOR_KT_DEFAULT * USER_I_MAX_DEFAULT)
#define USER_WM_MAX_DEFAULT 314.159265f /* 3000 rpm */

#define MOTOR_ENC_DIR_DEFAULT   1
#define MOTOR_THETA_OFF_DEFAULT 0.0f
#define ENCODER_TYPE_DEFAULT    ENC_MT6816

#define ENCODER_CONFIG_DEFAULT                                                                                         \
    {                                                                                                                  \
        .Type = ENCODER_TYPE_DEFAULT,                                                                                  \
    }

#define MOTOR_CAL_DEFAULT                                                                                              \
    {                                                                                                                  \
        .Enc_Dir = MOTOR_ENC_DIR_DEFAULT,                                                                              \
        .Theta_Off = MOTOR_THETA_OFF_DEFAULT,                                                                          \
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
        .Te_Max = MOTOR_TE_MAX_DEFAULT,                                                                                \
        .Wm_Max = MOTOR_WM_MAX_DEFAULT,                                                                                \
    }

#define USER_LIM_DEFAULT                                                                                               \
    {                                                                                                                  \
        .I_Max = USER_I_MAX_DEFAULT,                                                                                   \
        .Te_Max = USER_TE_MAX_DEFAULT,                                                                                 \
        .Wm_Max = USER_WM_MAX_DEFAULT,                                                                                 \
    }

#endif /* USER_MOTOR_PARAMS_H */
