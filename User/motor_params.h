/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef USER_MOTOR_PARAMS_H
#define USER_MOTOR_PARAMS_H

#include "Board_Config.h"
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

#define USER_I_MAX_DEFAULT  10.0f
#define USER_WM_MAX_DEFAULT 1256.637061f /* 12000 rpm */

#define MOTOR_ENC_DIR_DEFAULT   1
#define MOTOR_THETA_OFF_DEFAULT 0.0f
#define ENCODER_PROTOCOL_DEFAULT ENC_PROTOCOL_SPI
#define ENCODER_SPI_TYPE_DEFAULT ENC_SPI_MT6816
#define ENCODER_ABZ_PPR_DEFAULT  4096U

#define ENCODER_CONFIG_DEFAULT                                                                                         \
    {                                                                                                                  \
        .Protocol = ENCODER_PROTOCOL_DEFAULT,                                                                          \
        .SPI_Type = ENCODER_SPI_TYPE_DEFAULT,                                                                          \
        .ABZ_PPR = ENCODER_ABZ_PPR_DEFAULT,                                                                         \
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
        .I_Max = DRIVE_I_MAX_A,                                                                                        \
        .Wm_Max = DRIVE_WM_MAX,                                                                                        \
    }

#define USER_LIM_DEFAULT                                                                                               \
    {                                                                                                                  \
        .I_Max = USER_I_MAX_DEFAULT,                                                                                   \
        .Wm_Max = USER_WM_MAX_DEFAULT,                                                                                 \
    }

#endif /* USER_MOTOR_PARAMS_H */
