/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H

#include <stdint.h>

typedef enum
{
    DISABLED = 0,
    ENABLED,
    RUN,

} Motor_State_e;

typedef enum
{
    TORQUE = 0,
    SPEED,
    POSITION,
    OPEN_LOOP,
    IDENT,
    SENSORLESS_SPEED,
    PHASE_SEARCH,

} Motor_Mode_e;

typedef enum
{
    FAST_OFF = 0,
    FAST_CURRENT,
    FAST_VOLTAGE,

} Motor_Fast_Mode_e;

/*
 * User motion targets.
 *
 * Targets may be modified in DISABLED, ENABLED and RUN.
 * Motor state controls execution permission, not whether a target can be written.
 * Only RUN executes the saved target through the active motor mode.
 * Enable, Stop and Disable must not clear or overwrite these targets.
 *
 * Position target uses Pos_Turn + Pos_Theta to preserve single-turn angle precision.
 * Targets are stored in the user mechanical coordinate. Motor_Config.Dir maps
 * them into the internal motor coordinate only when they are executed.
 */
typedef struct
{
    float Te_Target; /* N*m, user mechanical coordinate */
    float Wm_Target; /* rad/s, user mechanical coordinate */

    int32_t Pos_Turn; /* User mechanical turns */
    float Pos_Theta; /* rad, [0, 2pi), user mechanical coordinate */

} Motor_Cmd_T;

/*
 * Servo phase calibration establishes one self-consistent internal motor
 * coordinate. Enc_Dir, phase order and Theta_Off belong to that calibration
 * result and must not be changed merely to reverse the user-facing direction.
 *
 * Internal sign invariant:
 *   +Iq -> +Te -> internal positive mechanical direction.
 *
 * Motor_Config.Dir separately maps this internal direction to the user
 * mechanical coordinate. By project convention the user may choose output-
 * shaft-side CCW as positive; if the observed direction is opposite, change
 * Motor_Config.Dir only. This does not invalidate Enc_Dir or Theta_Off.
 *
 * Enc_Dir maps the encoder native angle direction into the internal motor
 * coordinate. Only +1 / -1 are valid and direction calibration may only be
 * changed while DISABLED. After direction correction, Motor_Run Theta_m,
 * Turn and Wm all belong to the internal motor coordinate.
 *
 * Theta_Off must be calibrated after Enc_Dir / phase order are established:
 * Theta_e = wrap(Pp * Theta_m + Theta_Off).
 *
 * Valid is set only after a complete servo phase calibration result is
 * accepted. Changing encoder type or pole-pair count invalidates the result.
 */
typedef struct
{
    int8_t Enc_Dir; /* +1 / -1, calibration result */
    float Theta_Off; /* rad, calibration result */
    uint8_t Valid; /* 1 only when Enc_Dir / Theta_Off match current encoder and Pp */

} Motor_Cal_T;

typedef struct
{
    uint8_t Pp; /* Pole pairs */

    float Rs; /* Ohm */
    float Ld; /* H */
    float Lq; /* H */
    float Flux; /* Wb, positive magnitude; Te = 1.5 * Pp * Flux * Iq */

    float J; /* kg*m^2 */
    float B; /* N*m/(rad/s) */

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
    float I_Max; /* A */
    float Wm_Max; /* rad/s */

} Motor_Limit_T;

typedef struct
{
    int32_t
        Turn; /* Internal mechanical turns; +1 on positive 2pi->0 wrap, -1 on negative 0->2pi wrap; starts at 0 after power-up and is not retained across power loss */

    float Theta_m; /* rad, [0, 2pi), internal mechanical coordinate */
    float Wm; /* rad/s, internal mechanical coordinate */

    float Theta_e; /* rad, [0, 2pi), internal electrical angle currently used by FOC */

    float Id; /* A, internal dq coordinate */
    float Iq; /* A, +Iq produces +Te in the internal motor coordinate */

    float Ud; /* V */
    float Uq; /* V */

    float Ualpha; /* V, alpha-axis voltage command applied by the previous fast loop */
    float Ubeta; /* V, beta-axis voltage command applied by the previous fast loop */

} Motor_Run_T;

extern Motor_Cal_T Motor_Cal;
extern Motor_Para_T Motor_Para;
extern const Motor_Limit_T Motor_Lim;
extern Motor_Limit_T User_Lim;
extern Motor_Run_T Motor_Run;

#endif /* MOTOR_TYPE_H */
