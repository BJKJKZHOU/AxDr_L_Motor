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
    ENCODER_CAL,

} Motor_Mode_e;

typedef enum
{
    FAST_OFF = 0,
    FAST_CURRENT,
    FAST_VOLTAGE,

} Motor_Fast_Mode_e;

/*
 * Mechanical position represented as whole turns plus single-turn angle.
 *
 * Turn and Theta form one logical value and must be updated together whenever
 * the position is used as a command or transferred across an interface.
 * Theta uses rad in [0, 2pi).
 */
typedef struct
{
    int32_t Turn;
    float Theta;

} Motor_Position_T;

/*
 * User motion targets.
 *
 * Targets may be modified in DISABLED, ENABLED and RUN.
 * Motor state controls execution permission, not whether a target can be written.
 * Only RUN executes the saved target through the active motor mode.
 * Enable, Stop and Disable must not clear or overwrite these targets.
 *
 * Position target uses Turn + Theta to preserve single-turn angle precision.
 * Targets are stored in the user mechanical coordinate. Motor_Config.Dir maps
 * them into the internal motor coordinate only when they are executed.
 */
typedef struct
{
    float Te_Target; /* N*m, user mechanical coordinate */
    float Wm_Target; /* rad/s; signed in speed modes, magnitude in position mode */
    Motor_Position_T Position_Target;

} Motor_Cmd_T;

/*
 * 寻相确定 Enc_Dir 和 Theta_Off，统一正 Iq、正转矩和内部机械正方向。
 * 用户转向由 Motor_Config.Dir 设置；编码器方向和零位在禁能后应用。
 * Theta_e = wrap(Pp * Theta_m + Theta_Off)。
 *
 * Valid：0=未寻相，1=已寻相，2=寻相和编码器校准有效。
 * 极对数或当前使用的编码器配置变化时清零；独立重新寻相在新结果应用前保留原状态。
 * 完整编码器校准应用新系数后清零，最终寻相成功置 2；上电恢复保存值。
 */
typedef struct
{
    int8_t Enc_Dir; /* 寻相得到的方向，+1 / -1。 */
    float Theta_Off; /* 寻相得到的电角度偏置，rad。 */
    uint8_t Valid; /* 校准状态，对应参数 0x0402；非零允许有感运行。 */

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
    /* Internal mechanical position; Turn starts at zero and is not persisted. */
    int32_t Turn;
    float Theta_m; /* rad, [0, 2pi), paired with Turn */

    float Theta_e; /* rad, [0, 2pi), internal electrical angle currently used by FOC */

    float Id; /* A, internal dq coordinate */
    float Iq; /* A, +Iq produces +Te in the internal motor coordinate */

    float Ud; /* V */
    float Uq; /* V */

    float Ualpha; /* V, alpha-axis voltage command applied by the previous fast loop */
    float Ubeta; /* V, beta-axis voltage command applied by the previous fast loop */

} Motor_Run_T;

extern Motor_Cmd_T Motor_Cmd;
extern uint8_t Motor_Mode;
extern Motor_Cal_T Motor_Cal;
extern Motor_Para_T Motor_Para;
extern const Motor_Limit_T Motor_Lim;
extern Motor_Limit_T User_Lim;
extern Motor_Run_T Motor_Run;

#endif /* MOTOR_TYPE_H */
