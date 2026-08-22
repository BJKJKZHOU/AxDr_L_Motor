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
 */
typedef struct
{
    float Te_Target;       /* N*m */
    float Wm_Target;       /* rad/s */

    int32_t Pos_Turn;      /* Mechanical turns */
    float Pos_Theta;       /* rad, [0, 2pi) */

} Motor_Cmd_T;


/*
 * Mechanical positive direction convention:
 * Viewed from the motor output-shaft side toward the motor body, CCW is positive.
 * Mechanical angle, speed and torque signs must follow this convention.
 *
 * Enc_Dir maps the encoder raw direction into this mechanical coordinate system.
 * After direction correction, Theta_m increases in the positive mechanical direction.
 * Theta_Off is the calibrated electrical angle offset used by:
 * Theta_e = wrap(Pp * Theta_m + Theta_Off).
 */
typedef struct
{
    int8_t Enc_Dir;       /* +1 / -1 */
    float Theta_Off;      /* rad */

} Motor_Cal_T;


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


typedef struct
{
    int32_t Turn;      /* Software accumulated mechanical turns; +1 on positive 2pi->0 wrap, -1 on negative 0->2pi wrap; starts at 0 after power-up and is not retained across power loss */

    float Theta_m;     /* rad, [0, 2pi) */
    float Wm;          /* rad/s */

    float Theta_e;     /* rad, [0, 2pi), electrical angle currently used by FOC */

    float Id;          /* A */
    float Iq;          /* A */

    float Ud;          /* V */
    float Uq;          /* V */

    float Ualpha;      /* V, alpha-axis voltage command applied by the previous fast loop */
    float Ubeta;       /* V, beta-axis voltage command applied by the previous fast loop */

} Motor_Run_T;


extern Motor_Cal_T Motor_Cal;
extern Motor_Para_T Motor_Para;
extern const Motor_Limit_T Motor_Lim;
extern Motor_Limit_T User_Lim;
extern Motor_Run_T Motor_Run;


#endif /* MOTOR_TYPE_H */
