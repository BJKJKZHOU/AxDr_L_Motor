#ifndef MOTOR_TYPE_H
#define MOTOR_TYPE_H

#include <stdint.h>


typedef enum
{
    SERVO_DISABLED = 0,
    SERVO_ENABLED,
    SERVO_RUN,
} Servo_State_e;


typedef enum
{
    CTRL_TORQUE = 0,
    CTRL_SPEED,
    CTRL_POSITION,
} Ctrl_Mode_e;


typedef struct
{
    float Te_Target;
    float Wm_Target;

    int32_t Pos_Turn;
    float Pos_Theta;

} Motor_Cmd_T;


typedef struct
{
    int8_t Enc_Dir;
    float Theta_Off;

} Motor_Cal_T;


typedef struct
{
    uint8_t Pp;

    float Rs;
    float Ld;
    float Lq;
    float Flux;

    float J;
    float B;

} Motor_Para_T;


typedef struct
{
    float I_Max;
    float Te_Max;
    float Wm_Max;

} Motor_Limit_T;


typedef struct
{
    int32_t Turn;

    float Theta_m;
    float Wm;

    float Theta_e;

    float Id;
    float Iq;

    float Ud;
    float Uq;

} Motor_Run_T;


extern Motor_Cal_T Motor_Cal;
extern Motor_Para_T Motor_Para;
extern Motor_Run_T Motor_Run;


#endif /* MOTOR_TYPE_H */
