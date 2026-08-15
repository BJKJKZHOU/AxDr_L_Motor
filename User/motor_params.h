#ifndef USER_MOTOR_PARAMS_H
#define USER_MOTOR_PARAMS_H

#include "Motor_Type.h"


/* 2312S motor parameters. */
#define MOTOR_PP_DEFAULT       7U
#define MOTOR_RS_DEFAULT       0.202977806f
#define MOTOR_LD_DEFAULT       0.000108778855f
#define MOTOR_LQ_DEFAULT       0.000112416135f
#define MOTOR_FLUX_DEFAULT     0.006488f
#define MOTOR_J_DEFAULT        9.08865259e-05f
#define MOTOR_B_DEFAULT        0.000188353f

#define MOTOR_ENC_DIR_DEFAULT   1
#define MOTOR_THETA_OFF_DEFAULT 0.0f


#define MOTOR_CAL_DEFAULT               \
{                                       \
    .Enc_Dir   = MOTOR_ENC_DIR_DEFAULT, \
    .Theta_Off = MOTOR_THETA_OFF_DEFAULT, \
}


#define MOTOR_PARA_DEFAULT              \
{                                       \
    .Pp   = MOTOR_PP_DEFAULT,           \
    .Rs   = MOTOR_RS_DEFAULT,           \
    .Ld   = MOTOR_LD_DEFAULT,           \
    .Lq   = MOTOR_LQ_DEFAULT,           \
    .Flux = MOTOR_FLUX_DEFAULT,         \
    .J    = MOTOR_J_DEFAULT,            \
    .B    = MOTOR_B_DEFAULT,            \
}


#endif /* USER_MOTOR_PARAMS_H */
