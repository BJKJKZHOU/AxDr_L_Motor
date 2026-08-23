#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Motor_Type.h"
#include "Identification.h"

void Motor_Control(void);
Motor_Fast_Mode_e Motor_Fast_Run(float Ia_A,
                                 float Ib_A,
                                 float Ic_A,
                                 float *Id_Ref,
                                 float *Iq_Ref,
                                 float *Ualpha,
                                 float *Ubeta);

Motor_State_e Motor_State_Get(void);
Motor_Mode_e Motor_Mode_Get(void);

void Motor_Enable(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_Disable(void);

void Motor_Mode_Set(Motor_Mode_e Mode);
void Motor_Ident_Mode_Set(Ident_Mode_e Mode);
void Motor_Sensorless_Dir_Set(int8_t Dir);
bool Motor_Ident_Apply(void);

void Torque_Target_Set(float Te);
void Speed_Target_Set(float Wm);
void Position_Target_Set(int32_t Turn, float Theta);

#endif /* MOTOR_CONTROL_H */
