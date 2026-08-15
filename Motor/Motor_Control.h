#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Motor_Type.h"


void Motor_Control(void);
void Current_Ref_Get(float *Id_Ref, float *Iq_Ref);
Servo_State_e Servo_State_Get(void);

void Servo_Enable(void);
void Servo_Run(void);
void Servo_Stop(void);
void Servo_Disable(void);

void Ctrl_Mode_Set(Ctrl_Mode_e Mode);

void Torque_Target_Set(float Te);
void Speed_Target_Set(float Wm);
void Position_Target_Set(int32_t Turn, float Theta);


#endif /* MOTOR_CONTROL_H */
