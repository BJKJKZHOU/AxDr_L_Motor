#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H


/* Duty inputs use the normalized range [0.0f, 1.0f]. */
void PWM_Timing_Update(void);
void PWM_Update(float DutyA, float DutyB, float DutyC);
void PWM_Enable(void);
void PWM_Disable(void);


#endif /* MOTOR_PWM_H */
