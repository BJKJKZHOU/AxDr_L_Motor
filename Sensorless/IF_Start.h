#ifndef IF_START_H
#define IF_START_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IF_ACCEL = 0,
    IF_HOLD,

} IF_State_e;

void IF_Start_Reset(float Theta_Start, int8_t Dir);
void IF_Start_Target_Set(float We_Target);
bool IF_Start_Run(float *Theta_e, float *Id_Ref, float *Iq_Ref);
IF_State_e IF_Start_State_Get(void);
float IF_Start_We_Get(void);

#endif /* IF_START_H */
