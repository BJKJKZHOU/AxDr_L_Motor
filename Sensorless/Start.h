#ifndef SENSORLESS_START_H
#define SENSORLESS_START_H

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    SENSORLESS_ALIGN = 0,
    SENSORLESS_IF,

} Sensorless_Start_State_e;


void Sensorless_Start_Begin(int8_t Dir);
void Sensorless_Start_Stop(void);
bool Sensorless_Start_Active(void);
bool Sensorless_Start_Ready(void);

bool Sensorless_Start_Run(float *Id_Ref, float *Iq_Ref);
Sensorless_Start_State_e Sensorless_Start_State_Get(void);


#endif /* SENSORLESS_START_H */
