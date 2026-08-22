#ifndef SENSORLESS_START_H
#define SENSORLESS_START_H

#include <stdbool.h>
#include <stdint.h>

#include "Flux_Observer.h"
#include "PLL.h"


typedef enum
{
    SENSORLESS_ALIGN = 0,
    SENSORLESS_IF,
    SENSORLESS_OBS_WAIT,
    SENSORLESS_BLEND,
    SENSORLESS_OBS_HOLD,
    SENSORLESS_OBS_CURRENT_TRANS,
    SENSORLESS_RUN,

} Sensorless_Start_State_e;


extern Flux_Observer_T Flux_Obs;
extern PLL_T Flux_PLL;

extern volatile float Sensorless_Theta_IF;
extern volatile float Sensorless_Theta_Use;
extern volatile float Sensorless_Id_Ref;
extern volatile float Sensorless_Iq_Ref;
extern volatile float Sensorless_Blend;
extern volatile float Sensorless_We_Obs_F;
extern volatile float Sensorless_We_Ref;


void Sensorless_Start_Begin(int8_t Dir);
void Sensorless_Start_Stop(void);
void Sensorless_Speed_Target_Set(float We_Target);
bool Sensorless_Start_Active(void);
bool Sensorless_Start_Ready(void);

bool Sensorless_Start_Run(float Ia_A,
                          float Ib_A,
                          float *Id_Ref,
                          float *Iq_Ref);
Sensorless_Start_State_e Sensorless_Start_State_Get(void);


#endif /* SENSORLESS_START_H */
