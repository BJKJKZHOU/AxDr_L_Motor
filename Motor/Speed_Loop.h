#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#include "PID.h"


extern PID_T Speed_Ctrl;

float Speed_Loop(float Wm_Ref, float Iq_Min, float Iq_Max);


#endif /* SPEED_LOOP_H */
