#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#include "PID.h"


extern PID_T Speed_Ctrl;

float Speed_Loop(float Wm_Ref);


#endif /* SPEED_LOOP_H */
