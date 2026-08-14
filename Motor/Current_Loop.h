#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "PID.h"


extern PID_T Id_Ctrl;
extern PID_T Iq_Ctrl;


void Current_Loop(float Id_Ref,
                  float Iq_Ref,
                  float *Ualpha,
                  float *Ubeta);


#endif /* CURRENT_LOOP_H */
