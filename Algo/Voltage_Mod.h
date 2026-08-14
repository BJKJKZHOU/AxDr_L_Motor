#ifndef VOLTAGE_MOD_H
#define VOLTAGE_MOD_H


void SVPWM_Calc(float Ualpha,
                 float Ubeta,
                 float Vbus,
                 float *DutyA,
                 float *DutyB,
                 float *DutyC);


#endif
