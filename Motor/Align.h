#ifndef ALIGN_H
#define ALIGN_H

#include <stdbool.h>
#include <stdint.h>


void Align_Reset(void);
bool Align_Current(float Id_A,
                   uint32_t Hold_Cnt,
                   float *Id_Ref,
                   float *Iq_Ref);
bool Align_Voltage(float U_Step_V,
                   float U_Max_V,
                   float I_Limit_A,
                   uint32_t Hold_Cnt,
                   float Ialpha_A,
                   float *Ualpha,
                   float *Ubeta);


#endif /* ALIGN_H */
