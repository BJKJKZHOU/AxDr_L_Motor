#ifndef RS_LS_H
#define RS_LS_H

#include <stdbool.h>

typedef enum
{
    RS_LS_IDLE = 0,
    RS_LS_PROBE_RAMP,
    RS_LS_PROBE_MEASURE,
    RS_LS_ALIGN,
    RS_LS_RAMP,
    RS_LS_SETTLE,
    RS_LS_MEASURE_A,
    RS_LS_MEASURE_B,
    RS_LS_DONE,
    RS_LS_FAILED,

} Rs_Ls_State_e;

typedef struct
{
    float Rs_Ohm;
    float Ls_H;
    bool Valid;

} Rs_Ls_Result_T;

void Rs_Ls_Start(void);
void Rs_Ls_Reset(void);
void Rs_Ls_Fail(void);
bool Rs_Ls_Active(void);
void Rs_Ls_Run(float Ialpha_A, float *Ualpha_V, float *Ubeta_V);
Rs_Ls_State_e Rs_Ls_State_Get(void);
const Rs_Ls_Result_T *Rs_Ls_Result_Get(void);

#endif /* RS_LS_H */
