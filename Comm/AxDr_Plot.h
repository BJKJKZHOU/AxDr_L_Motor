#ifndef AXDR_PLOT_H
#define AXDR_PLOT_H

#include <stdbool.h>
#include <stdint.h>

#include "AxDr_Proto.h"


#define AXDR_FAST_MAX_CH      8U
#define AXDR_NORMAL_MAX_CH    15U
#define AXDR_FAST_BLOCK_SAMPLE 20U


typedef struct
{
    uint8_t Config_ID;
    uint8_t Count;
    uint16_t Var[AXDR_NORMAL_MAX_CH];
    uint8_t Valid;
    uint8_t Run;
} AxDr_Plot_Group_T;


extern volatile uint32_t Plot_Fast_Drop;
extern volatile uint32_t Plot_Normal_Drop;


AxDr_Status_e AxDr_Plot_Config(uint8_t Group,
                               uint8_t Config_ID,
                               const uint16_t *Var,
                               uint8_t Count);
AxDr_Status_e AxDr_Plot_Start(uint8_t Group_Mask);
AxDr_Status_e AxDr_Plot_Stop(uint8_t Group_Mask);
const AxDr_Plot_Group_T *AxDr_Plot_Group_Get(uint8_t Group);

void AxDr_Plot_Fast_Sample(void);
void AxDr_Plot_Normal_Sample(void);
bool AxDr_Plot_Fast_Pop(AxDr_Msg_T *Msg);
bool AxDr_Plot_Normal_Pop(AxDr_Msg_T *Msg);


#endif /* AXDR_PLOT_H */
