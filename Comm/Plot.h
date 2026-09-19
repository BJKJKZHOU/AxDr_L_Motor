/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PLOT_H
#define PLOT_H

#include <stdbool.h>
#include <stdint.h>

#include "Protocol.h"

#define AXDR_FAST_MAX_CH       8U
#define AXDR_NORMAL_MAX_CH     15U
#define AXDR_FAST_BLOCK_SAMPLE 20U

#define AXDR_PLOT_CAP_FAST   (1U << 0)
#define AXDR_PLOT_CAP_NORMAL (1U << 1)
#define AXDR_PLOT_CAP_END    0xFFU

typedef struct
{
    uint16_t Id;
    uint8_t Modes;
    float Fast_Scale;
    const volatile float *Data;
} Plot_Cap_T;

typedef struct
{
    uint8_t Config_ID;
    uint8_t Count;
    uint16_t Var[AXDR_NORMAL_MAX_CH];
    uint8_t Valid;
    uint8_t Run;
} Plot_Group_T;

extern volatile uint32_t Plot_Fast_Drop;
extern volatile uint32_t Plot_Normal_Drop;

AxDr_Status_e Plot_Config(uint8_t Group, uint8_t Config_ID, const uint16_t *Var, uint8_t Count);
AxDr_Status_e Plot_Start(uint8_t Group_Mask);
AxDr_Status_e Plot_Stop(uint8_t Group_Mask);
const Plot_Group_T *Plot_Group_Get(uint8_t Group);

uint8_t Plot_Capability_Count(void);
bool Plot_Capability_Get(uint8_t Index, Plot_Cap_T *Cap);
bool Plot_Capability_Find(uint16_t Id, Plot_Cap_T *Cap);
uint32_t Plot_Fast_Rate_Hz(void);
uint32_t Plot_Normal_Rate_Hz(void);

void Plot_Fast_Sample(void);
void Plot_Normal_Sample(void);
bool Plot_Fast_Pop(AxDr_Msg_T *Msg);
bool Plot_Normal_Pop(AxDr_Msg_T *Msg);

#endif /* PLOT_H */
