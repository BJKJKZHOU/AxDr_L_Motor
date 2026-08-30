/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ENC_NONE = 0,
    ENC_MT6816,
    ENC_MT6835,

} Encoder_Type_e;

typedef struct
{
    Encoder_Type_e Type;

} Encoder_Config_T;

typedef struct
{
    /* Raw stays in the native sensor domain and is never direction-corrected. */
    uint32_t Raw;

    /* Native normalized [0, 2pi) angle before Motor_Cal.Enc_Dir is applied. */
    float Theta_Native;

    /* Theta_m is already mapped by Motor_Cal.Enc_Dir into the internal mechanical coordinate. */
    float Theta_m;

    uint32_t Err_Cnt;
    uint32_t Miss_Cnt;
    uint16_t Startup_Cnt;
    uint8_t Lost_Cnt;
    uint8_t Valid;
    uint8_t Ready;
    uint8_t Fault;

} Encoder_T;

extern Encoder_Config_T Encoder_Config;
extern volatile Encoder_T Encoder;

void Encoder_DMA_Config(void);
void Encoder_DMA_IRQHandler(void);
void Encoder_Start(void);

bool Encoder_Type_Set(Encoder_Type_e Type);

/* Drivers publish native [0, 2pi) angle here; generic Encoder.c owns direction mapping, turns and speed. */
void Encoder_Sample_Update(uint32_t Raw, float Theta);
void Encoder_Sample_Invalid(void);
void Encoder_Sample_Reject(void);

#endif /* ENCODER_H */
