/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "Fast_Memory.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ENC_PROTOCOL_NONE = 0,
    ENC_PROTOCOL_SPI,
    ENC_PROTOCOL_ABZ,

} Encoder_Protocol_e;

typedef enum
{
    ENC_SPI_NONE = 0,
    ENC_SPI_MT6816,
    ENC_SPI_MT6835,

} Encoder_SPI_Type_e;

typedef struct
{
    uint8_t Protocol; /* Encoder_Protocol_e，按字节保存。 */
    uint8_t SPI_Type; /* Encoder_SPI_Type_e，仅用于 SPI。 */
    uint32_t ABZ_PPR; /* 每机械圈 A/B 周期数，计数器采用四倍频。 */

} Encoder_Config_T;

typedef struct
{
    /* 编码器原始计数。 */
    uint32_t Raw;

    /* 原始机械角度，范围 [0, 2pi)，rad。 */
    float Theta_Native;

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

typedef struct
{
    float C1;
    float S1;
    float C2;
    float S2;
} Encoder_Harmonic_T;

extern Encoder_Harmonic_T Encoder_Harmonic;
FAST_CODE float Encoder_Angle_Correct(float Theta_Native, bool Cal_Valid);

void Encoder_Config_Apply(void);
/* 调用时需屏蔽采样中断；硬件计数保持不变。 */
void Encoder_Feedback_Reset(void);
FAST_CODE void Encoder_DMA_IRQHandler(void);
FAST_CODE void Encoder_Start(void);

void Encoder_Position_Read(int32_t *Turn, float *Theta);

/* 驱动传入原始角度；此处处理补偿、方向和跨圈计数。 */
FAST_CODE void Encoder_Sample_Update(uint32_t Raw, float Theta);
FAST_CODE void Encoder_Sample_Invalid(void);
FAST_CODE void Encoder_Sample_Reject(void);

#endif /* ENCODER_H */
