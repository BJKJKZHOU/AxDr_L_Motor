/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#include "Fast_Memory.h"

#include <stdbool.h>
#include <stdint.h>

/* 事件代码为位掩码，可同时上报多个事件；等级由所属状态字段表示。
 * 保留现有位号，避免改变通信中的事件代码。 */
typedef enum
{
    PROT_NONE = 0U,

    /* Warning：随条件恢复自动解除，不作为故障停机。 */
    PROT_UNDERVOLTAGE = (1UL << 5), /* 0x0020：欠压，禁止启动。 */

    /* Fault：关断输出并锁存，禁用后通过清故障命令解除。 */
    PROT_ENCODER = (1UL << 0),      /* 0x0001：需要编码器的模式下，编码器故障。 */
    PROT_OVERCURRENT = (1UL << 3),  /* 0x0008：实测电流超过幅值和持续时间限制。 */
    PROT_OVERVOLTAGE = (1UL << 4),  /* 0x0010：母线过压。 */
    PROT_ADC = (1UL << 9),          /* 0x0200：ADC 注入队列溢出。 */

    /* 预留事件代码，当前没有检测或上报路径，启用时明确等级。 */
    PROT_ENCODER_FIELD = (1UL << 1),     /* 0x0002：编码器磁场异常。 */
    PROT_ENCODER_OVERSPEED = (1UL << 2), /* 0x0004：编码器超速。 */
    PROT_OVERTEMP = (1UL << 6),          /* 0x0040：过温。 */
    PROT_PHASE_LOSS = (1UL << 7),        /* 0x0080：缺相。 */
    PROT_DRIVER = (1UL << 8),           /* 0x0100：驱动器故障。 */

} Protection_Event_e;

typedef struct
{
    uint32_t Report;  /* 提示位图，当前保留。 */
    uint32_t Warning; /* 当前警告位图；欠压禁止启动，不自动停机。 */
    uint32_t Fault;   /* 锁存故障位图；非零时关断输出并禁止启动。 */

    /* 保留通信中的硬件关断状态；当前板的保护故障记录在 Fault。 */
    uint32_t Trip;

} Protection_T;

extern volatile Protection_T Protection;
extern const float Protection_Vbus_Min;
extern const float Protection_Vbus_Max;

bool Protection_Clear(void);
FAST_CODE void Protection_Fast_Run(void);

void Protection_Fault_Set(uint32_t Event);

#endif /* PROTECTION_H */
