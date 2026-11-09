/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ABZ_H
#define ABZ_H

#include "Fast_Memory.h"
#include <stdint.h>

extern volatile int32_t ABZ_Index_Cnt; // 待处理的 Z 计数，-1 表示无新采样。

void ABZ_Config(void);
FAST_CODE void ABZ_Start(void);

#endif /* ABZ_H */
