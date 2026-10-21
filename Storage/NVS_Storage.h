/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdint.h>

/* 启动时按参数表恢复 RAM，缺失或数值不合法的记录保留原值。
 * 控制系数更新和硬件配置由调用方在加载后完成。 */
int NVS_Storage_Load_All(void);
/* 保存全部需要断电保留的参数，包括按带宽计算增益时未使用的手调 Kp/Ki。
 * 保存失败时返回错误，由调用方决定是否重试。 */
int NVS_Storage_Save_All(void);
/* 按输入顺序逐项检查并保存指定参数。
 * 遇到错误立即返回，已保存的参数不恢复。 */
int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count);

#endif /* NVS_STORAGE_H */
