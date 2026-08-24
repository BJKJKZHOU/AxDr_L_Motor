/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define AXDR_NODE_ID      1U
#define AXDR_MAX_DATA_LEN 64U

#define AXDR_MSG_RESPONSE       0x02U
#define AXDR_MSG_CONTROL        0x03U
#define AXDR_MSG_PLOT           0x04U
#define AXDR_MSG_IDENTIFICATION 0x05U
#define AXDR_MSG_SENSORLESS     0x06U
#define AXDR_MSG_NORMAL_DATA    0x10U
#define AXDR_MSG_FAST_DATA      0x18U

#define AXDR_CTRL_ENABLE    0x01U
#define AXDR_CTRL_RUN       0x02U
#define AXDR_CTRL_STOP      0x03U
#define AXDR_CTRL_DISABLE   0x04U
#define AXDR_CTRL_MODE_SET  0x05U
#define AXDR_CTRL_SPEED_SET 0x06U

#define AXDR_IDENT_MODE_SET 0x01U
#define AXDR_IDENT_STATUS   0x02U
#define AXDR_IDENT_ABORT    0x03U
#define AXDR_IDENT_APPLY    0x04U

#define AXDR_IDENT_RS_LS 0x01U
#define AXDR_IDENT_FLUX  0x02U

#define AXDR_SENSORLESS_STATUS 0x02U
#define AXDR_SENSORLESS_STOP   0x03U

#define AXDR_PLOT_CONFIG 0x01U
#define AXDR_PLOT_START  0x02U
#define AXDR_PLOT_STOP   0x03U

#define AXDR_PLOT_FAST   0U
#define AXDR_PLOT_NORMAL 1U

#define AXDR_PLOT_FAST_MASK   (1U << AXDR_PLOT_FAST)
#define AXDR_PLOT_NORMAL_MASK (1U << AXDR_PLOT_NORMAL)

typedef enum
{
    AXDR_OK = 0,
    AXDR_ERR_OP,
    AXDR_ERR_LENGTH,
    AXDR_ERR_VAR_ID,
    AXDR_ERR_READ_ONLY,
    AXDR_ERR_VALUE,
    AXDR_ERR_STATE,
    AXDR_ERR_CONFIG,
    AXDR_ERR_BANDWIDTH,
    AXDR_ERR_NOT_SUPPORTED,
} AxDr_Status_e;

typedef struct
{
    uint16_t Id;
    uint8_t Len;
    uint8_t Data[AXDR_MAX_DATA_LEN];
} AxDr_Msg_T;

void Protocol_Rx(uint16_t Id, const uint8_t *Data, uint8_t Len);
bool Protocol_Tx_Pop(AxDr_Msg_T *Msg);

#endif /* PROTOCOL_H */
