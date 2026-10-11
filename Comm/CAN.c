/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "CAN.h"

#include "Protocol.h"
#include "fdcan.h"

#define CAN_PARAM_ID ((AXDR_MSG_PARAMETER << 6U) | AXDR_NODE_ID)

/* FDCAN HAL represents DataLength as a DLC value, not a byte count. */
static const uint8_t CAN_DLC_Len[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
    12U, 16U, 20U, 24U, 32U, 48U, 64U
};

static bool CAN_Ready = false;
static bool CAN_Tx_Pending = false;
static AxDr_Msg_T CAN_Tx_Msg;

volatile uint32_t CAN_Rx_Cnt = 0U;
volatile uint32_t CAN_Tx_Cnt = 0U;
volatile uint32_t CAN_Error_Cnt = 0U;

bool CAN_Init(void)
{
    FDCAN_FilterTypeDef Filter = { 0 };

    /* Only this node's Parameter/Action requests belong to CAN in phase 1. */
    Filter.IdType = FDCAN_STANDARD_ID;
    Filter.FilterIndex = 0U;
    Filter.FilterType = FDCAN_FILTER_MASK;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1 = CAN_PARAM_ID;
    Filter.FilterID2 = 0x7FFU;

    if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter) != HAL_OK) ||
        (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
        (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 16U, 0U) != HAL_OK) ||
        (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) ||
        (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
    {
        CAN_Error_Cnt++;
        return false;
    }

    /* The CubeMX-generated IRQ priorities will also be 8 after regeneration.
     * No FDCAN notifications are enabled in this polling implementation. */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 8U, 0U);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 8U, 0U);

    CAN_Ready = true;
    return true;
}

bool CAN_Active(void)
{
    return CAN_Ready;
}

void CAN_Poll(void)
{
    FDCAN_RxHeaderTypeDef Rx = { 0 };
    FDCAN_TxHeaderTypeDef Tx = { 0 };
    uint8_t Data[AXDR_MAX_DATA_LEN];
    uint8_t Len;
    uint32_t Dlc;

    if (!CAN_Ready)
    {
        return;
    }

    /* At most one request per 2 kHz Motor Thread cycle. No CAN ISR work. */
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &Rx, Data) == HAL_OK)
        {
            if ((Rx.IdType == FDCAN_STANDARD_ID) &&
                (Rx.RxFrameType == FDCAN_DATA_FRAME) &&
                (Rx.FDFormat == FDCAN_FD_CAN) &&
                (Rx.Identifier == CAN_PARAM_ID) &&
                (Rx.DataLength <= FDCAN_DLC_BYTES_64))
            {
                Len = CAN_DLC_Len[Rx.DataLength];
                CAN_Rx_Cnt++;
                Protocol_Rx((uint16_t)Rx.Identifier, Data, Len, AXDR_LINK_CAN);
            }
        }
        else
        {
            CAN_Error_Cnt++;
        }
    }

    if (!CAN_Tx_Pending)
    {
        CAN_Tx_Pending = Protocol_Tx_Pop(AXDR_LINK_CAN, &CAN_Tx_Msg);
    }

    if (!CAN_Tx_Pending || (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U))
    {
        return;
    }

    for (Dlc = 0U; Dlc < 16U; Dlc++)
    {
        if (CAN_DLC_Len[Dlc] == CAN_Tx_Msg.Len)
        {
            break;
        }
    }

    if (Dlc == 16U)
    {
        CAN_Error_Cnt++;
        CAN_Tx_Pending = false;
        return;
    }

    Tx.Identifier = CAN_Tx_Msg.Id;
    Tx.IdType = FDCAN_STANDARD_ID;
    Tx.TxFrameType = FDCAN_DATA_FRAME;
    Tx.DataLength = Dlc;
    Tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Tx.BitRateSwitch = FDCAN_BRS_ON;
    Tx.FDFormat = FDCAN_FD_CAN;
    Tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Tx, CAN_Tx_Msg.Data) == HAL_OK)
    {
        CAN_Tx_Pending = false;
        CAN_Tx_Cnt++; /* Queued in hardware; the peer still must acknowledge it. */
    }
    else
    {
        CAN_Error_Cnt++;
    }
}
