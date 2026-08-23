/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "USB_Thread.h"

#include <string.h>

#include "Plot.h"
#include "Protocol.h"
#include "main.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#define USB_TX_STACK_SIZE   1024U
#define USB_TX_THREAD_PRIO  11U
#define USB_RX_READ_SIZE    64U
#define USB_RX_STREAM_SIZE  160U
#define USB_FRAME_HEAD_SIZE 7U

static TX_THREAD USB_Tx_Thread_Obj;
static TX_EVENT_FLAGS_GROUP USB_Tx_Event;
static UX_SLAVE_CLASS_CDC_ACM *volatile Cdc_Acm = UX_NULL;

static uint8_t USB_Rx_Stream[USB_RX_STREAM_SIZE];
static uint16_t USB_Rx_Len = 0U;
static volatile ULONG USB_Tx_Pending = 0U;

static VOID USB_Tx_Entry(ULONG thread_input);
static void USB_Rx_Data(const uint8_t *Data, uint16_t Len);
static UINT USB_Write(const AxDr_Msg_T *Msg);

void USB_Activate(void *Cdc)
{
    Cdc_Acm = (UX_SLAVE_CLASS_CDC_ACM *)Cdc;
    USB_Tx_Wake(USB_TX_ALL);
}

void USB_Deactivate(void)
{
    Cdc_Acm = UX_NULL;
    USB_Rx_Len = 0U;
}

UINT USB_Tx_Thread_Init(TX_BYTE_POOL *Byte_Pool)
{
    CHAR *Stack;

    if (tx_event_flags_create(&USB_Tx_Event, "USB TX Event") != TX_SUCCESS)
    {
        return TX_GROUP_ERROR;
    }

    if (tx_byte_allocate(Byte_Pool, (VOID **)&Stack, USB_TX_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&USB_Tx_Thread_Obj,
                         "USB TX",
                         USB_Tx_Entry,
                         0U,
                         Stack,
                         USB_TX_STACK_SIZE,
                         USB_TX_THREAD_PRIO,
                         USB_TX_THREAD_PRIO,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}

void USB_Rx_Thread(void)
{
    uint8_t Buf[USB_RX_READ_SIZE];
    ULONG Len;
    UINT Status;
    UX_SLAVE_CLASS_CDC_ACM *Cdc;

    while (1)
    {
        Cdc = Cdc_Acm;

        if (Cdc == UX_NULL)
        {
            tx_thread_sleep(10U);
            continue;
        }

        Len = 0U;
        Status = ux_device_class_cdc_acm_read(Cdc, Buf, USB_RX_READ_SIZE, &Len);

        if ((Status == UX_SUCCESS) && (Len != 0U))
        {
            USB_Rx_Data(Buf, (uint16_t)Len);
        }
        else if (Status != UX_SUCCESS)
        {
            tx_thread_sleep(1U);
        }
    }
}

void USB_Tx_Wake(ULONG Flag)
{
    uint32_t Primask;

    if (__get_IPSR() == 0U)
    {
        (void)tx_event_flags_set(&USB_Tx_Event, Flag, TX_OR);
        return;
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    USB_Tx_Pending |= Flag;
    __set_PRIMASK(Primask);
}

void USB_Tx_Poll(void)
{
    ULONG Pending;
    uint32_t Primask;

    Primask = __get_PRIMASK();
    __disable_irq();
    Pending = USB_Tx_Pending;
    USB_Tx_Pending = 0U;
    __set_PRIMASK(Primask);

    if (Pending != 0U)
    {
        (void)tx_event_flags_set(&USB_Tx_Event, Pending, TX_OR);
    }
}

static VOID USB_Tx_Entry(ULONG thread_input)
{
    ULONG Flags;
    AxDr_Msg_T Msg;
    UX_SLAVE_CLASS_CDC_ACM *Cdc;

    TX_PARAMETER_NOT_USED(thread_input);

    while (1)
    {
        if (tx_event_flags_get(&USB_Tx_Event, USB_TX_ALL, TX_OR_CLEAR, &Flags, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }

        (void)Flags;
        Cdc = Cdc_Acm;

        if (Cdc == UX_NULL)
        {
            continue;
        }

        while (Protocol_Tx_Pop(&Msg))
        {
            if (USB_Write(&Msg) != UX_SUCCESS)
            {
                break;
            }
        }

        while (Plot_Fast_Pop(&Msg))
        {
            if (USB_Write(&Msg) != UX_SUCCESS)
            {
                break;
            }
        }

        if (Plot_Normal_Pop(&Msg))
        {
            (void)USB_Write(&Msg);
        }
    }
}

static UINT USB_Write(const AxDr_Msg_T *Msg)
{
    uint8_t Buf[USB_FRAME_HEAD_SIZE + AXDR_MAX_DATA_LEN];
    ULONG Actual;
    ULONG Len;
    UX_SLAVE_CLASS_CDC_ACM *Cdc;

    Cdc = Cdc_Acm;

    if (Cdc == UX_NULL)
    {
        return UX_ERROR;
    }

    Buf[0] = 'A';
    Buf[1] = 'X';
    Buf[2] = 'D';
    Buf[3] = 'R';
    Buf[4] = (uint8_t)Msg->Id;
    Buf[5] = (uint8_t)(Msg->Id >> 8);
    Buf[6] = Msg->Len;

    if (Msg->Len != 0U)
    {
        memcpy(&Buf[USB_FRAME_HEAD_SIZE], Msg->Data, Msg->Len);
    }

    Len = USB_FRAME_HEAD_SIZE + Msg->Len;
    Actual = 0U;

    if (ux_device_class_cdc_acm_write(Cdc, Buf, Len, &Actual) != UX_SUCCESS)
    {
        return UX_ERROR;
    }

    return (Actual == Len) ? UX_SUCCESS : UX_ERROR;
}

static void USB_Rx_Data(const uint8_t *Data, uint16_t Len)
{
    uint16_t Magic;
    uint16_t Can_ID;
    uint16_t Frame_Len;
    uint8_t Data_Len;

    if (Len > (USB_RX_STREAM_SIZE - USB_Rx_Len))
    {
        USB_Rx_Len = 0U;
    }

    memcpy(&USB_Rx_Stream[USB_Rx_Len], Data, Len);
    USB_Rx_Len = (uint16_t)(USB_Rx_Len + Len);

    while (USB_Rx_Len >= 4U)
    {
        Magic = 0U;

        while ((Magic + 4U) <= USB_Rx_Len)
        {
            if ((USB_Rx_Stream[Magic] == 'A') && (USB_Rx_Stream[Magic + 1U] == 'X') &&
                (USB_Rx_Stream[Magic + 2U] == 'D') && (USB_Rx_Stream[Magic + 3U] == 'R'))
            {
                break;
            }

            Magic++;
        }

        if ((Magic + 4U) > USB_Rx_Len)
        {
            if (USB_Rx_Len > 3U)
            {
                memmove(USB_Rx_Stream, &USB_Rx_Stream[USB_Rx_Len - 3U], 3U);
                USB_Rx_Len = 3U;
            }
            return;
        }

        if (Magic != 0U)
        {
            memmove(USB_Rx_Stream, &USB_Rx_Stream[Magic], USB_Rx_Len - Magic);
            USB_Rx_Len = (uint16_t)(USB_Rx_Len - Magic);
        }

        if (USB_Rx_Len < USB_FRAME_HEAD_SIZE)
        {
            return;
        }

        Can_ID = (uint16_t)USB_Rx_Stream[4] | ((uint16_t)USB_Rx_Stream[5] << 8);
        Data_Len = USB_Rx_Stream[6];

        if ((Can_ID > 0x07FFU) || (Data_Len > AXDR_MAX_DATA_LEN))
        {
            memmove(USB_Rx_Stream, &USB_Rx_Stream[1], USB_Rx_Len - 1U);
            USB_Rx_Len--;
            continue;
        }

        Frame_Len = (uint16_t)(USB_FRAME_HEAD_SIZE + Data_Len);

        if (USB_Rx_Len < Frame_Len)
        {
            return;
        }

        Protocol_Rx(Can_ID, &USB_Rx_Stream[USB_FRAME_HEAD_SIZE], Data_Len);

        USB_Rx_Len = (uint16_t)(USB_Rx_Len - Frame_Len);

        if (USB_Rx_Len != 0U)
        {
            memmove(USB_Rx_Stream, &USB_Rx_Stream[Frame_Len], USB_Rx_Len);
        }
    }
}
