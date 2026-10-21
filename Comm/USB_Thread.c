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
#define USB_TX_READY        1UL
#define USB_RX_READ_SIZE    64U
#define USB_RX_STREAM_SIZE  160U
#define USB_FRAME_HEAD_SIZE 7U
#define USB_FRAME_MAX_SIZE  (USB_FRAME_HEAD_SIZE + AXDR_MAX_DATA_LEN)

static TX_THREAD USB_Tx_Thread_Obj;
static TX_EVENT_FLAGS_GROUP USB_Tx_Event;
static UX_SLAVE_CLASS_CDC_ACM *volatile Cdc_Acm = UX_NULL;

/* 接收缓冲区和长度只由接收线程修改。 */
static uint8_t USB_Rx_Stream[USB_RX_STREAM_SIZE];
static uint16_t USB_Rx_Len = 0U;
static volatile ULONG USB_Tx_Pending = 0U;

static VOID USB_Tx_Entry(ULONG thread_input);
static void USB_Rx_Data(const uint8_t *Data, uint16_t Len);
static uint16_t USB_Frame_Pack(uint8_t *Buf, const AxDr_Msg_T *Msg);

void USB_Activate(void *Cdc)
{
    Cdc_Acm = (UX_SLAVE_CLASS_CDC_ACM *)Cdc;
    USB_Tx_Wake();
}

void USB_Deactivate(void)
{
    Cdc_Acm = UX_NULL;
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
            USB_Rx_Len = 0U;
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
            USB_Rx_Len = 0U;
            tx_thread_sleep(1U);
        }
    }
}

void USB_Tx_Wake(void)
{
    uint32_t Primask;

    if (__get_IPSR() == 0U)
    {
        (void)tx_event_flags_set(&USB_Tx_Event, USB_TX_READY, TX_OR);
        return;
    }

    Primask = __get_PRIMASK();
    __disable_irq();
    USB_Tx_Pending = USB_TX_READY;
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
    /* Owned only by this thread; keep the batch off its 1 KB call stack. */
    static uint8_t Buf[UX_SLAVE_REQUEST_DATA_MAX_LENGTH];
    ULONG Flags;
    ULONG Len;
    ULONG Actual;
    AxDr_Msg_T Msg;
    UX_SLAVE_CLASS_CDC_ACM *Cdc;

    TX_PARAMETER_NOT_USED(thread_input);

    while (1)
    {
        if (tx_event_flags_get(&USB_Tx_Event, USB_TX_READY, TX_OR_CLEAR, &Flags, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }

        (void)Flags;
        while (1)
        {
            Cdc = Cdc_Acm;
            if (Cdc == UX_NULL)
            {
                break;
            }

            Len = 0U;

            /* Service responses and NORMAL before continuous FAST traffic.
             * Reserve room before Pop: it releases the source buffer. */
            if (Protocol_Tx_Pop(&Msg))
            {
                Len += USB_Frame_Pack(&Buf[Len], &Msg);
            }
            if (Plot_Normal_Pop(&Msg))
            {
                Len += USB_Frame_Pack(&Buf[Len], &Msg);
            }
            while (((sizeof(Buf) - Len) >= USB_FRAME_MAX_SIZE) && Plot_Fast_Pop(&Msg))
            {
                Len += USB_Frame_Pack(&Buf[Len], &Msg);
            }

            if (Len == 0U)
            {
                break;
            }

            /* Send the available batch now, without waiting to fill it.
             * Drain remaining messages before waiting for another event. */
            Actual = 0U;
            if ((ux_device_class_cdc_acm_write(Cdc, Buf, Len, &Actual) != UX_SUCCESS) ||
                (Actual != Len))
            {
                break;
            }
        }
    }
}

static uint16_t USB_Frame_Pack(uint8_t *Buf, const AxDr_Msg_T *Msg)
{
    if (!AxDr_CANFD_Length_Valid(Msg->Len))
    {
        return 0U;
    }

    /* Preserve each AXDR frame, including CAN FD padding, within the stream. */
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

    return (uint16_t)(USB_FRAME_HEAD_SIZE + Msg->Len);
}

static void USB_Rx_Data(const uint8_t *Data, uint16_t Len)
{
    uint16_t Pos = 0U;
    uint16_t Can_ID;
    uint16_t Frame_Len;
    uint8_t Data_Len;

    if (Len > (USB_RX_STREAM_SIZE - USB_Rx_Len))
    {
        USB_Rx_Len = 0U;
    }

    memcpy(&USB_Rx_Stream[USB_Rx_Len], Data, Len);
    USB_Rx_Len = (uint16_t)(USB_Rx_Len + Len);

    while ((Pos + 4U) <= USB_Rx_Len)
    {
        const uint8_t *Frame = &USB_Rx_Stream[Pos];

        if (memcmp(Frame, "AXDR", 4U) != 0)
        {
            Pos++;
            continue;
        }

        if ((Pos + USB_FRAME_HEAD_SIZE) > USB_Rx_Len)
        {
            break;
        }

        Can_ID = (uint16_t)Frame[4] | ((uint16_t)Frame[5] << 8);
        Data_Len = Frame[6];

        if ((Can_ID > 0x07FFU) || !AxDr_CANFD_Length_Valid(Data_Len))
        {
            Pos++;
            continue;
        }

        Frame_Len = (uint16_t)(USB_FRAME_HEAD_SIZE + Data_Len);

        if ((Pos + Frame_Len) > USB_Rx_Len)
        {
            break;
        }

        Protocol_Rx(Can_ID, &Frame[USB_FRAME_HEAD_SIZE], Data_Len);
        Pos = (uint16_t)(Pos + Frame_Len);
    }

    /* 未收完整的帧或帧头留到下次，已处理的数据只在此处移除。 */
    if (Pos != 0U)
    {
        USB_Rx_Len = (uint16_t)(USB_Rx_Len - Pos);
        memmove(USB_Rx_Stream, &USB_Rx_Stream[Pos], USB_Rx_Len);
    }
}
