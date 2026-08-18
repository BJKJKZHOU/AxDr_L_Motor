#include "AxDr_USB.h"

#include <string.h>

#include "AxDr_Proto.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"


#define USB_TX_STACK_SIZE    1024U
#define USB_TX_THREAD_PRIO   11U
#define USB_RX_READ_SIZE     64U
#define USB_RX_STREAM_SIZE   160U
#define USB_FRAME_HEAD_SIZE  7U


static TX_THREAD USB_Tx_Thread;
static UX_SLAVE_CLASS_CDC_ACM * volatile Cdc_Acm = UX_NULL;

static uint8_t USB_Rx_Stream[USB_RX_STREAM_SIZE];
static uint16_t USB_Rx_Len = 0U;


static VOID AxDr_USB_Tx_Entry(ULONG thread_input);
static void AxDr_USB_Rx_Data(const uint8_t *Data, uint16_t Len);
static UINT AxDr_USB_Write(const AxDr_Msg_T *Msg);


void AxDr_USB_Activate(void *Cdc)
{
    Cdc_Acm = (UX_SLAVE_CLASS_CDC_ACM *)Cdc;
}


void AxDr_USB_Deactivate(void)
{
    Cdc_Acm = UX_NULL;
    USB_Rx_Len = 0U;
}


UINT AxDr_USB_Tx_Init(TX_BYTE_POOL *Byte_Pool)
{
    CHAR *Stack;

    if (tx_byte_allocate(Byte_Pool, (VOID **)&Stack,
                         USB_TX_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
        return TX_POOL_ERROR;
    }

    if (tx_thread_create(&USB_Tx_Thread, "USB TX", AxDr_USB_Tx_Entry, 0U,
                         Stack, USB_TX_STACK_SIZE,
                         USB_TX_THREAD_PRIO, USB_TX_THREAD_PRIO,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return TX_THREAD_ERROR;
    }

    return TX_SUCCESS;
}


void AxDr_USB_Rx(void)
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
        Status = ux_device_class_cdc_acm_read(Cdc, Buf,
                                               USB_RX_READ_SIZE, &Len);

        if ((Status == UX_SUCCESS) && (Len != 0U))
        {
            AxDr_USB_Rx_Data(Buf, (uint16_t)Len);
        }
        else if (Status != UX_SUCCESS)
        {
            tx_thread_sleep(1U);
        }
    }
}


static VOID AxDr_USB_Tx_Entry(ULONG thread_input)
{
    AxDr_Msg_T Msg;
    UX_SLAVE_CLASS_CDC_ACM *Cdc;

    TX_PARAMETER_NOT_USED(thread_input);

    while (1)
    {
        Cdc = Cdc_Acm;

        if (Cdc != UX_NULL)
        {
            while (AxDr_Tx_Pop(&Msg))
            {
                if (AxDr_USB_Write(&Msg) != UX_SUCCESS)
                {
                    break;
                }
            }
        }

        tx_thread_sleep(1U);
    }
}


static UINT AxDr_USB_Write(const AxDr_Msg_T *Msg)
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


static void AxDr_USB_Rx_Data(const uint8_t *Data, uint16_t Len)
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
            if ((USB_Rx_Stream[Magic] == 'A') &&
                (USB_Rx_Stream[Magic + 1U] == 'X') &&
                (USB_Rx_Stream[Magic + 2U] == 'D') &&
                (USB_Rx_Stream[Magic + 3U] == 'R'))
            {
                break;
            }

            Magic++;
        }

        if ((Magic + 4U) > USB_Rx_Len)
        {
            if (USB_Rx_Len > 3U)
            {
                memmove(USB_Rx_Stream,
                        &USB_Rx_Stream[USB_Rx_Len - 3U], 3U);
                USB_Rx_Len = 3U;
            }
            return;
        }

        if (Magic != 0U)
        {
            memmove(USB_Rx_Stream, &USB_Rx_Stream[Magic],
                    USB_Rx_Len - Magic);
            USB_Rx_Len = (uint16_t)(USB_Rx_Len - Magic);
        }

        if (USB_Rx_Len < USB_FRAME_HEAD_SIZE)
        {
            return;
        }

        Can_ID = (uint16_t)USB_Rx_Stream[4]
               | ((uint16_t)USB_Rx_Stream[5] << 8);
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

        AxDr_Rx_Msg(Can_ID, &USB_Rx_Stream[USB_FRAME_HEAD_SIZE], Data_Len);

        USB_Rx_Len = (uint16_t)(USB_Rx_Len - Frame_Len);

        if (USB_Rx_Len != 0U)
        {
            memmove(USB_Rx_Stream, &USB_Rx_Stream[Frame_Len], USB_Rx_Len);
        }
    }
}
