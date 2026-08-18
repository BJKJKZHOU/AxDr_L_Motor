#include "AxDr_Proto.h"

#include <string.h>

#include "AxDr_Plot.h"


#define AXDR_RESP_NUM    4U


static AxDr_Msg_T Resp_Buf[AXDR_RESP_NUM];
static volatile uint8_t Resp_Wr = 0U;
static volatile uint8_t Resp_Rd = 0U;


static void AxDr_Response_Push(const AxDr_Msg_T *Msg)
{
    uint8_t Next;

    Next = (uint8_t)((Resp_Wr + 1U) % AXDR_RESP_NUM);

    if (Next == Resp_Rd)
    {
        return;
    }

    Resp_Buf[Resp_Wr] = *Msg;
    Resp_Wr = Next;
}


static void AxDr_Response(uint8_t Txn,
                          uint8_t Req_Msg,
                          uint8_t Req_Op,
                          AxDr_Status_e Status,
                          const uint8_t *Data,
                          uint8_t Len)
{
    AxDr_Msg_T Msg = {0};

    Msg.Id = (uint16_t)((AXDR_MSG_RESPONSE << 6) | AXDR_NODE_ID);
    Msg.Len = (uint8_t)(4U + Len);
    Msg.Data[0] = Txn;
    Msg.Data[1] = Req_Msg;
    Msg.Data[2] = Req_Op;
    Msg.Data[3] = (uint8_t)Status;

    if ((Data != 0) && (Len != 0U))
    {
        memcpy(&Msg.Data[4], Data, Len);
    }

    AxDr_Response_Push(&Msg);
}


static void AxDr_Plot_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    AxDr_Status_e Status;

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_PLOT_CONFIG)
    {
        uint8_t Group;
        uint8_t Config_ID;
        uint8_t Count;
        uint16_t Var[AXDR_NORMAL_MAX_CH];
        uint8_t Resp[3U + AXDR_NORMAL_MAX_CH * 2U];

        if (Len < 5U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Group = Data[2];
            Config_ID = Data[3];
            Count = Data[4];

            if ((Count > AXDR_NORMAL_MAX_CH) ||
                (Len != (uint8_t)(5U + Count * 2U)))
            {
                Status = AXDR_ERR_LENGTH;
            }
            else
            {
                for (uint8_t n = 0U; n < Count; n++)
                {
                    Var[n] = (uint16_t)Data[5U + n * 2U]
                           | ((uint16_t)Data[6U + n * 2U] << 8);
                }

                Status = AxDr_Plot_Config(Group, Config_ID, Var, Count);

                if ((Status == AXDR_OK) && (Broadcast == 0U))
                {
                    Resp[0] = Group;
                    Resp[1] = Config_ID;
                    Resp[2] = Count;

                    for (uint8_t n = 0U; n < Count; n++)
                    {
                        Resp[3U + n * 2U] = (uint8_t)Var[n];
                        Resp[4U + n * 2U] = (uint8_t)(Var[n] >> 8);
                    }

                    AxDr_Response(Txn, AXDR_MSG_PLOT, Op, Status,
                                  Resp, (uint8_t)(3U + Count * 2U));
                    return;
                }
            }
        }
    }
    else if (Op == AXDR_PLOT_START)
    {
        if (Len != 3U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Status = AxDr_Plot_Start(Data[2]);
        }
    }
    else if (Op == AXDR_PLOT_STOP)
    {
        if (Len != 3U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Status = AxDr_Plot_Stop(Data[2]);
        }
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    if (Broadcast == 0U)
    {
        AxDr_Response(Txn, AXDR_MSG_PLOT, Op, Status, 0, 0U);
    }
}


void AxDr_Rx_Msg(uint16_t Id, const uint8_t *Data, uint8_t Len)
{
    uint8_t Msg_Type;
    uint8_t Node;

    if ((Id > 0x07FFU) || (Len > AXDR_MAX_DATA_LEN))
    {
        return;
    }

    Msg_Type = (uint8_t)((Id >> 6) & 0x1FU);
    Node = (uint8_t)(Id & 0x3FU);

    if ((Node != 0U) && (Node != AXDR_NODE_ID))
    {
        return;
    }

    if (Msg_Type == AXDR_MSG_PLOT)
    {
        AxDr_Plot_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
}


bool AxDr_Tx_Pop(AxDr_Msg_T *Msg)
{
    if (Resp_Rd == Resp_Wr)
    {
        return false;
    }

    *Msg = Resp_Buf[Resp_Rd];
    Resp_Rd = (uint8_t)((Resp_Rd + 1U) % AXDR_RESP_NUM);

    return true;
}
