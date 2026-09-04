/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protocol.h"

#include <stddef.h>
#include <string.h>

#include "Parameter.h"
#include "Plot.h"
#include "Protection.h"
#include "USB_Thread.h"
#include "motor_thread.h"

#define AXDR_RESP_NUM 8U

static AxDr_Msg_T Resp_Buf[AXDR_RESP_NUM];
static volatile uint8_t Resp_Wr = 0U;
static volatile uint8_t Resp_Rd = 0U;

static uint32_t Event_Last_Report = 0U;
static uint32_t Event_Last_Warning = 0U;
static uint32_t Event_Last_Error = 0U;
static uint32_t Event_Last_Trip = 0U;

static bool Response_Push(const AxDr_Msg_T *Msg)
{
    uint8_t Next;
    TX_INTERRUPT_SAVE_AREA

    TX_DISABLE
    Next = (uint8_t)((Resp_Wr + 1U) % AXDR_RESP_NUM);
    if (Next == Resp_Rd)
    {
        TX_RESTORE
        return false;
    }
    Resp_Buf[Resp_Wr] = *Msg;
    Resp_Wr = Next;
    TX_RESTORE

    USB_Tx_Wake(USB_TX_RESP);
    return true;
}

static void Response(uint8_t Txn,
                     uint8_t Req_Msg,
                     uint8_t Req_Op,
                     AxDr_Status_e Status,
                     const uint8_t *Data,
                     uint8_t Len)
{
    AxDr_Msg_T Msg = { 0 };

    Msg.Id = (uint16_t)((AXDR_MSG_RESPONSE << 6) | AXDR_NODE_ID);
    Msg.Len = (uint8_t)(4U + Len);
    Msg.Data[0] = Txn;
    Msg.Data[1] = Req_Msg;
    Msg.Data[2] = Req_Op;
    Msg.Data[3] = (uint8_t)Status;

    if ((Data != NULL) && (Len != 0U))
    {
        memcpy(&Msg.Data[4], Data, Len);
    }

    (void)Response_Push(&Msg);
}

static AxDr_Status_e Parameter_Status_Map(Parameter_Status_e Status)
{
    switch (Status)
    {
        case PARAM_OK:
            return AXDR_OK;
        case PARAM_ERR_ID:
            return AXDR_ERR_VAR_ID;
        case PARAM_ERR_READ_ONLY:
            return AXDR_ERR_READ_ONLY;
        case PARAM_ERR_VALUE:
        case PARAM_ERR_TYPE:
            return AXDR_ERR_VALUE;
        case PARAM_ERR_STATE:
            return AXDR_ERR_STATE;
        default:
            return AXDR_ERR_CONFIG;
    }
}

static AxDr_Status_e Motor_Parameter_Request_Status_Map(
    Motor_Parameter_Request_Status_e Status)
{
    switch (Status)
    {
        case MOTOR_PARAM_REQUEST_OK:
            return AXDR_OK;
        case MOTOR_PARAM_REQUEST_ERR_ID:
            return AXDR_ERR_VAR_ID;
        case MOTOR_PARAM_REQUEST_ERR_QUEUE:
        default:
            return AXDR_ERR_CONFIG;
    }
}

static void Parameter_Rx(const uint8_t *Data, uint8_t Len)
{
    uint8_t Txn;
    uint8_t Op;
    uint8_t Value_Size;
    uint16_t Id;
    Parameter_Type_e Type;
    Parameter_Value_T Value = { 0 };
    Parameter_Status_e Parameter_Status;
    Motor_Parameter_Request_Status_e Request_Status;
    AxDr_Status_e Status;
    uint8_t Resp[3U + sizeof(Parameter_Value_T)];
    uint8_t Id_Resp[2];

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Id = 0U;
    Status = AXDR_OK;

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_PARAM_READ)
    {
        if (Len != 4U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Id = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Parameter_Status = Parameter_Read(Id, &Type, &Value);
            Status = Parameter_Status_Map(Parameter_Status);

            if (Status == AXDR_OK)
            {
                Value_Size = Parameter_Value_Size(Type);
                Resp[0] = (uint8_t)Id;
                Resp[1] = (uint8_t)(Id >> 8);
                Resp[2] = (uint8_t)Type;
                memcpy(&Resp[3], &Value, Value_Size);
                Response(Txn,
                         AXDR_MSG_PARAMETER,
                         Op,
                         Status,
                         Resp,
                         (uint8_t)(3U + Value_Size));
                return;
            }
        }
    }
    else if (Op == AXDR_PARAM_WRITE)
    {
        if (Len < 5U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Id = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Type = (Parameter_Type_e)Data[4];

            if (Type == PARAM_ACTION)
            {
                if (Len != 5U)
                {
                    Status = AXDR_ERR_LENGTH;
                }
            }
            else
            {
                Value_Size = Parameter_Value_Size(Type);
                if ((Value_Size == 0U) || (Len != (uint8_t)(5U + Value_Size)))
                {
                    Status = (Value_Size == 0U) ? AXDR_ERR_VALUE : AXDR_ERR_LENGTH;
                }
                else
                {
                    memcpy(&Value, &Data[5], Value_Size);
                }
            }

            if (Status == AXDR_OK)
            {
                Request_Status = Motor_Parameter_Write_Request(Id,
                                                               Type,
                                                               &Value,
                                                               Txn,
                                                               true);
                Status = Motor_Parameter_Request_Status_Map(Request_Status);
                if (Status == AXDR_OK)
                {
                    return;
                }
            }
        }
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    if (Len >= 4U)
    {
        Id_Resp[0] = (uint8_t)Id;
        Id_Resp[1] = (uint8_t)(Id >> 8);
        Response(Txn, AXDR_MSG_PARAMETER, Op, Status, Id_Resp, sizeof(Id_Resp));
    }
    else
    {
        Response(Txn, AXDR_MSG_PARAMETER, Op, Status, NULL, 0U);
    }
}

void Protocol_Parameter_Write_Response(uint8_t Txn,
                                       uint16_t Parameter_Id,
                                       Parameter_Status_e Status)
{
    uint8_t Resp[2];

    Resp[0] = (uint8_t)Parameter_Id;
    Resp[1] = (uint8_t)(Parameter_Id >> 8);
    Response(Txn,
             AXDR_MSG_PARAMETER,
             AXDR_PARAM_WRITE,
             Parameter_Status_Map(Status),
             Resp,
             sizeof(Resp));
}

void Protocol_Action_Response(uint8_t Txn,
                              uint16_t Action_Id,
                              AxDr_Status_e Status)
{
    uint8_t Resp[2];

    Resp[0] = (uint8_t)Action_Id;
    Resp[1] = (uint8_t)(Action_Id >> 8);
    Response(Txn,
             AXDR_MSG_PARAMETER,
             AXDR_PARAM_WRITE,
             Status,
             Resp,
             sizeof(Resp));
}

void Protocol_Action_Complete(uint8_t Txn,
                              uint16_t Action_Id,
                              AxDr_Status_e Status)
{
    AxDr_Msg_T Msg = { 0 };

    Msg.Id = (uint16_t)((AXDR_MSG_EVENT << 6) | AXDR_NODE_ID);
    Msg.Len = 5U;
    Msg.Data[0] = AXDR_EVENT_ACTION_COMPLETE;
    Msg.Data[1] = Txn;
    Msg.Data[2] = (uint8_t)Action_Id;
    Msg.Data[3] = (uint8_t)(Action_Id >> 8);
    Msg.Data[4] = (uint8_t)Status;
    (void)Response_Push(&Msg);
}

static void Plot_Rx(const uint8_t *Data, uint8_t Len)
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

            if ((Count > AXDR_NORMAL_MAX_CH) || (Len != (uint8_t)(5U + Count * 2U)))
            {
                Status = AXDR_ERR_LENGTH;
            }
            else
            {
                for (uint8_t n = 0U; n < Count; n++)
                {
                    Var[n] = (uint16_t)Data[5U + n * 2U] |
                             ((uint16_t)Data[6U + n * 2U] << 8);
                }

                Status = Plot_Config(Group, Config_ID, Var, Count);

                if (Status == AXDR_OK)
                {
                    Resp[0] = Group;
                    Resp[1] = Config_ID;
                    Resp[2] = Count;
                    for (uint8_t n = 0U; n < Count; n++)
                    {
                        Resp[3U + n * 2U] = (uint8_t)Var[n];
                        Resp[4U + n * 2U] = (uint8_t)(Var[n] >> 8);
                    }
                    Response(Txn, AXDR_MSG_PLOT, Op, Status, Resp, (uint8_t)(3U + Count * 2U));
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
            Status = Plot_Start(Data[2]);
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
            Status = Plot_Stop(Data[2]);
        }
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    Response(Txn, AXDR_MSG_PLOT, Op, Status, NULL, 0U);
}

void Protocol_Rx(uint16_t Id, const uint8_t *Data, uint8_t Len)
{
    uint8_t Msg_Type;
    uint8_t Node;

    if ((Id > 0x07FFU) || (Len > AXDR_MAX_DATA_LEN))
    {
        return;
    }

    Msg_Type = (uint8_t)((Id >> 6) & 0x1FU);
    Node = (uint8_t)(Id & 0x3FU);

    if (Node != AXDR_NODE_ID)
    {
        return;
    }

    if (Msg_Type == AXDR_MSG_PARAMETER)
    {
        Parameter_Rx(Data, Len);
    }
    else if (Msg_Type == AXDR_MSG_PLOT)
    {
        Plot_Rx(Data, Len);
    }
}

void Protocol_Event_Poll(void)
{
    uint32_t Report;
    uint32_t Warning;
    uint32_t Error;
    uint32_t Trip;
    AxDr_Msg_T Msg = { 0 };

    Report = Protection.Report;
    Warning = Protection.Warning;
    Error = Protection.Stop;
    Trip = Protection.Trip;

    if ((Report == Event_Last_Report) &&
        (Warning == Event_Last_Warning) &&
        (Error == Event_Last_Error) &&
        (Trip == Event_Last_Trip))
    {
        return;
    }

    Msg.Id = (uint16_t)((AXDR_MSG_EVENT << 6) | AXDR_NODE_ID);
    Msg.Len = 17U;
    Msg.Data[0] = AXDR_EVENT_NOTIFY;
    memcpy(&Msg.Data[1], &Report, sizeof(Report));
    memcpy(&Msg.Data[5], &Warning, sizeof(Warning));
    memcpy(&Msg.Data[9], &Error, sizeof(Error));
    memcpy(&Msg.Data[13], &Trip, sizeof(Trip));

    if (Response_Push(&Msg))
    {
        Event_Last_Report = Report;
        Event_Last_Warning = Warning;
        Event_Last_Error = Error;
        Event_Last_Trip = Trip;
    }
}

bool Protocol_Tx_Pop(AxDr_Msg_T *Msg)
{
    TX_INTERRUPT_SAVE_AREA

    TX_DISABLE
    if (Resp_Rd == Resp_Wr)
    {
        TX_RESTORE
        return false;
    }

    *Msg = Resp_Buf[Resp_Rd];
    Resp_Rd = (uint8_t)((Resp_Rd + 1U) % AXDR_RESP_NUM);
    TX_RESTORE

    return true;
}
