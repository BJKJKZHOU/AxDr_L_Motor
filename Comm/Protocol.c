/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protocol.h"

#include <string.h>

#include "Encoder.h"
#include "Identification.h"
#include "IF_Start.h"
#include "Motor_Control.h"
#include "Parameter.h"
#include "Plot.h"
#include "Protection.h"
#include "Sensorless.h"
#include "Servo_Phase.h"
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

    if ((Data != 0) && (Len != 0U))
    {
        memcpy(&Msg.Data[4], Data, Len);
    }

    (void)Response_Push(&Msg);
}

static bool Motor_Action_Cmd_Send(Motor_Cmd_e Cmd_Id,
                                  uint32_t Arg,
                                  uint32_t Arg2,
                                  uint8_t Txn,
                                  uint8_t Req_Msg,
                                  uint8_t Req_Op,
                                  bool Reply)
{
    Motor_Cmd_Msg_T Msg = { 0 };

    Msg.Cmd = (ULONG)Cmd_Id;
    Msg.Arg = (ULONG)Arg;
    Msg.Arg2 = (ULONG)Arg2;
    Msg.Reserved = ((ULONG)Req_Op << MOTOR_ACTION_OP_SHIFT) |
                   ((ULONG)Req_Msg << MOTOR_ACTION_MSG_SHIFT) |
                   ((ULONG)Txn << MOTOR_ACTION_TXN_SHIFT);

    if (Reply)
    {
        Msg.Reserved |= MOTOR_ACTION_RESPONSE;
    }

    return tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT) == TX_SUCCESS;
}

static bool Parameter_Cmd_Send(uint16_t Id,
                               Parameter_Type_e Type,
                               const Parameter_Value_T *Value,
                               uint8_t Txn,
                               bool Reply)
{
    Motor_Cmd_Msg_T Msg = { 0 };

    Msg.Cmd = (ULONG)MOTOR_CMD_PARAMETER_WRITE;
    Msg.Arg = (ULONG)Id;
    memcpy(&Msg.Arg2, Value, sizeof(Msg.Arg2));
    memcpy(&Msg.Arg3,
           (const uint8_t *)Value + sizeof(Msg.Arg2),
           sizeof(Msg.Arg3));
    Msg.Reserved = (ULONG)Type | ((ULONG)Txn << MOTOR_PARAM_TXN_SHIFT);

    if (Reply)
    {
        Msg.Reserved |= MOTOR_PARAM_RESPONSE;
    }

    return tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT) == TX_SUCCESS;
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

static void Control_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    Motor_Cmd_e Cmd;
    AxDr_Status_e Status;

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;
    Cmd = MOTOR_CMD_ENABLE;

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_CTRL_ENABLE)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Cmd = MOTOR_CMD_ENABLE;
        }
    }
    else if (Op == AXDR_CTRL_RUN)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Cmd = MOTOR_CMD_RUN;
        }
    }
    else if (Op == AXDR_CTRL_STOP)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Cmd = MOTOR_CMD_STOP;
        }
    }
    else if (Op == AXDR_CTRL_DISABLE)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Cmd = MOTOR_CMD_DISABLE;
        }
    }
    else if (Op == AXDR_CTRL_PHASE_STATUS)
    {
        uint8_t Resp[40];
        const Servo_Phase_Result_T *Result;

        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }

        if (Broadcast == 0U)
        {
            if (Status == AXDR_OK)
            {
                Result = Servo_Phase_Last_Result_Get();
                Resp[0] = (uint8_t)Result->State;
                Resp[1] = (uint8_t)Result->Fail;
                Resp[2] = Motor_Cal.Valid;
                Resp[3] = (uint8_t)Result->Enc_Dir;
                Resp[4] = Encoder.Ready;
                Resp[5] = Encoder.Fault;
                Resp[6] = Encoder.Valid;
                Resp[7] = (uint8_t)Encoder_Config.Type;
                memcpy(&Resp[8], &Result->Theta_Off_Pos, sizeof(float));
                memcpy(&Resp[12], &Result->Theta_Off_Neg, sizeof(float));
                memcpy(&Resp[16], &Result->Theta_Off_Error, sizeof(float));
                memcpy(&Resp[20], &Result->Theta_Off, sizeof(float));
                memcpy(&Resp[24], &Result->Pos_Move, sizeof(float));
                memcpy(&Resp[28], &Result->Neg_Move, sizeof(float));
                memcpy(&Resp[32], &Result->Verify_Move, sizeof(float));
                memcpy(&Resp[36], &Result->I_Search_A, sizeof(float));
                Response(Txn, AXDR_MSG_CONTROL, Op, Status, Resp, sizeof(Resp));
            }
            else
            {
                Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
            }
        }
        return;
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    if (Status == AXDR_OK)
    {
        if (Motor_Action_Cmd_Send(Cmd,
                                  0U,
                                  0U,
                                  Txn,
                                  AXDR_MSG_CONTROL,
                                  Op,
                                  Broadcast == 0U))
        {
            return;
        }
        Status = AXDR_ERR_CONFIG;
    }

    if (Broadcast == 0U)
    {
        Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
    }
}

static void Parameter_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    uint8_t Value_Size;
    uint16_t Id;
    Parameter_Type_e Type;
    Parameter_Value_T Value = { 0 };
    Parameter_Status_e Parameter_Status;
    AxDr_Status_e Status;
    uint8_t Resp[3U + sizeof(Parameter_Value_T)];

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_PARAM_READ)
    {
        if (Broadcast != 0U)
        {
            return;
        }

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
                Response(Txn, AXDR_MSG_PARAMETER, Op, Status, Resp, (uint8_t)(3U + Value_Size));
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
            Value_Size = Parameter_Value_Size(Type);

            if ((Value_Size == 0U) || (Len != (uint8_t)(5U + Value_Size)))
            {
                Status = (Value_Size == 0U) ? AXDR_ERR_VALUE : AXDR_ERR_LENGTH;
            }
            else
            {
                memcpy(&Value, &Data[5], Value_Size);

                if (!Parameter_Cmd_Send(Id, Type, &Value, Txn, Broadcast == 0U))
                {
                    Status = AXDR_ERR_CONFIG;
                }
                else
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

    if (Broadcast == 0U)
    {
        Response(Txn, AXDR_MSG_PARAMETER, Op, Status, 0, 0U);
    }
}

void Protocol_Parameter_Write_Response(uint8_t Txn, Parameter_Status_e Status)
{
    Response(Txn,
             AXDR_MSG_PARAMETER,
             AXDR_PARAM_WRITE,
             Parameter_Status_Map(Status),
             0,
             0U);
}

void Protocol_Action_Response(uint8_t Txn,
                              uint8_t Req_Msg,
                              uint8_t Req_Op,
                              AxDr_Status_e Status)
{
    Response(Txn, Req_Msg, Req_Op, Status, 0, 0U);
}

void Protocol_Action_Complete(uint8_t Txn,
                              uint8_t Req_Msg,
                              uint8_t Req_Op,
                              AxDr_Status_e Status)
{
    AxDr_Msg_T Msg = { 0 };

    Msg.Id = (uint16_t)((AXDR_MSG_EVENT << 6) | AXDR_NODE_ID);
    Msg.Len = 5U;
    Msg.Data[0] = AXDR_EVENT_ACTION_COMPLETE;
    Msg.Data[1] = Txn;
    Msg.Data[2] = Req_Msg;
    Msg.Data[3] = Req_Op;
    Msg.Data[4] = (uint8_t)Status;
    (void)Response_Push(&Msg);
}

static void Identification_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    Ident_Mode_e Start_Mode;
    AxDr_Status_e Status;

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;

    if (Broadcast != 0U)
    {
        return;
    }

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if ((Op == AXDR_IDENT_RS_LS_START) || (Op == AXDR_IDENT_FLUX_START))
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if ((Motor_State_Get() != ENABLED) || (Motor_Mode_Get() != IDENT))
        {
            Status = AXDR_ERR_STATE;
        }
        else
        {
            Start_Mode = (Op == AXDR_IDENT_RS_LS_START) ? IDENT_RS_LS : IDENT_FLUX;

            if (Motor_Action_Cmd_Send(MOTOR_CMD_IDENT_START,
                                      (uint32_t)Start_Mode,
                                      0U,
                                      Txn,
                                      AXDR_MSG_IDENTIFICATION,
                                      Op,
                                      true))
            {
                return;
            }
            Status = AXDR_ERR_CONFIG;
        }
    }
    else if (Op == AXDR_IDENT_ABORT)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if ((Motor_State_Get() != RUN) || (Motor_Mode_Get() != IDENT) || !Identification_Active())
        {
            Status = AXDR_ERR_STATE;
        }
        else if (Motor_Action_Cmd_Send(MOTOR_CMD_IDENT_ABORT,
                                       0U,
                                       0U,
                                       Txn,
                                       AXDR_MSG_IDENTIFICATION,
                                       Op,
                                       true))
        {
            return;
        }
        else
        {
            Status = AXDR_ERR_CONFIG;
        }
    }
    else if (Op == AXDR_IDENT_APPLY)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if ((Motor_State_Get() == RUN) ||
                 (Motor_Mode_Get() != IDENT) ||
                 !Identification_Result_Valid())
        {
            Status = AXDR_ERR_STATE;
        }
        else if (Motor_Action_Cmd_Send(MOTOR_CMD_IDENT_APPLY,
                                       0U,
                                       0U,
                                       Txn,
                                       AXDR_MSG_IDENTIFICATION,
                                       Op,
                                       true))
        {
            return;
        }
        else
        {
            Status = AXDR_ERR_CONFIG;
        }
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    Response(Txn, AXDR_MSG_IDENTIFICATION, Op, Status, 0, 0U);
}

static void Sensorless_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    AxDr_Status_e Status;

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;

    if (Broadcast != 0U)
    {
        return;
    }

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_SENSORLESS_STATUS)
    {
        uint8_t Resp[7];
        float We;

        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            We = IF_Start_We_Get();
            Resp[0] = Sensorless_Active() ? 1U : 0U;
            Resp[1] = (uint8_t)Sensorless_State_Get();
            Resp[2] = (uint8_t)IF_Start_State_Get();
            memcpy(&Resp[3], &We, sizeof(float));
            Response(Txn, AXDR_MSG_SENSORLESS, Op, AXDR_OK, Resp, sizeof(Resp));
            return;
        }
    }
    else if (Op == AXDR_SENSORLESS_STOP)
    {
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if ((Motor_State_Get() != RUN) || (Motor_Mode_Get() != SENSORLESS_SPEED))
        {
            Status = AXDR_ERR_STATE;
        }
        else if (Motor_Action_Cmd_Send(MOTOR_CMD_STOP,
                                       0U,
                                       0U,
                                       Txn,
                                       AXDR_MSG_SENSORLESS,
                                       Op,
                                       true))
        {
            return;
        }
        else
        {
            Status = AXDR_ERR_CONFIG;
        }
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    Response(Txn, AXDR_MSG_SENSORLESS, Op, Status, 0, 0U);
}

static void Event_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn;
    uint8_t Op;
    AxDr_Status_e Status;

    if (Broadcast != 0U)
    {
        return;
    }

    Txn = (Len > 0U) ? Data[0] : 0U;
    Op = (Len > 1U) ? Data[1] : 0U;
    Status = AXDR_OK;

    if (Len != 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op != AXDR_EVENT_CLEAR)
    {
        Status = AXDR_ERR_OP;
    }
    else if (Motor_State_Get() != DISABLED)
    {
        Status = AXDR_ERR_STATE;
    }
    else if (Motor_Action_Cmd_Send(MOTOR_CMD_PROTECTION_CLEAR,
                                   0U,
                                   0U,
                                   Txn,
                                   AXDR_MSG_EVENT,
                                   Op,
                                   true))
    {
        return;
    }
    else
    {
        Status = AXDR_ERR_CONFIG;
    }

    Response(Txn, AXDR_MSG_EVENT, Op, Status, 0, 0U);
}

static void Plot_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
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

    if (Broadcast == 0U)
    {
        Response(Txn, AXDR_MSG_PLOT, Op, Status, 0, 0U);
    }
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

    if ((Node != 0U) && (Node != AXDR_NODE_ID))
    {
        return;
    }

    if (Msg_Type == AXDR_MSG_CONTROL)
    {
        Control_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
    else if (Msg_Type == AXDR_MSG_PLOT)
    {
        Plot_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
    else if (Msg_Type == AXDR_MSG_IDENTIFICATION)
    {
        Identification_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
    else if (Msg_Type == AXDR_MSG_SENSORLESS)
    {
        Sensorless_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
    else if (Msg_Type == AXDR_MSG_PARAMETER)
    {
        Parameter_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    }
    else if (Msg_Type == AXDR_MSG_EVENT)
    {
        Event_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
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
