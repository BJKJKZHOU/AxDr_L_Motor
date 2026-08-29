/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Protocol.h"

#include <string.h>

#include "Flux.h"
#include "Identification.h"
#include "IF_Start.h"
#include "Motor_Control.h"
#include "Motor_Para.h"
#include "Motor_Type.h"
#include "Plot.h"
#include "Rs_Ls.h"
#include "Sensorless.h"
#include "USB_Thread.h"
#include "motor_thread.h"

#define AXDR_RESP_NUM 4U

static AxDr_Msg_T Resp_Buf[AXDR_RESP_NUM];
static volatile uint8_t Resp_Wr = 0U;
static volatile uint8_t Resp_Rd = 0U;

static bool Response_Push(const AxDr_Msg_T *Msg)
{
    uint8_t Next = (uint8_t)((Resp_Wr + 1U) % AXDR_RESP_NUM);

    if (Next == Resp_Rd)
    {
        return false;
    }

    Resp_Buf[Resp_Wr] = *Msg;
    Resp_Wr = Next;
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

static bool Motor_Cmd_Send(Motor_Cmd_e Cmd_Id, uint32_t Arg)
{
    Motor_Cmd_Msg_T Msg;

    Msg.Cmd = (ULONG)Cmd_Id;
    Msg.Arg = (ULONG)Arg;
    return tx_queue_send(&Motor_Cmd_Q, &Msg, TX_NO_WAIT) == TX_SUCCESS;
}

static bool Motor_Para_Value_Valid(float Value)
{
    return __builtin_isfinite(Value) && (Value > 0.0f);
}

static AxDr_Status_e Known_Motor_Para_Set(const uint8_t *Payload)
{
    uint8_t Pp;
    float Rs;
    float Ld;
    float Lq;
    float Flux;

    if (Motor_State_Get() != DISABLED)
    {
        return AXDR_ERR_STATE;
    }

    Pp = Payload[0];
    memcpy(&Rs, &Payload[1], sizeof(float));
    memcpy(&Ld, &Payload[5], sizeof(float));
    memcpy(&Lq, &Payload[9], sizeof(float));
    memcpy(&Flux, &Payload[13], sizeof(float));

    if ((Pp == 0U) || !Motor_Para_Value_Valid(Rs) || !Motor_Para_Value_Valid(Ld) ||
        !Motor_Para_Value_Valid(Lq) || !Motor_Para_Value_Valid(Flux))
    {
        return AXDR_ERR_VALUE;
    }

    Motor_Para.Pp = Pp;
    Motor_Para.Rs = Rs;
    Motor_Para.Ld = Ld;
    Motor_Para.Lq = Lq;
    Motor_Para.Flux = Flux;
    Motor_Para_Changed(MOTOR_PARA_PP | MOTOR_PARA_RL | MOTOR_PARA_FLUX);
    return AXDR_OK;
}

static void Control_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn = (Len > 0U) ? Data[0] : 0U;
    uint8_t Op = (Len > 1U) ? Data[1] : 0U;
    uint8_t Mode;
    uint32_t Arg;
    float Value;
    Motor_Cmd_e Cmd = MOTOR_CMD_ENABLE;
    AxDr_Status_e Status = AXDR_OK;

    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_CTRL_ENABLE)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else Cmd = MOTOR_CMD_ENABLE;
    }
    else if (Op == AXDR_CTRL_RUN)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else Cmd = MOTOR_CMD_RUN;
    }
    else if (Op == AXDR_CTRL_STOP)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else Cmd = MOTOR_CMD_STOP;
    }
    else if (Op == AXDR_CTRL_DISABLE)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else Cmd = MOTOR_CMD_DISABLE;
    }
    else if (Op == AXDR_CTRL_MODE_SET)
    {
        if (Len != 3U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if (Motor_State_Get() != DISABLED)
        {
            Status = AXDR_ERR_STATE;
        }
        else
        {
            Mode = Data[2];
            if (Mode > (uint8_t)SENSORLESS_SPEED) Status = AXDR_ERR_VALUE;
            else if (!Motor_Cmd_Send(MOTOR_CMD_MODE_SET, Mode)) Status = AXDR_ERR_CONFIG;
        }
        if (Broadcast == 0U) Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
        return;
    }
    else if ((Op == AXDR_CTRL_SPEED_SET) || (Op == AXDR_CTRL_I_LIMIT_SET))
    {
        if (Len != 6U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            memcpy(&Value, &Data[2], sizeof(Value));
            if (!__builtin_isfinite(Value))
            {
                Status = AXDR_ERR_VALUE;
            }
            else if (Op == AXDR_CTRL_I_LIMIT_SET)
            {
                if ((Motor_State_Get() != DISABLED) || (Value <= 0.0f) || (Value > Motor_Lim.I_Max))
                {
                    Status = (Motor_State_Get() != DISABLED) ? AXDR_ERR_STATE : AXDR_ERR_VALUE;
                }
                else
                {
                    memcpy(&Arg, &Value, sizeof(Arg));
                    if (!Motor_Cmd_Send(MOTOR_CMD_I_LIMIT_SET, Arg)) Status = AXDR_ERR_CONFIG;
                }
            }
            else
            {
                memcpy(&Arg, &Value, sizeof(Arg));
                if (!Motor_Cmd_Send(MOTOR_CMD_SPEED_SET, Arg)) Status = AXDR_ERR_CONFIG;
            }
        }
        if (Broadcast == 0U) Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
        return;
    }
    else if (Op == AXDR_CTRL_PP_SET)
    {
        if (Len != 3U) Status = AXDR_ERR_LENGTH;
        else if (Motor_State_Get() != DISABLED) Status = AXDR_ERR_STATE;
        else if (Data[2] == 0U) Status = AXDR_ERR_VALUE;
        else if (!Motor_Cmd_Send(MOTOR_CMD_PP_SET, Data[2])) Status = AXDR_ERR_CONFIG;
        if (Broadcast == 0U) Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
        return;
    }
    else if (Op == AXDR_CTRL_PP_GET)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        if (Broadcast == 0U)
        {
            Response(Txn, AXDR_MSG_CONTROL, Op, Status,
                     (Status == AXDR_OK) ? &Motor_Para.Pp : 0,
                     (Status == AXDR_OK) ? 1U : 0U);
        }
        return;
    }
    else if (Op == AXDR_CTRL_MOTOR_PARA_SET)
    {
        if (Len != 19U) Status = AXDR_ERR_LENGTH;
        else Status = Known_Motor_Para_Set(&Data[2]);
        if (Broadcast == 0U) Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
        return;
    }
    else if (Op == AXDR_CTRL_MOTOR_PARA_GET)
    {
        uint8_t Resp[17];
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Resp[0] = Motor_Para.Pp;
            memcpy(&Resp[1], &Motor_Para.Rs, sizeof(float));
            memcpy(&Resp[5], &Motor_Para.Ld, sizeof(float));
            memcpy(&Resp[9], &Motor_Para.Lq, sizeof(float));
            memcpy(&Resp[13], &Motor_Para.Flux, sizeof(float));
        }
        if (Broadcast == 0U)
        {
            Response(Txn, AXDR_MSG_CONTROL, Op, Status,
                     (Status == AXDR_OK) ? Resp : 0,
                     (Status == AXDR_OK) ? sizeof(Resp) : 0U);
        }
        return;
    }
    else
    {
        Status = AXDR_ERR_OP;
    }

    if ((Status == AXDR_OK) && !Motor_Cmd_Send(Cmd, 0U)) Status = AXDR_ERR_CONFIG;
    if (Broadcast == 0U) Response(Txn, AXDR_MSG_CONTROL, Op, Status, 0, 0U);
}

static void Identification_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn = (Len > 0U) ? Data[0] : 0U;
    uint8_t Op = (Len > 1U) ? Data[1] : 0U;
    AxDr_Status_e Status = AXDR_OK;

    if (Broadcast != 0U) return;
    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_IDENT_MODE_SET)
    {
        if (Len != 3U) Status = AXDR_ERR_LENGTH;
        else if ((Data[2] != AXDR_IDENT_RS_LS) && (Data[2] != AXDR_IDENT_FLUX)) Status = AXDR_ERR_NOT_SUPPORTED;
        else if (Motor_State_Get() != DISABLED) Status = AXDR_ERR_STATE;
        else if (!Motor_Cmd_Send(MOTOR_CMD_IDENT_SET, Data[2])) Status = AXDR_ERR_CONFIG;
    }
    else if (Op == AXDR_IDENT_STATUS)
    {
        uint8_t Resp[16];
        uint8_t Resp_Len;
        Ident_Mode_e Mode;
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Mode = Identification_Mode_Get();
            Resp[0] = (uint8_t)Mode;
            Resp[1] = (uint8_t)Identification_State_Get();
            Resp[2] = Identification_Stage_Get();
            if (Mode == IDENT_FLUX)
            {
                const Flux_Result_T *Result = Flux_Result_Get();
                Resp[3] = Result->Valid ? 1U : 0U;
                memcpy(&Resp[4], &Result->Flux_Wb, sizeof(float));
                memcpy(&Resp[8], &Result->Point_Max_Rel_Dev, sizeof(float));
                memcpy(&Resp[12], &Result->U_Util_Max, sizeof(float));
                Resp_Len = 16U;
            }
            else
            {
                const Rs_Ls_Result_T *Result = Rs_Ls_Result_Get();
                Resp[3] = Result->Valid ? 1U : 0U;
                memcpy(&Resp[4], &Result->Rs_Ohm, sizeof(float));
                memcpy(&Resp[8], &Result->Ls_H, sizeof(float));
                Resp_Len = 12U;
            }
            Response(Txn, AXDR_MSG_IDENTIFICATION, Op, AXDR_OK, Resp, Resp_Len);
            return;
        }
    }
    else if (Op == AXDR_IDENT_FLUX_POINT_GET)
    {
        uint8_t Resp[25];
        const Flux_Result_T *Result;
        const Flux_Point_T *Point_Result;
        uint8_t Point_Index;
        if (Len != 3U) Status = AXDR_ERR_LENGTH;
        else if (Identification_Mode_Get() != IDENT_FLUX) Status = AXDR_ERR_STATE;
        else
        {
            Point_Index = Data[2];
            Result = Flux_Result_Get();
            if (Point_Index >= FLUX_POINT_NUM) Status = AXDR_ERR_VALUE;
            else if (Point_Index >= Result->Point_Num) Status = AXDR_ERR_STATE;
            else
            {
                Point_Result = &Result->Point[Point_Index];
                Resp[0] = Point_Index;
                memcpy(&Resp[1], &Point_Result->We, sizeof(float));
                memcpy(&Resp[5], &Point_Result->E, sizeof(float));
                memcpy(&Resp[9], &Point_Result->Id, sizeof(float));
                memcpy(&Resp[13], &Point_Result->Iq, sizeof(float));
                memcpy(&Resp[17], &Point_Result->Ud, sizeof(float));
                memcpy(&Resp[21], &Point_Result->Uq, sizeof(float));
                Response(Txn, AXDR_MSG_IDENTIFICATION, Op, AXDR_OK, Resp, sizeof(Resp));
                return;
            }
        }
    }
    else if (Op == AXDR_IDENT_ABORT)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else if ((Motor_State_Get() != RUN) || (Motor_Mode_Get() != IDENT) || !Identification_Active()) Status = AXDR_ERR_STATE;
        else if (!Motor_Cmd_Send(MOTOR_CMD_STOP, 0U)) Status = AXDR_ERR_CONFIG;
    }
    else if (Op == AXDR_IDENT_APPLY)
    {
        bool Result_Valid;
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else if ((Motor_State_Get() == RUN) || (Identification_State_Get() != IDENT_DONE)) Status = AXDR_ERR_STATE;
        else
        {
            Result_Valid = (Identification_Mode_Get() == IDENT_FLUX) ? Flux_Result_Get()->Valid : Rs_Ls_Result_Get()->Valid;
            if (!Result_Valid) Status = AXDR_ERR_CONFIG;
            else if (!Motor_Cmd_Send(MOTOR_CMD_IDENT_APPLY, 0U)) Status = AXDR_ERR_CONFIG;
        }
    }
    else Status = AXDR_ERR_OP;

    Response(Txn, AXDR_MSG_IDENTIFICATION, Op, Status, 0, 0U);
}

static void Sensorless_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn = (Len > 0U) ? Data[0] : 0U;
    uint8_t Op = (Len > 1U) ? Data[1] : 0U;
    AxDr_Status_e Status = AXDR_OK;

    if (Broadcast != 0U) return;
    if (Len < 2U)
    {
        Status = AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_SENSORLESS_STATUS)
    {
        uint8_t Resp[38];
        const Sensorless_Diag_T *Diag;
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Diag = Sensorless_Diag_Get();
            Resp[0] = Sensorless_Active() ? 1U : 0U;
            Resp[1] = Sensorless_Ready() ? 1U : 0U;
            Resp[2] = (uint8_t)Sensorless_State_Get();
            Resp[3] = (uint8_t)IF_Start_State_Get();
            memcpy(&Resp[4], &Diag->We_IF, sizeof(float));
            memcpy(&Resp[8], &Diag->We_Obs_F, sizeof(float));
            memcpy(&Resp[12], &Diag->We_Err, sizeof(float));
            memcpy(&Resp[16], &Diag->PLL_Err, sizeof(float));
            memcpy(&Resp[20], &Diag->Flux_Ratio, sizeof(float));
            memcpy(&Resp[24], &Diag->Theta_Err, sizeof(float));
            memcpy(&Resp[28], &Diag->Stable_s, sizeof(float));
            memcpy(&Resp[32], &Diag->Stable_Max_s, sizeof(float));
            Resp[36] = Diag->Reject;
            Resp[37] = Diag->Reject_Seen;
            Response(Txn, AXDR_MSG_SENSORLESS, Op, AXDR_OK, Resp, sizeof(Resp));
            return;
        }
    }
    else if (Op == AXDR_SENSORLESS_STOP)
    {
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else if ((Motor_State_Get() != RUN) || (Motor_Mode_Get() != SENSORLESS_SPEED)) Status = AXDR_ERR_STATE;
        else if (!Motor_Cmd_Send(MOTOR_CMD_STOP, 0U)) Status = AXDR_ERR_CONFIG;
    }
    else if (Op == AXDR_SENSORLESS_SHADOW_SET)
    {
        if (Len != 3U) Status = AXDR_ERR_LENGTH;
        else if (Motor_State_Get() != DISABLED) Status = AXDR_ERR_STATE;
        else if (Data[2] > 1U) Status = AXDR_ERR_VALUE;
        else if (!Sensorless_Shadow_Set(Data[2] != 0U)) Status = AXDR_ERR_STATE;
    }
    else if (Op == AXDR_SENSORLESS_SHADOW_GET)
    {
        uint8_t Shadow;
        if (Len != 2U) Status = AXDR_ERR_LENGTH;
        else
        {
            Shadow = Sensorless_Shadow_Get() ? 1U : 0U;
            Response(Txn, AXDR_MSG_SENSORLESS, Op, AXDR_OK, &Shadow, 1U);
            return;
        }
    }
    else if (Op == AXDR_SENSORLESS_PLL_BW_SET)
    {
        float Bw_Hz;
        if (Len != 6U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else if (Motor_State_Get() != DISABLED)
        {
            Status = AXDR_ERR_STATE;
        }
        else
        {
            memcpy(&Bw_Hz, &Data[2], sizeof(float));
            if (!Sensorless_PLL_BW_Set(Bw_Hz)) Status = AXDR_ERR_VALUE;
        }
    }
    else if (Op == AXDR_SENSORLESS_PLL_BW_GET)
    {
        float Bw_Hz;
        if (Len != 2U)
        {
            Status = AXDR_ERR_LENGTH;
        }
        else
        {
            Bw_Hz = Sensorless_PLL_BW_Get();
            Response(Txn, AXDR_MSG_SENSORLESS, Op, AXDR_OK, (const uint8_t *)&Bw_Hz, sizeof(Bw_Hz));
            return;
        }
    }
    else Status = AXDR_ERR_OP;

    Response(Txn, AXDR_MSG_SENSORLESS, Op, Status, 0, 0U);
}

static void Plot_Rx(const uint8_t *Data, uint8_t Len, uint8_t Broadcast)
{
    uint8_t Txn = (Len > 0U) ? Data[0] : 0U;
    uint8_t Op = (Len > 1U) ? Data[1] : 0U;
    AxDr_Status_e Status = AXDR_OK;

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
                    Var[n] = (uint16_t)Data[5U + n * 2U] | ((uint16_t)Data[6U + n * 2U] << 8);
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
        Status = (Len == 3U) ? Plot_Start(Data[2]) : AXDR_ERR_LENGTH;
    }
    else if (Op == AXDR_PLOT_STOP)
    {
        Status = (Len == 3U) ? Plot_Stop(Data[2]) : AXDR_ERR_LENGTH;
    }
    else Status = AXDR_ERR_OP;

    if (Broadcast == 0U) Response(Txn, AXDR_MSG_PLOT, Op, Status, 0, 0U);
}

void Protocol_Rx(uint16_t Id, const uint8_t *Data, uint8_t Len)
{
    uint8_t Msg_Type;
    uint8_t Node;

    if ((Id > 0x07FFU) || (Len > AXDR_MAX_DATA_LEN)) return;

    Msg_Type = (uint8_t)((Id >> 6) & 0x1FU);
    Node = (uint8_t)(Id & 0x3FU);
    if ((Node != 0U) && (Node != AXDR_NODE_ID)) return;

    if (Msg_Type == AXDR_MSG_CONTROL) Control_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    else if (Msg_Type == AXDR_MSG_PLOT) Plot_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    else if (Msg_Type == AXDR_MSG_IDENTIFICATION) Identification_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
    else if (Msg_Type == AXDR_MSG_SENSORLESS) Sensorless_Rx(Data, Len, (Node == 0U) ? 1U : 0U);
}

bool Protocol_Tx_Pop(AxDr_Msg_T *Msg)
{
    if (Resp_Rd == Resp_Wr) return false;

    *Msg = Resp_Buf[Resp_Rd];
    Resp_Rd = (uint8_t)((Resp_Rd + 1U) % AXDR_RESP_NUM);
    return true;
}
