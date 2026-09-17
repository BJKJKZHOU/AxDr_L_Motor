/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Plot.h"

#include <string.h>

#include "Identification.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Type.h"
#include "Parameter.h"
#include "Sensorless.h"
#include "USB_Thread.h"

#define PLOT_BUF_NONE 0xFFU
#define PLOT_DEBUG_IDENT_THETA_ID 0xF001U

typedef struct
{
    const volatile float *Data;
    float Scale;
} Fast_Var_T;

typedef struct
{
    const volatile float *Data;
} Normal_Var_T;

static Plot_Group_T Plot_Group[2] = { 0 };
static Fast_Var_T Fast_Var[AXDR_FAST_MAX_CH] = { 0 };
static Normal_Var_T Normal_Var[AXDR_NORMAL_MAX_CH] = { 0 };

static int16_t Fast_Buf[2][AXDR_FAST_BLOCK_SAMPLE * AXDR_FAST_MAX_CH] = { 0 };
static float Normal_Buf[2][AXDR_NORMAL_MAX_CH] = { 0 };

static volatile uint8_t Fast_Fill = 0U;
static volatile uint8_t Fast_Ready = PLOT_BUF_NONE;
static volatile uint8_t Fast_Sample_Cnt = 0U;
static uint8_t Fast_Tx_Sample = 0U;
static uint16_t Fast_Seq = 0U;

static volatile uint8_t Normal_Fill = 0U;
static volatile uint8_t Normal_Ready = PLOT_BUF_NONE;
static uint16_t Normal_Seq = 0U;

volatile uint32_t Plot_Fast_Drop = 0U;
volatile uint32_t Plot_Normal_Drop = 0U;

static const volatile float *Plot_Data_Get(uint16_t Var_ID, float *Scale)
{
    switch (Var_ID)
    {
        case PLOT_DEBUG_IDENT_THETA_ID:
            *Scale = 0.0002f;
            return &Ident_PLL.State.Theta;
#define PARAM_GENERATE_PLOT
#include "Parameter.generated.inc"
#undef PARAM_GENERATE_PLOT
        default:
            *Scale = 0.0f;
            return 0;
    }
}

static int16_t Plot_Fast_Quant(float Value, float Scale)
{
    float Raw;

    Raw = Value / Scale;

    if (Raw >= 32767.0f)
    {
        return 32767;
    }

    if (Raw <= -32768.0f)
    {
        return -32768;
    }

    if (Raw >= 0.0f)
    {
        Raw += 0.5f;
    }
    else
    {
        Raw -= 0.5f;
    }

    return (int16_t)Raw;
}

static void Plot_Fast_Flush(void)
{
    Fast_Fill = 0U;
    Fast_Ready = PLOT_BUF_NONE;
    Fast_Sample_Cnt = 0U;
    Fast_Tx_Sample = 0U;
}

static void Plot_Normal_Flush(void)
{
    Normal_Fill = 0U;
    Normal_Ready = PLOT_BUF_NONE;
}

AxDr_Status_e Plot_Config(uint8_t Group, uint8_t Config_ID, const uint16_t *Var, uint8_t Count)
{
    uint8_t Max_Ch;
    float Scale;
    const volatile float *Data;
    Plot_Group_T *Plot;

    if (Group > AXDR_PLOT_NORMAL)
    {
        return AXDR_ERR_CONFIG;
    }

    Plot = &Plot_Group[Group];

    if (Plot->Run != 0U)
    {
        return AXDR_ERR_STATE;
    }

    Max_Ch = (Group == AXDR_PLOT_FAST) ? AXDR_FAST_MAX_CH : AXDR_NORMAL_MAX_CH;

    if ((Count == 0U) || (Count > Max_Ch))
    {
        return AXDR_ERR_CONFIG;
    }

    for (uint8_t n = 0U; n < Count; n++)
    {
        Data = Plot_Data_Get(Var[n], &Scale);

        if (Data == 0)
        {
            return AXDR_ERR_VAR_ID;
        }

        if ((Group == AXDR_PLOT_FAST) && (Scale <= 0.0f))
        {
            return AXDR_ERR_VAR_ID;
        }
    }

    Plot->Config_ID = Config_ID;
    Plot->Count = Count;

    for (uint8_t n = 0U; n < Count; n++)
    {
        Data = Plot_Data_Get(Var[n], &Scale);
        Plot->Var[n] = Var[n];

        if (Group == AXDR_PLOT_FAST)
        {
            Fast_Var[n].Data = Data;
            Fast_Var[n].Scale = Scale;
        }
        else
        {
            Normal_Var[n].Data = Data;
        }
    }

    Plot->Valid = 1U;

    if (Group == AXDR_PLOT_FAST)
    {
        Plot_Fast_Flush();
    }
    else
    {
        Plot_Normal_Flush();
    }

    return AXDR_OK;
}

AxDr_Status_e Plot_Start(uint8_t Group_Mask)
{
    if ((Group_Mask == 0U) || ((Group_Mask & ~(AXDR_PLOT_FAST_MASK | AXDR_PLOT_NORMAL_MASK)) != 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if (((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U) && (Plot_Group[AXDR_PLOT_FAST].Valid == 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if (((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U) && (Plot_Group[AXDR_PLOT_NORMAL].Valid == 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if ((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U)
    {
        Plot_Fast_Flush();
        Plot_Group[AXDR_PLOT_FAST].Run = 1U;
    }

    if ((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U)
    {
        Plot_Normal_Flush();
        Plot_Group[AXDR_PLOT_NORMAL].Run = 1U;
    }

    return AXDR_OK;
}

AxDr_Status_e Plot_Stop(uint8_t Group_Mask)
{
    if ((Group_Mask == 0U) || ((Group_Mask & ~(AXDR_PLOT_FAST_MASK | AXDR_PLOT_NORMAL_MASK)) != 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if ((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_FAST].Run = 0U;
        Plot_Fast_Flush();
    }

    if ((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_NORMAL].Run = 0U;
        Plot_Normal_Flush();
    }

    return AXDR_OK;
}

const Plot_Group_T *Plot_Group_Get(uint8_t Group)
{
    if (Group > AXDR_PLOT_NORMAL)
    {
        return 0;
    }

    return &Plot_Group[Group];
}

void Plot_Fast_Sample(void)
{
    uint16_t Base;
    uint8_t Count;
    uint8_t Fill;
    Plot_Group_T *Plot;

    Plot = &Plot_Group[AXDR_PLOT_FAST];

    if ((Plot->Run == 0U) || (Plot->Valid == 0U))
    {
        return;
    }

    Count = Plot->Count;
    Fill = Fast_Fill;
    Base = (uint16_t)Fast_Sample_Cnt * Count;

    for (uint8_t n = 0U; n < Count; n++)
    {
        Fast_Buf[Fill][Base + n] = Plot_Fast_Quant(*Fast_Var[n].Data, Fast_Var[n].Scale);
    }

    Fast_Sample_Cnt++;

    if (Fast_Sample_Cnt < AXDR_FAST_BLOCK_SAMPLE)
    {
        return;
    }

    Fast_Sample_Cnt = 0U;

    if (Fast_Ready != PLOT_BUF_NONE)
    {
        Plot_Fast_Drop++;
        return;
    }

    Fast_Ready = Fill;
    Fast_Fill ^= 1U;
    USB_Tx_Wake(USB_TX_FAST);
}

void Plot_Normal_Sample(void)
{
    uint8_t Fill;
    Plot_Group_T *Plot;

    Plot = &Plot_Group[AXDR_PLOT_NORMAL];

    if ((Plot->Run == 0U) || (Plot->Valid == 0U))
    {
        return;
    }

    if (Normal_Ready != PLOT_BUF_NONE)
    {
        Plot_Normal_Drop++;
        return;
    }

    Fill = Normal_Fill;

    for (uint8_t n = 0U; n < Plot->Count; n++)
    {
        Normal_Buf[Fill][n] = *Normal_Var[n].Data;
    }

    Normal_Ready = Fill;
    Normal_Fill ^= 1U;
    USB_Tx_Wake(USB_TX_NORMAL);
}

bool Plot_Fast_Pop(AxDr_Msg_T *Msg)
{
    uint8_t Ready;
    uint8_t Count;
    uint8_t Max_Sample;
    uint8_t Sample_Count;
    uint8_t Remain;
    uint8_t Required;
    uint16_t Src;
    uint16_t Dst;
    int16_t Raw;
    Plot_Group_T *Plot;

    Ready = Fast_Ready;

    if (Ready == PLOT_BUF_NONE)
    {
        return false;
    }

    Plot = &Plot_Group[AXDR_PLOT_FAST];
    Count = Plot->Count;
    Max_Sample = (uint8_t)((AXDR_MAX_DATA_LEN - 4U) / (Count * 2U));
    Remain = (uint8_t)(AXDR_FAST_BLOCK_SAMPLE - Fast_Tx_Sample);
    Sample_Count = (Remain < Max_Sample) ? Remain : Max_Sample;
    Required = (uint8_t)(4U + Sample_Count * Count * 2U);

    Msg->Id = (uint16_t)((AXDR_MSG_FAST_DATA << 6) | AXDR_NODE_ID);
    Msg->Len = AxDr_CANFD_Length(Required);
    memset(Msg->Data, 0, Msg->Len);
    Msg->Data[0] = (uint8_t)Fast_Seq;
    Msg->Data[1] = (uint8_t)(Fast_Seq >> 8);
    Msg->Data[2] = Plot->Config_ID;
    Msg->Data[3] = Sample_Count;

    Dst = 4U;

    for (uint8_t s = 0U; s < Sample_Count; s++)
    {
        Src = (uint16_t)(Fast_Tx_Sample + s) * Count;

        for (uint8_t n = 0U; n < Count; n++)
        {
            Raw = Fast_Buf[Ready][Src + n];
            Msg->Data[Dst++] = (uint8_t)Raw;
            Msg->Data[Dst++] = (uint8_t)((uint16_t)Raw >> 8);
        }
    }

    Fast_Seq++;
    Fast_Tx_Sample = (uint8_t)(Fast_Tx_Sample + Sample_Count);

    if (Fast_Tx_Sample >= AXDR_FAST_BLOCK_SAMPLE)
    {
        Fast_Tx_Sample = 0U;
        Fast_Ready = PLOT_BUF_NONE;
    }

    return true;
}

bool Plot_Normal_Pop(AxDr_Msg_T *Msg)
{
    uint8_t Ready;
    uint8_t Required;
    uint16_t Dst;
    uint32_t Raw;
    union
    {
        float F;
        uint32_t U;
    } Cv;
    Plot_Group_T *Plot;

    Ready = Normal_Ready;

    if (Ready == PLOT_BUF_NONE)
    {
        return false;
    }

    Plot = &Plot_Group[AXDR_PLOT_NORMAL];
    Required = (uint8_t)(4U + Plot->Count * 4U);

    Msg->Id = (uint16_t)((AXDR_MSG_NORMAL_DATA << 6) | AXDR_NODE_ID);
    Msg->Len = AxDr_CANFD_Length(Required);
    memset(Msg->Data, 0, Msg->Len);
    Msg->Data[0] = (uint8_t)Normal_Seq;
    Msg->Data[1] = (uint8_t)(Normal_Seq >> 8);
    Msg->Data[2] = Plot->Config_ID;
    Msg->Data[3] = Plot->Count;

    Dst = 4U;

    for (uint8_t n = 0U; n < Plot->Count; n++)
    {
        Cv.F = Normal_Buf[Ready][n];
        Raw = Cv.U;
        Msg->Data[Dst++] = (uint8_t)Raw;
        Msg->Data[Dst++] = (uint8_t)(Raw >> 8);
        Msg->Data[Dst++] = (uint8_t)(Raw >> 16);
        Msg->Data[Dst++] = (uint8_t)(Raw >> 24);
    }

    Normal_Seq++;
    Normal_Ready = PLOT_BUF_NONE;

    return true;
}
