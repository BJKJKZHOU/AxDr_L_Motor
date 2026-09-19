/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Plot.h"

#include <string.h>

#include "Identification.h"
#include "Mechanical_ESO.h"
#include "Motor_ADC.h"
#include "Motor_Control.h"
#include "Motor_Type.h"
#include "Parameter.h"
#include "Sensorless.h"
#include "USB_Thread.h"
#include "control_params.h"

#define PLOT_BUF_NONE 0xFFU
#define PLOT_DEBUG_IDENT_THETA_ID 0xF001U
#define PLOT_TUNING_CAP_COUNT     5U

typedef struct
{
    const volatile float *Data;
    float Scale;
} Fast_Var_T;

typedef struct
{
    const volatile float *Data;
} Normal_Var_T;

typedef struct
{
    uint8_t Config_ID;
    uint8_t Count;
    uint16_t Var[AXDR_FAST_MAX_CH];
    Fast_Var_T Data[AXDR_FAST_MAX_CH];
    uint8_t Valid;
} Fast_Config_T;

typedef struct
{
    uint8_t Config_ID;
    uint8_t Count;
    uint16_t Var[AXDR_NORMAL_MAX_CH];
    Normal_Var_T Data[AXDR_NORMAL_MAX_CH];
    uint8_t Valid;
} Normal_Config_T;

typedef struct
{
    uint8_t Config_ID;
    uint8_t Count;
} Plot_Buffer_Meta_T;

static const Plot_Cap_T Plot_Capability[] = {
#include "Plot.generated.inc"
};

static Plot_Group_T Plot_Group[2] = { 0 };

static Fast_Config_T Fast_Config[2] = { 0 };
static Normal_Config_T Normal_Config[2] = { 0 };
static volatile uint8_t Fast_Active = 0U;
static volatile uint8_t Normal_Active = 0U;
static volatile uint8_t Fast_Pending = 0U;
static volatile uint8_t Normal_Pending = 0U;

static int16_t Fast_Buf[2][AXDR_FAST_BLOCK_SAMPLE * AXDR_FAST_MAX_CH] = { 0 };
static float Normal_Buf[2][AXDR_NORMAL_MAX_CH] = { 0 };
static Plot_Buffer_Meta_T Fast_Meta[2] = { 0 };
static Plot_Buffer_Meta_T Normal_Meta[2] = { 0 };

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

static void Plot_Fast_Group_Sync(void)
{
    const Fast_Config_T *Config = &Fast_Config[Fast_Active];

    Plot_Group[AXDR_PLOT_FAST].Config_ID = Config->Config_ID;
    Plot_Group[AXDR_PLOT_FAST].Count = Config->Count;
    Plot_Group[AXDR_PLOT_FAST].Valid = Config->Valid;

    for (uint8_t n = 0U; n < Config->Count; n++)
    {
        Plot_Group[AXDR_PLOT_FAST].Var[n] = Config->Var[n];
    }
}

static void Plot_Normal_Group_Sync(void)
{
    const Normal_Config_T *Config = &Normal_Config[Normal_Active];

    Plot_Group[AXDR_PLOT_NORMAL].Config_ID = Config->Config_ID;
    Plot_Group[AXDR_PLOT_NORMAL].Count = Config->Count;
    Plot_Group[AXDR_PLOT_NORMAL].Valid = Config->Valid;

    for (uint8_t n = 0U; n < Config->Count; n++)
    {
        Plot_Group[AXDR_PLOT_NORMAL].Var[n] = Config->Var[n];
    }
}

static void Plot_Fast_Apply_Pending(void)
{
    if (Fast_Pending == 0U)
    {
        return;
    }

    Fast_Active ^= 1U;
    Fast_Pending = 0U;
    Plot_Fast_Group_Sync();
}

static void Plot_Normal_Apply_Pending(void)
{
    if (Normal_Pending == 0U)
    {
        return;
    }

    Normal_Active ^= 1U;
    Normal_Pending = 0U;
    Plot_Normal_Group_Sync();
}

static uint8_t Plot_Generated_Capability_Count(void)
{
    return (uint8_t)(sizeof(Plot_Capability) / sizeof(Plot_Capability[0]));
}

static bool Plot_Tuning_Capability_Get(uint8_t Index, Plot_Cap_T *Cap)
{
    if (Cap == 0)
    {
        return false;
    }

    switch (Index)
    {
        case 0U:
            Cap->Id = PARAM_REF_IQ;
            Cap->Modes = AXDR_PLOT_CAP_FAST | AXDR_PLOT_CAP_NORMAL;
            Cap->Fast_Scale = 0.001f;
            Cap->Data = &Motor_Plot_Iq_Ref;
            return true;

        case 1U:
            Cap->Id = PARAM_RUN_WM;
            Cap->Modes = AXDR_PLOT_CAP_NORMAL;
            Cap->Fast_Scale = 0.0f;
            Cap->Data = &Motor_Plot_Wm;
            return true;

        case 2U:
            Cap->Id = PARAM_REF_WM;
            Cap->Modes = AXDR_PLOT_CAP_NORMAL;
            Cap->Fast_Scale = 0.0f;
            Cap->Data = &Motor_Plot_Wm_Ref;
            return true;

        case 3U:
            Cap->Id = PARAM_RUN_POSITION;
            Cap->Modes = AXDR_PLOT_CAP_NORMAL;
            Cap->Fast_Scale = 0.0f;
            Cap->Data = &Motor_Plot_Position;
            return true;

        case 4U:
            Cap->Id = PARAM_REF_POSITION;
            Cap->Modes = AXDR_PLOT_CAP_NORMAL;
            Cap->Fast_Scale = 0.0f;
            Cap->Data = &Motor_Plot_Position_Ref;
            return true;

        default:
            return false;
    }
}

uint8_t Plot_Capability_Count(void)
{
    return (uint8_t)(Plot_Generated_Capability_Count() + PLOT_TUNING_CAP_COUNT);
}

bool Plot_Capability_Get(uint8_t Index, Plot_Cap_T *Cap)
{
    uint8_t Generated_Count;

    if ((Cap == 0) || (Index >= Plot_Capability_Count()))
    {
        return false;
    }

    Generated_Count = Plot_Generated_Capability_Count();
    if (Index < Generated_Count)
    {
        *Cap = Plot_Capability[Index];
        return true;
    }

    return Plot_Tuning_Capability_Get((uint8_t)(Index - Generated_Count), Cap);
}

bool Plot_Capability_Find(uint16_t Id, Plot_Cap_T *Cap)
{
    uint8_t Count;
    Plot_Cap_T Candidate;

    if (Cap == 0)
    {
        return false;
    }

    Count = Plot_Capability_Count();
    for (uint8_t n = 0U; n < Count; n++)
    {
        if (Plot_Capability_Get(n, &Candidate) && (Candidate.Id == Id))
        {
            *Cap = Candidate;
            return true;
        }
    }

    return false;
}

static bool Plot_Capability_Resolve(uint16_t Id, Plot_Cap_T *Cap)
{
    if (Id == PLOT_DEBUG_IDENT_THETA_ID)
    {
        if (Cap == 0)
        {
            return false;
        }

        Cap->Id = PLOT_DEBUG_IDENT_THETA_ID;
        Cap->Modes = AXDR_PLOT_CAP_FAST | AXDR_PLOT_CAP_NORMAL;
        Cap->Fast_Scale = 0.0002f;
        Cap->Data = &Ident_PLL.State.Theta;
        return true;
    }

    return Plot_Capability_Find(Id, Cap);
}

uint32_t Plot_Fast_Rate_Hz(void)
{
    return (uint32_t)(CUR_FREQ_HZ_DEFAULT + 0.5f);
}

uint32_t Plot_Normal_Rate_Hz(void)
{
    return (uint32_t)(SPD_FREQ_HZ_DEFAULT * 0.5f + 0.5f);
}

AxDr_Status_e Plot_Config(uint8_t Group, uint8_t Config_ID, const uint16_t *Var, uint8_t Count)
{
    Plot_Cap_T Cap;
    uint8_t Mode;

    if (Group > AXDR_PLOT_NORMAL)
    {
        return AXDR_ERR_CONFIG;
    }

    Mode = (Group == AXDR_PLOT_FAST) ? AXDR_PLOT_CAP_FAST : AXDR_PLOT_CAP_NORMAL;

    if (Group == AXDR_PLOT_FAST)
    {
        Fast_Config_T *Config;
        uint8_t Target;

        if ((Count == 0U) || (Count > AXDR_FAST_MAX_CH))
        {
            return AXDR_ERR_CONFIG;
        }

        if ((Plot_Group[AXDR_PLOT_FAST].Run != 0U) && (Fast_Pending != 0U))
        {
            return AXDR_ERR_STATE;
        }

        Target = (uint8_t)(Fast_Active ^ 1U);
        Config = &Fast_Config[Target];
        memset(Config, 0, sizeof(*Config));
        Config->Config_ID = Config_ID;
        Config->Count = Count;

        for (uint8_t n = 0U; n < Count; n++)
        {
            if (!Plot_Capability_Resolve(Var[n], &Cap) ||
                ((Cap.Modes & Mode) == 0U) ||
                (Cap.Fast_Scale <= 0.0f))
            {
                return AXDR_ERR_VAR_ID;
            }

            Config->Var[n] = Var[n];
            Config->Data[n].Data = Cap.Data;
            Config->Data[n].Scale = Cap.Fast_Scale;
        }

        Config->Valid = 1U;

        if (Plot_Group[AXDR_PLOT_FAST].Run != 0U)
        {
            Fast_Pending = 1U;
        }
        else
        {
            Fast_Active = Target;
            Plot_Fast_Group_Sync();
            Plot_Fast_Flush();
        }

        return AXDR_OK;
    }

    {
        Normal_Config_T *Config;
        uint8_t Target;

        if ((Count == 0U) || (Count > AXDR_NORMAL_MAX_CH))
        {
            return AXDR_ERR_CONFIG;
        }

        if ((Plot_Group[AXDR_PLOT_NORMAL].Run != 0U) && (Normal_Pending != 0U))
        {
            return AXDR_ERR_STATE;
        }

        Target = (uint8_t)(Normal_Active ^ 1U);
        Config = &Normal_Config[Target];
        memset(Config, 0, sizeof(*Config));
        Config->Config_ID = Config_ID;
        Config->Count = Count;

        for (uint8_t n = 0U; n < Count; n++)
        {
            if (!Plot_Capability_Resolve(Var[n], &Cap) || ((Cap.Modes & Mode) == 0U))
            {
                return AXDR_ERR_VAR_ID;
            }

            Config->Var[n] = Var[n];
            Config->Data[n].Data = Cap.Data;
        }

        Config->Valid = 1U;

        if (Plot_Group[AXDR_PLOT_NORMAL].Run != 0U)
        {
            Normal_Pending = 1U;
        }
        else
        {
            Normal_Active = Target;
            Plot_Normal_Group_Sync();
            Plot_Normal_Flush();
        }

        return AXDR_OK;
    }
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
        Plot_Fast_Apply_Pending();
        Plot_Fast_Flush();
    }

    if ((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_NORMAL].Run = 0U;
        Plot_Normal_Apply_Pending();
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
    const Fast_Config_T *Config;

    if ((Plot_Group[AXDR_PLOT_FAST].Run == 0U) || (Plot_Group[AXDR_PLOT_FAST].Valid == 0U))
    {
        return;
    }

    Config = &Fast_Config[Fast_Active];
    Count = Config->Count;
    Fill = Fast_Fill;
    Base = (uint16_t)Fast_Sample_Cnt * Count;

    for (uint8_t n = 0U; n < Count; n++)
    {
        Fast_Buf[Fill][Base + n] = Plot_Fast_Quant(*Config->Data[n].Data, Config->Data[n].Scale);
    }

    Fast_Sample_Cnt++;

    if (Fast_Sample_Cnt < AXDR_FAST_BLOCK_SAMPLE)
    {
        return;
    }

    Fast_Sample_Cnt = 0U;
    Fast_Meta[Fill].Config_ID = Config->Config_ID;
    Fast_Meta[Fill].Count = Count;

    if (Fast_Ready != PLOT_BUF_NONE)
    {
        Plot_Fast_Drop++;
    }
    else
    {
        Fast_Ready = Fill;
        Fast_Fill ^= 1U;
        USB_Tx_Wake(USB_TX_FAST);
    }

    Plot_Fast_Apply_Pending();
}

void Plot_Normal_Sample(void)
{
    uint8_t Fill;
    const Normal_Config_T *Config;

    if ((Plot_Group[AXDR_PLOT_NORMAL].Run == 0U) || (Plot_Group[AXDR_PLOT_NORMAL].Valid == 0U))
    {
        return;
    }

    Config = &Normal_Config[Normal_Active];

    if (Normal_Ready != PLOT_BUF_NONE)
    {
        Plot_Normal_Drop++;
        Plot_Normal_Apply_Pending();
        return;
    }

    Fill = Normal_Fill;

    for (uint8_t n = 0U; n < Config->Count; n++)
    {
        Normal_Buf[Fill][n] = *Config->Data[n].Data;
    }

    Normal_Meta[Fill].Config_ID = Config->Config_ID;
    Normal_Meta[Fill].Count = Config->Count;
    Normal_Ready = Fill;
    Normal_Fill ^= 1U;
    USB_Tx_Wake(USB_TX_NORMAL);

    Plot_Normal_Apply_Pending();
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
    Plot_Buffer_Meta_T Meta;

    Ready = Fast_Ready;

    if (Ready == PLOT_BUF_NONE)
    {
        return false;
    }

    Meta = Fast_Meta[Ready];
    Count = Meta.Count;
    Max_Sample = (uint8_t)((AXDR_MAX_DATA_LEN - 4U) / (Count * 2U));
    Remain = (uint8_t)(AXDR_FAST_BLOCK_SAMPLE - Fast_Tx_Sample);
    Sample_Count = (Remain < Max_Sample) ? Remain : Max_Sample;
    Required = (uint8_t)(4U + Sample_Count * Count * 2U);

    Msg->Id = (uint16_t)((AXDR_MSG_FAST_DATA << 6) | AXDR_NODE_ID);
    Msg->Len = AxDr_CANFD_Length(Required);
    memset(Msg->Data, 0, Msg->Len);
    Msg->Data[0] = (uint8_t)Fast_Seq;
    Msg->Data[1] = (uint8_t)(Fast_Seq >> 8);
    Msg->Data[2] = Meta.Config_ID;
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
    Plot_Buffer_Meta_T Meta;

    Ready = Normal_Ready;

    if (Ready == PLOT_BUF_NONE)
    {
        return false;
    }

    Meta = Normal_Meta[Ready];
    Required = (uint8_t)(4U + Meta.Count * 4U);

    Msg->Id = (uint16_t)((AXDR_MSG_NORMAL_DATA << 6) | AXDR_NODE_ID);
    Msg->Len = AxDr_CANFD_Length(Required);
    memset(Msg->Data, 0, Msg->Len);
    Msg->Data[0] = (uint8_t)Normal_Seq;
    Msg->Data[1] = (uint8_t)(Normal_Seq >> 8);
    Msg->Data[2] = Meta.Config_ID;
    Msg->Data[3] = Meta.Count;

    Dst = 4U;

    for (uint8_t n = 0U; n < Meta.Count; n++)
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
