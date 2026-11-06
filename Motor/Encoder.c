/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Encoder.h"
#include "Enc_Home.h"

#include "ABZ.h"
#include "MT6816.h"
#include "MT6835.h"
#include "Math.h"
#include "Sin_LUT.h"
#include "Mechanical_ESO.h"
#include "Motor_Type.h"
#include "control_params.h"
#include "main.h"
#include "motor_params.h"
#include "Encoder_Port.h"

#define ENC_READY_VALID_CNT      16U
#define ENC_RUNTIME_INVALID_CNT  3U
#define ENC_STARTUP_TIMEOUT_CNT  ((uint16_t)(CUR_FREQ_HZ_DEFAULT * 0.05f))

Encoder_Config_T Encoder_Config = ENCODER_CONFIG_DEFAULT;
volatile Encoder_T Encoder = { 0 };
Encoder_Harmonic_T Encoder_Harmonic = { 0 };

FAST_CODE float Encoder_Angle_Correct(float Theta_Native, bool Cal_Valid)
{
    float Sin;
    float Cos;
    float Error;

    if (!Cal_Valid ||
        ((Encoder_Harmonic.C1 == 0.0f) && (Encoder_Harmonic.S1 == 0.0f) &&
         (Encoder_Harmonic.C2 == 0.0f) && (Encoder_Harmonic.S2 == 0.0f)))
    {
        return Theta_Native;
    }

    SinCos(Theta_Native, &Sin, &Cos);
    Error = Encoder_Harmonic.C1 * Cos + Encoder_Harmonic.S1 * Sin +
            Encoder_Harmonic.C2 * (Cos * Cos - Sin * Sin) +
            Encoder_Harmonic.S2 * (2.0f * Sin * Cos);
    return Angle_Wrap(Theta_Native - Error);
}

static uint8_t Pos_Valid = 0U;
static uint8_t Ready_Cnt = 0U;

static void (*Drv_Start)(void) = 0;
static void (*Drv_IRQHandler)(void) = 0;

void Encoder_Position_Reset(void)
{
    Enc_Home.Valid = 0U;
    Pos_Valid = 0U;
    Encoder.Valid = 0U;
    Motor_Run.Turn = 0;
    Motor_Run.Theta_m = 0.0f;
    Mechanical_ESO.State = (Mechanical_ESO_State_T){ 0 };
}

static void Startup_Count(void)
{
    if (Encoder.Fault != 0U)
    {
        return;
    }

    Encoder.Startup_Cnt++;

    if (Encoder.Startup_Cnt >= ENC_STARTUP_TIMEOUT_CNT)
    {
        Encoder.Fault = 1U;
    }
}

void Encoder_Config_Apply(void)
{
    uint32_t Primask;

    /* 切换驱动时暂停中断，避免采样和 DMA 同时访问外设。 */
    Primask = __get_PRIMASK();
    __disable_irq();
    Encoder_Port_Release();
    Ready_Cnt = 0U;
    Encoder_Position_Reset();
    Encoder = (Encoder_T){ 0 };

    Drv_Start = 0;
    Drv_IRQHandler = 0;
    switch ((Encoder_Protocol_e)Encoder_Config.Protocol)
    {
        case ENC_PROTOCOL_SPI:
            Encoder_Port_SPI();
            if (Encoder_Config.SPI_Type == ENC_SPI_MT6816)
            {
                Drv_Start = MT6816_Start;
                Drv_IRQHandler = MT6816_IRQHandler;
                MT6816_Config();
            }
            else
            {
                Drv_Start = MT6835_Start;
                Drv_IRQHandler = MT6835_IRQHandler;
                MT6835_Config();
            }
            break;

        case ENC_PROTOCOL_ABZ:
            Drv_Start = ABZ_Start;
            ABZ_Config();
            break;

        case ENC_PROTOCOL_NONE:
        default:
            break;
    }
    __set_PRIMASK(Primask);
}

void Encoder_Position_Read(int32_t *Turn, float *Theta)
{
    uint32_t Primask;

    /* 一起读取圈数和角度，避免采样中断在两次读取之间更新位置。 */
    Primask = __get_PRIMASK();
    __disable_irq();
    *Turn = Motor_Run.Turn;
    *Theta = Motor_Run.Theta_m;
    __set_PRIMASK(Primask);
}

void Encoder_Start(void)
{
    if (Drv_Start != 0)
    {
        Drv_Start();
    }
}

void Encoder_DMA_IRQHandler(void)
{
    if (Drv_IRQHandler != 0)
    {
        Drv_IRQHandler();
    }
}

void Encoder_Sample_Invalid(void)
{
    Encoder_Sample_Reject();

    if ((Encoder.Ready != 0U) && (Encoder.Fault == 0U))
    {
        Encoder.Lost_Cnt++;

        if (Encoder.Lost_Cnt >= ENC_RUNTIME_INVALID_CNT)
        {
            Encoder.Fault = 1U;
        }
    }
}

void Encoder_Sample_Reject(void)
{
    Encoder.Valid = 0U;

    if (Encoder.Ready == 0U)
    {
        Ready_Cnt = 0U;
        Startup_Count();
    }
}

void Encoder_Sample_Update(uint32_t Raw, float Theta)
{
    float Delta;

    Encoder.Raw = Raw;
    Encoder.Theta_Native = Theta;
    Theta = Encoder_Angle_Correct(Theta, Motor_Cal.Valid == 2U);

    if (Motor_Cal.Enc_Dir < 0)
    {
        Theta = Angle_Wrap(-Theta);
    }

    if (Pos_Valid != 0U)
    {
        /* 更新前的单圈角度就是上一拍角度，用于判断是否跨圈。 */
        Delta = Theta - Motor_Run.Theta_m;

        if (Delta < -PI_F)
        {
            Motor_Run.Turn++;
        }
        else if (Delta > PI_F)
        {
            Motor_Run.Turn--;
        }
    }
    else
    {
        /* 坐标重建后，用首个采样角度对齐观测器。 */
        Mechanical_ESO.State.Theta = Theta;
        Pos_Valid = 1U;
    }

    Motor_Run.Theta_m = Theta;
    Encoder.Lost_Cnt = 0U;
    Encoder.Valid = 1U;

    if ((Encoder.Ready == 0U) && (Encoder.Fault == 0U))
    {
        Startup_Count();

        Ready_Cnt++;

        if (Ready_Cnt >= ENC_READY_VALID_CNT)
        {
            /* 编码器就绪时，用当前角度初始化观测器。 */
            Mechanical_ESO.State.Theta = Theta;
            Encoder.Ready = 1U;
        }
    }
}
