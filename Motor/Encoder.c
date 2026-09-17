/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Encoder.h"

#include "MT6816.h"
#include "MT6835.h"
#include "Math.h"
#include "Motor_Cal.h"
#include "Motor_Type.h"
#include "control_params.h"
#include "motor_params.h"

#define ENC_READY_VALID_CNT      16U
#define ENC_RUNTIME_INVALID_CNT  3U
#define ENC_STARTUP_TIMEOUT_CNT  ((uint16_t)(CUR_FREQ_HZ_DEFAULT * 0.05f))

Encoder_Config_T Encoder_Config = ENCODER_CONFIG_DEFAULT;
volatile Encoder_T Encoder = { 0 };

static float Theta_Pre = 0.0f;
static float Delta_Sum = 0.0f;
static uint32_t Speed_Div_Cnt = 0U;
static uint8_t Pos_Valid = 0U;
static uint8_t Ready_Cnt = 0U;

static void (*Drv_Config)(void) = 0;
static void (*Drv_Start)(void) = 0;
static void (*Drv_IRQHandler)(void) = 0;

static bool SPI_Driver_Bind(Encoder_SPI_Type_e Type)
{
    switch (Type)
    {
        case ENC_SPI_MT6816:
            Drv_Config = MT6816_Config;
            Drv_Start = MT6816_Start;
            Drv_IRQHandler = MT6816_IRQHandler;
            return true;

        case ENC_SPI_MT6835:
            Drv_Config = MT6835_Config;
            Drv_Start = MT6835_Start;
            Drv_IRQHandler = MT6835_IRQHandler;
            return true;

        default:
            return false;
    }
}

static bool Driver_Bind(void)
{
    Drv_Config = 0;
    Drv_Start = 0;
    Drv_IRQHandler = 0;

    switch ((Encoder_Protocol_e)Encoder_Config.Protocol)
    {
        case ENC_PROTOCOL_SPI:
            return SPI_Driver_Bind((Encoder_SPI_Type_e)Encoder_Config.SPI_Type);

        default:
            return false;
    }
}

static void Feedback_Reset(void)
{
    Theta_Pre = 0.0f;
    Delta_Sum = 0.0f;
    Speed_Div_Cnt = 0U;
    Pos_Valid = 0U;
    Ready_Cnt = 0U;

    Encoder.Raw = 0U;
    Encoder.Theta_Native = 0.0f;
    Encoder.Theta_m = 0.0f;
    Encoder.Position = 0.0f;
    Encoder.Err_Cnt = 0U;
    Encoder.Miss_Cnt = 0U;
    Encoder.Startup_Cnt = 0U;
    Encoder.Lost_Cnt = 0U;
    Encoder.Valid = 0U;
    Encoder.Ready = 0U;
    Encoder.Fault = 0U;

    Motor_Run.Turn = 0;
    Motor_Run.Theta_m = 0.0f;
    Motor_Run.Wm = 0.0f;
}

static void Startup_Count(void)
{
    if ((Encoder.Ready != 0U) || (Encoder.Fault != 0U))
    {
        return;
    }

    if (Encoder.Startup_Cnt < ENC_STARTUP_TIMEOUT_CNT)
    {
        Encoder.Startup_Cnt++;
    }

    if (Encoder.Startup_Cnt >= ENC_STARTUP_TIMEOUT_CNT)
    {
        Encoder.Fault = 1U;
    }
}

static void Startup_Invalid(void)
{
    Encoder.Valid = 0U;

    if (Encoder.Ready == 0U)
    {
        Ready_Cnt = 0U;
        Startup_Count();
    }
}

void Encoder_DMA_Config(void)
{
    Feedback_Reset();

    if ((Encoder_Protocol_e)Encoder_Config.Protocol == ENC_PROTOCOL_NONE)
    {
        Drv_Config = 0;
        Drv_Start = 0;
        Drv_IRQHandler = 0;
        return;
    }

    if (!Driver_Bind())
    {
        Encoder.Fault = 1U;
        return;
    }

    Drv_Config();
}

void Encoder_Config_Changed(void)
{
    Motor_Cal_Invalidate();
    Encoder_DMA_Config();
}

float Encoder_Position_Get(void)
{
    return Encoder.Position;
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
    Startup_Invalid();

    if ((Encoder.Ready != 0U) && (Encoder.Fault == 0U))
    {
        if (Encoder.Lost_Cnt < ENC_RUNTIME_INVALID_CNT)
        {
            Encoder.Lost_Cnt++;
        }

        if (Encoder.Lost_Cnt >= ENC_RUNTIME_INVALID_CNT)
        {
            Encoder.Fault = 1U;
        }
    }
}

void Encoder_Sample_Reject(void)
{
    Startup_Invalid();
}

void Encoder_Sample_Update(uint32_t Raw, float Theta)
{
    float Delta;
    float Wm_Raw;

    Encoder.Raw = Raw;
    Encoder.Theta_Native = Theta;

    if (Motor_Cal.Enc_Dir < 0)
    {
        Theta = Angle_Wrap(-Theta);
    }

    if (Pos_Valid != 0U)
    {
        Delta = Theta - Theta_Pre;

        if (Delta < -PI_F)
        {
            Motor_Run.Turn++;
            Delta += TWO_PI_F;
        }
        else if (Delta > PI_F)
        {
            Motor_Run.Turn--;
            Delta -= TWO_PI_F;
        }

        Delta_Sum += Delta;

        if (++Speed_Div_Cnt >= (uint32_t)(CUR_FREQ_HZ_DEFAULT / SPD_FREQ_HZ_DEFAULT))
        {
            Speed_Div_Cnt = 0U;
            Wm_Raw = Delta_Sum / SPD_TS;
            Delta_Sum = 0.0f;
            Motor_Run.Wm += SPD_FBK_ALPHA_DEFAULT * (Wm_Raw - Motor_Run.Wm);
        }
    }
    else
    {
        Pos_Valid = 1U;
    }

    Theta_Pre = Theta;
    Encoder.Theta_m = Theta;
    Encoder.Position = (float)Motor_Run.Turn * TWO_PI_F + Theta;
    Encoder.Lost_Cnt = 0U;
    Encoder.Valid = 1U;
    Motor_Run.Theta_m = Theta;

    if ((Encoder.Ready == 0U) && (Encoder.Fault == 0U))
    {
        Startup_Count();

        if (Ready_Cnt < ENC_READY_VALID_CNT)
        {
            Ready_Cnt++;
        }

        if (Ready_Cnt >= ENC_READY_VALID_CNT)
        {
            Encoder.Ready = 1U;
        }
    }
}
