/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Encoder.h"

#include "MT6816.h"
#include "MT6835.h"
#include "Math.h"
#include "Sin_LUT.h"
#include "Mechanical_ESO.h"
#include "Motor_Type.h"
#include "control_params.h"
#include "main.h"
#include "motor_params.h"

#define ENC_READY_VALID_CNT      16U
#define ENC_RUNTIME_INVALID_CNT  3U
#define ENC_STARTUP_TIMEOUT_CNT  ((uint16_t)(CUR_FREQ_HZ_DEFAULT * 0.05f))

Encoder_Config_T Encoder_Config = ENCODER_CONFIG_DEFAULT;
volatile Encoder_T Encoder = { 0 };
Encoder_Harmonic_T Encoder_Harmonic = { 0 };

FAST_CODE float Encoder_Angle_Correct(float Theta_Native)
{
    float Sin;
    float Cos;
    float Error;

    if ((Encoder_Harmonic.C1 == 0.0f) && (Encoder_Harmonic.S1 == 0.0f) &&
        (Encoder_Harmonic.C2 == 0.0f) && (Encoder_Harmonic.S2 == 0.0f))
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

static void (*Drv_Config)(void) = 0;
static void (*Drv_Start)(void) = 0;
static void (*Drv_IRQHandler)(void) = 0;

static void None_Config(void)
{
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN);
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel5->CCR,
              DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA1->IFCR = DMA_IFCR_CGIF5;
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
}

static void None_Start(void)
{
}

static void None_IRQHandler(void)
{
    DMA1->IFCR = DMA_IFCR_CGIF5;
}

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
    switch ((Encoder_Protocol_e)Encoder_Config.Protocol)
    {
        case ENC_PROTOCOL_NONE:
            Drv_IRQHandler = None_IRQHandler;
            Drv_Start = None_Start;
            Drv_Config = None_Config;
            return true;

        case ENC_PROTOCOL_SPI:
            return SPI_Driver_Bind((Encoder_SPI_Type_e)Encoder_Config.SPI_Type);

        default:
            Drv_Config = 0;
            Drv_Start = 0;
            Drv_IRQHandler = 0;
            return false;
    }
}

static void Feedback_Reset(void)
{
    Pos_Valid = 0U;
    Ready_Cnt = 0U;

    Encoder.Raw = 0U;
    Encoder.Theta_Native = 0.0f;
    Encoder.Theta_m = 0.0f;
    Encoder.Err_Cnt = 0U;
    Encoder.Miss_Cnt = 0U;
    Encoder.Startup_Cnt = 0U;
    Encoder.Lost_Cnt = 0U;
    Encoder.Valid = 0U;
    Encoder.Ready = 0U;
    Encoder.Fault = 0U;

    Motor_Run.Turn = 0;
    Mechanical_ESO.State = (Mechanical_ESO_State_T){ 0 };
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

void Encoder_DMA_Config(void)
{
    uint32_t Primask;
    uint32_t CR1;
    uint32_t CR2;

    /* Rebinding must not race a PWM trigger or an old DMA completion.
     * MT6835 owns TX channel 3 as well as the shared RX channel 5. */
    Primask = __get_PRIMASK();
    __disable_irq();
    CR1 = SPI1->CR1;
    CR2 = SPI1->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    SPI1->CR2 = CR2;
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel5->CCR,
              DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF5;
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    /* Discard any queued SPI words when switching to another driver or None. */
    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    SPI1->CR2 = CR2;
    SPI1->CR1 = CR1;

    Feedback_Reset();

    if (!Driver_Bind())
    {
        Encoder.Fault = 1U;
        __set_PRIMASK(Primask);
        return;
    }

    Drv_Config();
    __set_PRIMASK(Primask);
}

void Encoder_Position_Read(int32_t *Turn, float *Theta)
{
    uint32_t Primask;

    /* Turn and Theta_m are updated together by the encoder DMA ISR. */
    Primask = __get_PRIMASK();
    __disable_irq();
    *Turn = Motor_Run.Turn;
    *Theta = Encoder.Theta_m;
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
    Theta = Encoder_Angle_Correct(Theta);

    if (Motor_Cal.Enc_Dir < 0)
    {
        Theta = Angle_Wrap(-Theta);
    }

    if (Pos_Valid != 0U)
    {
        /* 更新前的单圈角度就是上一拍角度，用于判断是否跨圈。 */
        Delta = Theta - Encoder.Theta_m;

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
        Pos_Valid = 1U;
    }

    Encoder.Theta_m = Theta;
    Encoder.Lost_Cnt = 0U;
    Encoder.Valid = 1U;

    if ((Encoder.Ready == 0U) && (Encoder.Fault == 0U))
    {
        Startup_Count();

        if (Ready_Cnt < ENC_READY_VALID_CNT)
        {
            Ready_Cnt++;
        }

        if (Ready_Cnt >= ENC_READY_VALID_CNT)
        {
            /* Establish the observer angle when the feedback first becomes
             * usable, not each time the power stage is enabled. */
            Mechanical_ESO.State.Theta = Theta;
            Encoder.Ready = 1U;
        }
    }
}
