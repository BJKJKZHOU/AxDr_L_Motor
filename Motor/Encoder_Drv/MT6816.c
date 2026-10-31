/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "MT6816.h"

#include "Encoder.h"
#include "Encoder_Port.h"
#include "Math.h"
#include "control_params.h"
#include "main.h"

#define MT6816_REG_ANGLE_LSB 0x03U
#define MT6816_REG_ANGLE_MSB 0x04U
#define MT6816_CMD_READ      0x80U
#define MT6816_NO_MAG_MASK   0x0002U

#define ENC_CPR    16384.0f
#define RAW_TO_RAD (TWO_PI_F / ENC_CPR)

#define ENC_NOMAG_FAULT_MS  2U
#define ENC_NOMAG_FAULT_CNT ((uint16_t)((CUR_FREQ_HZ_DEFAULT * (float)ENC_NOMAG_FAULT_MS) / 1000.0f))

/* 170 MHz 下：片选建立时间 100 ns，帧间高电平 200 ns，SPI 超时 1 us。 */
#define ENC_CSN_LOW_CYC  17U
#define ENC_CSN_HIGH_CYC 34U
#define ENC_SPI_END_CYC  170U

volatile MT6816_State_T MT6816_State = { 0 };

static volatile uint16_t Rx[2];
static uint8_t Step = 0U;
static uint8_t Busy = 0U;

static FAST_CODE uint8_t Parity_Check(uint16_t Data)
{
    Data ^= Data >> 8;
    Data ^= Data >> 4;
    Data ^= Data >> 2;
    Data ^= Data >> 1;

    return (uint8_t)((~Data) & 1U);
}

static FAST_CODE void DMA_Rearm(void)
{
    volatile uint32_t Dummy;

    CLEAR_BIT(ENC_SPI->CR2, SPI_CR2_RXDMAEN);
    CLEAR_BIT(ENC_DMA_RX->CCR, DMA_CCR_EN);
    ENC_DMA->IFCR = ENC_DMA_RX_CLEAR;

    if ((ENC_SPI->SR & (SPI_SR_RXNE | SPI_SR_OVR)) != 0U)
    {
        Dummy = ENC_SPI->DR;
        Dummy = ENC_SPI->SR;
        (void)Dummy;
    }

    ENC_DMA_RX->CPAR = (uint32_t)&ENC_SPI->DR;
    ENC_DMA_RX->CMAR = (uint32_t)&Rx[0];
    ENC_DMA_RX->CNDTR = 2U;
    /* 从 MT6835 切回时，恢复循环接收两个 16 位数据的方式。 */
    SET_BIT(ENC_DMA_RX->CCR,
            DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE);
    SET_BIT(ENC_DMA_RX->CCR, DMA_CCR_EN);
    SET_BIT(ENC_SPI->CR2, SPI_CR2_RXDMAEN);
}

static FAST_CODE void Transfer_Abort(void)
{
    ENC_CS_PORT->BSRR = ENC_CS_PIN;
    Step = 0U;
    Busy = 0U;
    Encoder.Miss_Cnt++;
    Encoder_Sample_Invalid();
    Fast_Time.Enc_Miss++;
    DMA_Rearm();
}

void MT6816_Config(void)
{
    ENC_CS_PORT->BSRR = ENC_CS_PIN;
    Rx[0] = 0U;
    Rx[1] = 0U;
    Step = 0U;
    Busy = 0U;

    MT6816_State.Parity_Err = 0U;
    MT6816_State.No_Mag_Cnt = 0U;
    MT6816_State.No_Mag = 0U;

    CLEAR_BIT(ENC_SPI->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(ENC_SPI->CR1, SPI_CR1_SPE);
    DMA_Rearm();
}

void MT6816_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) || (ENC_DMA_RX->CNDTR != 2U))
    {
        Transfer_Abort();
    }

    if ((ENC_SPI->SR & SPI_SR_TXE) == 0U)
    {
        Encoder.Miss_Cnt++;
        Encoder_Sample_Invalid();
        Fast_Time.Enc_Miss++;
        return;
    }

    Busy = 1U;
    Step = 0U;

    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16U;
    Wait_T0 = DWT->CYCCNT;

    while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
    {
    }

    *(__IO uint16_t *)&ENC_SPI->DR = (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_LSB) << 8);
}

void MT6816_IRQHandler(void)
{
    float Theta;
    uint16_t Data;
    uint32_t Raw;
    uint32_t T0;
    uint32_t Wait_T0;
    uint32_t Flags;

    T0 = DWT->CYCCNT;
    Flags = ENC_DMA->ISR;

    if ((Flags & ENC_DMA_RX_ERR) != 0U)
    {
        ENC_DMA->IFCR = ENC_DMA_RX_CLEAR;
        Transfer_Abort();
        return;
    }

    if ((Flags & ENC_DMA_RX_HT) != 0U)
    {
        ENC_DMA->IFCR = ENC_DMA_RX_HT_CLEAR;
        Fast_Time.SPI_RX_Cnt++;
        Fast_Time.SPI_1_Cyc = T0 - Fast_Time.T0;
        Fast_Time.SPI_1_Flag = Flags;
        Fast_Time.SPI_1_CNDTR = ENC_DMA_RX->CNDTR;

        if ((Busy == 0U) || (Step != 0U))
        {
            Transfer_Abort();
            Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
            return;
        }

        Wait_T0 = DWT->CYCCNT;
        while (((ENC_SPI->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
        {
        }

        if ((ENC_SPI->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE)
        {
            Transfer_Abort();
            Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
            return;
        }

        ENC_CS_PORT->BSRR = ENC_CS_PIN;
        Wait_T0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_HIGH_CYC)
        {
        }

        Step = 1U;
        ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16U;
        Wait_T0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
        {
        }

        *(__IO uint16_t *)&ENC_SPI->DR = (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_MSB) << 8);
        Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    if ((Flags & ENC_DMA_RX_TC) == 0U)
    {
        return;
    }

    ENC_DMA->IFCR = ENC_DMA_RX_TC_CLEAR;
    Fast_Time.SPI_RX_Cnt++;
    Fast_Time.SPI_2_Cyc = T0 - Fast_Time.T0;
    Fast_Time.SPI_2_Flag = Flags;
    Fast_Time.SPI_2_CNDTR = ENC_DMA_RX->CNDTR;

    if ((Busy == 0U) || (Step != 1U))
    {
        Transfer_Abort();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    Wait_T0 = DWT->CYCCNT;
    while (((ENC_SPI->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
    {
    }

    if ((ENC_SPI->SR & SPI_SR_BSY) != 0U)
    {
        Transfer_Abort();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    ENC_CS_PORT->BSRR = ENC_CS_PIN;
    Data = (uint16_t)(((Rx[0] & 0x00FFU) << 8) | (Rx[1] & 0x00FFU));

    if (Parity_Check(Data) == 0U)
    {
        MT6816_State.Parity_Err++;
        Encoder.Err_Cnt++;
        Encoder_Sample_Invalid();
        Step = 0U;
        Busy = 0U;
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    if ((Data & MT6816_NO_MAG_MASK) != 0U)
    {
        MT6816_State.No_Mag = 1U;
        Encoder_Sample_Reject();

        if (MT6816_State.No_Mag_Cnt < ENC_NOMAG_FAULT_CNT)
        {
            MT6816_State.No_Mag_Cnt++;
        }

        if (MT6816_State.No_Mag_Cnt >= ENC_NOMAG_FAULT_CNT)
        {
            Encoder.Fault = 1U;
        }

        Step = 0U;
        Busy = 0U;
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    MT6816_State.No_Mag = 0U;
    MT6816_State.No_Mag_Cnt = 0U;

    Raw = Data >> 2;
    Theta = (float)Raw * RAW_TO_RAD;
    Encoder_Sample_Update(Raw, Theta);
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;

    Step = 0U;
    Busy = 0U;
    Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
}
