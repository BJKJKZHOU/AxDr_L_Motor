/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "MT6816.h"

#include "Encoder.h"
#include "Fast_Profile.h"
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

#define ENC_CSN_LOW_CYC  16U
#define ENC_CSN_HIGH_CYC 32U
#define ENC_SPI_END_CYC  160U

volatile MT6816_State_T MT6816_State = { 0 };

static volatile uint16_t Rx[2];
static uint8_t Step = 0U;
static uint8_t Busy = 0U;

static uint8_t Parity_Check(uint16_t Data)
{
    Data ^= Data >> 8;
    Data ^= Data >> 4;
    Data ^= Data >> 2;
    Data ^= Data >> 1;

    return (uint8_t)((~Data) & 1U);
}

static void DMA_Rearm(void)
{
    volatile uint32_t Dummy;

    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN);
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    DMA1->IFCR = DMA_IFCR_CGIF5;

    if ((SPI1->SR & (SPI_SR_RXNE | SPI_SR_OVR)) != 0U)
    {
        Dummy = SPI1->DR;
        Dummy = SPI1->SR;
        (void)Dummy;
    }

    DMA1_Channel5->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel5->CMAR = (uint32_t)&Rx[0];
    DMA1_Channel5->CNDTR = 2U;
    SET_BIT(DMA1_Channel5->CCR, DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE);
    SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    SET_BIT(SPI1->CR2, SPI_CR2_RXDMAEN);
}

static void Transfer_Abort(void)
{
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    Step = 0U;
    Busy = 0U;
    Encoder.Miss_Cnt++;
    Encoder_Sample_Invalid();
    Fast_Time.Enc_Miss++;
    DMA_Rearm();
}

void MT6816_Config(void)
{
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    Rx[0] = 0U;
    Rx[1] = 0U;
    Step = 0U;
    Busy = 0U;

    MT6816_State.Parity_Err = 0U;
    MT6816_State.No_Mag_Cnt = 0U;
    MT6816_State.No_Mag = 0U;

    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    DMA_Rearm();
}

void MT6816_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) || (DMA1_Channel5->CNDTR != 2U))
    {
        Transfer_Abort();
    }

    if ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
        Encoder.Miss_Cnt++;
        Encoder_Sample_Invalid();
        Fast_Time.Enc_Miss++;
        return;
    }

    Busy = 1U;
    Step = 0U;

    SPI1_CSN_GPIO_Port->BSRR = (uint32_t)SPI1_CSN_Pin << 16U;
    Wait_T0 = DWT->CYCCNT;

    while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
    {
    }

    *(__IO uint16_t *)&SPI1->DR = (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_LSB) << 8);
}

void MT6816_IRQHandler(void)
{
    float Theta;
    uint16_t Data;
    uint32_t T0;
    uint32_t Wait_T0;
    uint32_t Flags;

    T0 = DWT->CYCCNT;
    Flags = DMA1->ISR;

    if ((Flags & DMA_ISR_TEIF5) != 0U)
    {
        DMA1->IFCR = DMA_IFCR_CGIF5;
        Transfer_Abort();
        return;
    }

    if ((Flags & DMA_ISR_HTIF5) != 0U)
    {
        DMA1->IFCR = DMA_IFCR_CHTIF5;
        Fast_Time.SPI_RX_Cnt++;
        Fast_Time.SPI_1_Cyc = T0 - Fast_Time.T0;
        Fast_Time.SPI_1_Flag = Flags;
        Fast_Time.SPI_1_CNDTR = DMA1_Channel5->CNDTR;

        if ((Busy == 0U) || (Step != 0U))
        {
            Transfer_Abort();
            Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
            return;
        }

        Wait_T0 = DWT->CYCCNT;
        while (((SPI1->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
        {
        }

        if ((SPI1->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE)
        {
            Transfer_Abort();
            Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
            return;
        }

        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        Wait_T0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_HIGH_CYC)
        {
        }

        Step = 1U;
        SPI1_CSN_GPIO_Port->BSRR = (uint32_t)SPI1_CSN_Pin << 16U;
        Wait_T0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
        {
        }

        *(__IO uint16_t *)&SPI1->DR = (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_MSB) << 8);
        Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    if ((Flags & DMA_ISR_TCIF5) == 0U)
    {
        return;
    }

    DMA1->IFCR = DMA_IFCR_CTCIF5;
    Fast_Time.SPI_RX_Cnt++;
    Fast_Time.SPI_2_Cyc = T0 - Fast_Time.T0;
    Fast_Time.SPI_2_Flag = Flags;
    Fast_Time.SPI_2_CNDTR = DMA1_Channel5->CNDTR;

    if ((Busy == 0U) || (Step != 1U))
    {
        Transfer_Abort();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    Wait_T0 = DWT->CYCCNT;
    while (((SPI1->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
    {
    }

    if ((SPI1->SR & SPI_SR_BSY) != 0U)
    {
        Transfer_Abort();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
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

    Encoder.Raw = Data >> 2;
    Theta = (float)Encoder.Raw * RAW_TO_RAD;
    Encoder_Sample_Update(Encoder.Raw, Theta);
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;

    Step = 0U;
    Busy = 0U;
    Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
}
