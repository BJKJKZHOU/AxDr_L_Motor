/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "MT6835.h"

#include "Encoder.h"
#include "Fast_Profile.h"
#include "Math.h"
#include "control_params.h"
#include "main.h"

#define MT6835_BURST_CMD_ADDR 0xA003U
#define MT6835_CPR            2097152.0f
#define MT6835_RAW_TO_RAD     (TWO_PI_F / MT6835_CPR)

#define MT6835_STATUS_OVERSPEED 0x01U
#define MT6835_STATUS_WEAKFIELD 0x02U
#define MT6835_STATUS_UNDERVOLT 0x04U

#define ENC_UNDERVOLT_FAULT_MS  2U
#define ENC_UNDERVOLT_FAULT_CNT ((uint16_t)((CUR_FREQ_HZ_DEFAULT * (float)ENC_UNDERVOLT_FAULT_MS) / 1000.0f))

#define ENC_CSN_LOW_CYC 16U
#define ENC_SPI_END_CYC 160U

volatile MT6835_State_T MT6835_State = { 0 };

static volatile uint16_t Rx[3];
static uint8_t Step = 0U;
static uint8_t Busy = 0U;

static uint8_t CRC8_Calc(uint32_t Raw, uint8_t Status)
{
    uint8_t Crc;
    uint8_t Data;
    uint8_t Bit;

    Crc = 0U;

    Data = (uint8_t)(Raw >> 13);
    Crc ^= Data;
    for (Bit = 0U; Bit < 8U; Bit++)
    {
        Crc = (Crc & 0x80U) ? (uint8_t)((Crc << 1) ^ 0x07U) : (uint8_t)(Crc << 1);
    }

    Data = (uint8_t)(Raw >> 5);
    Crc ^= Data;
    for (Bit = 0U; Bit < 8U; Bit++)
    {
        Crc = (Crc & 0x80U) ? (uint8_t)((Crc << 1) ^ 0x07U) : (uint8_t)(Crc << 1);
    }

    Data = (uint8_t)((Raw << 3) | (uint32_t)(Status & 0x07U));
    Crc ^= Data;
    for (Bit = 0U; Bit < 8U; Bit++)
    {
        Crc = (Crc & 0x80U) ? (uint8_t)((Crc << 1) ^ 0x07U) : (uint8_t)(Crc << 1);
    }

    return Crc;
}

static void DMA_Rearm(volatile uint16_t *Dst)
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
    DMA1_Channel5->CMAR = (uint32_t)Dst;
    DMA1_Channel5->CNDTR = 1U;
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_HTIE);
    SET_BIT(DMA1_Channel5->CCR, DMA_CCR_TCIE | DMA_CCR_TEIE);
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
    DMA_Rearm(&Rx[0]);
}

static bool Frame_Finished(void)
{
    uint32_t Wait_T0;

    Wait_T0 = DWT->CYCCNT;
    while (((SPI1->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
    {
    }

    return ((SPI1->SR & (SPI_SR_BSY | SPI_SR_TXE)) == SPI_SR_TXE);
}

void MT6835_Config(void)
{
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    Rx[0] = 0U;
    Rx[1] = 0U;
    Rx[2] = 0U;
    Step = 0U;
    Busy = 0U;

    MT6835_State.CRC_Err = 0U;
    MT6835_State.Under_Voltage_Cnt = 0U;
    MT6835_State.Status = 0U;
    MT6835_State.Over_Speed = 0U;
    MT6835_State.Weak_Field = 0U;
    MT6835_State.Under_Voltage = 0U;

    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    DMA_Rearm(&Rx[0]);
}

void MT6835_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) || (DMA1_Channel5->CNDTR != 1U))
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

    *(__IO uint16_t *)&SPI1->DR = MT6835_BURST_CMD_ADDR;
}

void MT6835_IRQHandler(void)
{
    float Theta;
    uint32_t Raw;
    uint16_t Data_H;
    uint16_t Data_L;
    uint8_t Status;
    uint8_t Crc_Recv;
    uint32_t T0;
    uint32_t Flags;

    T0 = DWT->CYCCNT;
    Flags = DMA1->ISR;

    if ((Flags & DMA_ISR_TEIF5) != 0U)
    {
        DMA1->IFCR = DMA_IFCR_CGIF5;
        Transfer_Abort();
        return;
    }

    if ((Flags & DMA_ISR_TCIF5) == 0U)
    {
        return;
    }

    DMA1->IFCR = DMA_IFCR_CTCIF5;
    Fast_Time.SPI_RX_Cnt++;

    if ((Busy == 0U) || !Frame_Finished())
    {
        Transfer_Abort();
        return;
    }

    if (Step == 0U)
    {
        Step = 1U;
        DMA_Rearm(&Rx[1]);
        *(__IO uint16_t *)&SPI1->DR = 0U;
        return;
    }

    if (Step == 1U)
    {
        Fast_Time.SPI_1_Cyc = T0 - Fast_Time.T0;
        Fast_Time.SPI_1_Flag = Flags;
        Fast_Time.SPI_1_CNDTR = DMA1_Channel5->CNDTR;

        Step = 2U;
        DMA_Rearm(&Rx[2]);
        *(__IO uint16_t *)&SPI1->DR = 0U;
        Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    if (Step != 2U)
    {
        Transfer_Abort();
        return;
    }

    Fast_Time.SPI_2_Cyc = T0 - Fast_Time.T0;
    Fast_Time.SPI_2_Flag = Flags;
    Fast_Time.SPI_2_CNDTR = DMA1_Channel5->CNDTR;

    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;

    Data_H = Rx[1];
    Data_L = Rx[2];
    Raw = ((uint32_t)(Data_H >> 8) << 13) |
          ((uint32_t)(Data_H & 0x00FFU) << 5) |
          ((uint32_t)(Data_L >> 11) & 0x1FU);
    Status = (uint8_t)((Data_L >> 8) & 0x07U);
    Crc_Recv = (uint8_t)(Data_L & 0x00FFU);

    if (CRC8_Calc(Raw, Status) != Crc_Recv)
    {
        MT6835_State.CRC_Err++;
        Encoder.Err_Cnt++;
        Encoder_Sample_Invalid();
        Step = 0U;
        Busy = 0U;
        DMA_Rearm(&Rx[0]);
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    MT6835_State.Status = Status;
    MT6835_State.Over_Speed = (Status & MT6835_STATUS_OVERSPEED) ? 1U : 0U;
    MT6835_State.Weak_Field = (Status & MT6835_STATUS_WEAKFIELD) ? 1U : 0U;
    MT6835_State.Under_Voltage = (Status & MT6835_STATUS_UNDERVOLT) ? 1U : 0U;

    if (MT6835_State.Under_Voltage != 0U)
    {
        Encoder_Sample_Reject();

        if (MT6835_State.Under_Voltage_Cnt < ENC_UNDERVOLT_FAULT_CNT)
        {
            MT6835_State.Under_Voltage_Cnt++;
        }

        if (MT6835_State.Under_Voltage_Cnt >= ENC_UNDERVOLT_FAULT_CNT)
        {
            Encoder.Fault = 1U;
        }

        Step = 0U;
        Busy = 0U;
        DMA_Rearm(&Rx[0]);
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    MT6835_State.Under_Voltage_Cnt = 0U;

    Theta = (float)Raw * MT6835_RAW_TO_RAD;
    Encoder_Sample_Update(Raw, Theta);
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;

    Step = 0U;
    Busy = 0U;
    DMA_Rearm(&Rx[0]);
    Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
}
