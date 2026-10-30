/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "MT6835.h"

#include "Encoder.h"
#include "Encoder_Port.h"
#include "Math.h"
#include "control_params.h"
#include "main.h"

#define MT6835_BURST_CMD_ADDR 0xA003U
#define MT6835_CPR            2097152.0f
#define MT6835_RAW_TO_RAD     (TWO_PI_F / MT6835_CPR)

#define MT6835_STATUS_UNDERVOLT 0x04U

#define ENC_UNDERVOLT_FAULT_MS  2U
#define ENC_UNDERVOLT_FAULT_CNT ((uint16_t)((CUR_FREQ_HZ_DEFAULT * (float)ENC_UNDERVOLT_FAULT_MS) / 1000.0f))

/* At 170 MHz: CSN setup 100 ns, SPI timeout 1 us. */
#define ENC_CSN_LOW_CYC 17U
#define ENC_SPI_END_CYC 170U

volatile MT6835_State_T MT6835_State = { 0 };

static const uint16_t Tx[3] = { MT6835_BURST_CMD_ADDR, 0U, 0U };
static volatile uint16_t Rx[3];
static uint8_t Busy = 0U;

static FAST_CODE void Transfer_Abort(void)
{
    uint32_t CR1;
    uint32_t CR2;

    CLEAR_BIT(ENC_DMA_TX->CCR, DMA_CCR_EN);
    CLEAR_BIT(ENC_DMA_RX->CCR, DMA_CCR_EN);
    ENC_DMA->IFCR = ENC_DMA_TX_CLEAR | ENC_DMA_RX_CLEAR;
    ENC_CS_PORT->BSRR = ENC_CS_PIN;

    /* Flush a partial frame and queued TX words before the next CSN edge. */
    CR1 = ENC_SPI->CR1;
    CR2 = ENC_SPI->CR2;
    SET_BIT(ENC_SPI_RST_REG, ENC_SPI_RST);
    CLEAR_BIT(ENC_SPI_RST_REG, ENC_SPI_RST);
    ENC_SPI->CR2 = CR2;
    ENC_SPI->CR1 = CR1;

    Busy = 0U;
    Encoder.Miss_Cnt++;
    Encoder_Sample_Invalid();
    Fast_Time.Enc_Miss++;
}

static FAST_CODE bool Frame_Finished(void)
{
    uint32_t Wait_T0;

    Wait_T0 = DWT->CYCCNT;
    while (((ENC_SPI->SR & SPI_SR_BSY) != 0U) && ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
    {
    }

    return ((ENC_SPI->SR & (SPI_SR_BSY | SPI_SR_TXE)) == SPI_SR_TXE);
}

void MT6835_Config(void)
{
    ENC_CS_PORT->BSRR = ENC_CS_PIN;
    Rx[0] = 0U;
    Rx[1] = 0U;
    Rx[2] = 0U;
    Busy = 0U;

    MT6835_State.CRC_Err = 0U;
    MT6835_State.Under_Voltage_Cnt = 0U;
    MT6835_State.Status = 0U;

    /* TX supplies the complete burst; only RX interrupts.
     * Keep halfword transfers so MT6816 can share the same SPI data format. */
    ENC_DMA_TX->CCR = DMA_CCR_DIR | DMA_CCR_MINC |
                        DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_0;
    ENC_DMA_TX->CPAR = (uint32_t)&ENC_SPI->DR;
    ENC_DMA_TX->CMAR = (uint32_t)Tx;
    ENC_DMA_TX->CNDTR = 0U;

    ENC_DMA_RX->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                        DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;
    ENC_DMA_RX->CPAR = (uint32_t)&ENC_SPI->DR;
    ENC_DMA_RX->CMAR = (uint32_t)Rx;
    ENC_DMA_RX->CNDTR = 0U;
    ENC_DMA->IFCR = ENC_DMA_TX_CLEAR | ENC_DMA_RX_CLEAR;

    SET_BIT(ENC_SPI->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    SET_BIT(ENC_SPI->CR1, SPI_CR1_SPE);
}

void MT6835_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) ||
        ((ENC_DMA->ISR & (ENC_DMA_TX_ERR | ENC_DMA_RX_ERR)) != 0U) ||
        ((ENC_SPI->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_FTLVL)) != SPI_SR_TXE))
    {
        Transfer_Abort();
        return;
    }

    CLEAR_BIT(ENC_DMA_TX->CCR, DMA_CCR_EN);
    CLEAR_BIT(ENC_DMA_RX->CCR, DMA_CCR_EN);
    ENC_DMA->IFCR = ENC_DMA_TX_CLEAR | ENC_DMA_RX_CLEAR;
    ENC_DMA_TX->CNDTR = 3U;
    ENC_DMA_RX->CNDTR = 3U;
    SET_BIT(ENC_DMA_RX->CCR, DMA_CCR_EN);
    Busy = 1U;

    ENC_CS_PORT->BSRR = (uint32_t)ENC_CS_PIN << 16U;
    Wait_T0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
    {
    }

    SET_BIT(ENC_DMA_TX->CCR, DMA_CCR_EN);
}

void MT6835_IRQHandler(void)
{
    float Theta;
    uint32_t Raw;
    uint16_t Data_H;
    uint16_t Data_L;
    uint8_t Status;
    uint8_t Crc;
    uint8_t Crc_Recv;
    uint32_t T0;
    uint32_t Flags;

    T0 = DWT->CYCCNT;
    Flags = ENC_DMA->ISR;

    if ((Flags & ENC_DMA_RX_ERR) != 0U)
    {
        ENC_DMA->IFCR = ENC_DMA_RX_CLEAR;
        Transfer_Abort();
        return;
    }

    if ((Flags & ENC_DMA_RX_TC) == 0U)
    {
        return;
    }

    ENC_DMA->IFCR = ENC_DMA_RX_TC_CLEAR;
    Fast_Time.SPI_RX_Cnt++;

    if ((Busy == 0U) || !Frame_Finished())
    {
        Transfer_Abort();
        return;
    }

    /* SPI_2 fields remain the final-frame timing; SPI_RX_Cnt is now one. */
    Fast_Time.SPI_2_Cyc = T0 - Fast_Time.T0;
    Fast_Time.SPI_2_Flag = Flags;
    Fast_Time.SPI_2_CNDTR = ENC_DMA_RX->CNDTR;

    ENC_CS_PORT->BSRR = ENC_CS_PIN;

    Data_H = Rx[1];
    Data_L = Rx[2];
    Raw = ((uint32_t)Data_H << 5) | (uint32_t)(Data_L >> 11);
    Status = (uint8_t)((Data_L >> 8) & 0x07U);
    Crc_Recv = (uint8_t)(Data_L & 0x00FFU);

    /* Registers 0x003..0x005, wire byte order; initial CRC and final XOR are zero. */
    Crc = CRC8_07(0U, (uint8_t)(Data_H >> 8));
    Crc = CRC8_07(Crc, (uint8_t)Data_H);
    Crc = CRC8_07(Crc, (uint8_t)(Data_L >> 8));

    if (Crc != Crc_Recv)
    {
        MT6835_State.CRC_Err++;
        Encoder.Err_Cnt++;
        Encoder_Sample_Invalid();
        Busy = 0U;
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    MT6835_State.Status = Status;

    if ((Status & MT6835_STATUS_UNDERVOLT) != 0U)
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

        Busy = 0U;
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    MT6835_State.Under_Voltage_Cnt = 0U;

    Theta = (float)Raw * MT6835_RAW_TO_RAD;
    Encoder_Sample_Update(Raw, Theta);
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;

    Busy = 0U;
    Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
}
