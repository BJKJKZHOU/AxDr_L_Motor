/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "MT6835.h"

#include "Encoder.h"
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

static const uint16_t Tx[3] = { MT6835_BURST_CMD_ADDR, 0U, 0U };
static volatile uint16_t Rx[3];
static uint8_t Busy = 0U;

static FAST_CODE void Transfer_Abort(void)
{
    uint32_t CR1;
    uint32_t CR2;

    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF5;
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;

    /* Flush a partial frame and queued TX words before the next CSN edge. */
    CR1 = SPI1->CR1;
    CR2 = SPI1->CR2;
    SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    SPI1->CR2 = CR2;
    SPI1->CR1 = CR1;

    Busy = 0U;
    Encoder.Miss_Cnt++;
    Encoder_Sample_Invalid();
    Fast_Time.Enc_Miss++;
}

static FAST_CODE bool Frame_Finished(void)
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
    Busy = 0U;

    MT6835_State.CRC_Err = 0U;
    MT6835_State.Under_Voltage_Cnt = 0U;
    MT6835_State.Status = 0U;
    MT6835_State.Over_Speed = 0U;
    MT6835_State.Weak_Field = 0U;
    MT6835_State.Under_Voltage = 0U;

    /* Channel 3 supplies the complete burst; only RX channel 5 interrupts.
     * Keep halfword transfers so MT6816 can share the same SPI data format. */
    DMA1_Channel3->CCR = DMA_CCR_DIR | DMA_CCR_MINC |
                        DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_0;
    DMAMUX1_Channel2->CCR = DMA_REQUEST_SPI1_TX;
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t)Tx;
    DMA1_Channel3->CNDTR = 0U;

    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                        DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel5->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel5->CMAR = (uint32_t)Rx;
    DMA1_Channel5->CNDTR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF5;

    SET_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

void MT6835_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) ||
        ((DMA1->ISR & (DMA_ISR_TEIF3 | DMA_ISR_TEIF5)) != 0U) ||
        ((SPI1->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_FTLVL)) != SPI_SR_TXE))
    {
        Transfer_Abort();
        return;
    }

    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF5;
    DMA1_Channel3->CNDTR = 3U;
    DMA1_Channel5->CNDTR = 3U;
    SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    Busy = 1U;

    SPI1_CSN_GPIO_Port->BSRR = (uint32_t)SPI1_CSN_Pin << 16U;
    Wait_T0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - Wait_T0) < ENC_CSN_LOW_CYC)
    {
    }

    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
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

    /* SPI_2 fields remain the final-frame timing; SPI_RX_Cnt is now one. */
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
