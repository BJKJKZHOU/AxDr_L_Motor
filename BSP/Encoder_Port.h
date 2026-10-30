/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ENCODER_PORT_H
#define ENCODER_PORT_H

#include "stm32g4xx_hal.h"
#include "main.h"

/* SPI1 RX uses DMA1 CH5; TX uses CH3 without a separate IRQ. */
#define ENC_SPI             SPI1
#define ENC_SPI_RST_REG     RCC->APB2RSTR
#define ENC_SPI_RST         RCC_APB2RSTR_SPI1RST
#define ENC_CS_PORT         SPI1_CSN_GPIO_Port
#define ENC_CS_PIN          SPI1_CSN_Pin
#define ENC_DMA             DMA1
#define ENC_DMA_TX          DMA1_Channel3
#define ENC_DMA_RX          DMA1_Channel5
#define ENC_DMA_TX_CLEAR    DMA_IFCR_CGIF3
#define ENC_DMA_RX_CLEAR    DMA_IFCR_CGIF5
#define ENC_DMA_RX_HT_CLEAR DMA_IFCR_CHTIF5
#define ENC_DMA_RX_TC_CLEAR DMA_IFCR_CTCIF5
#define ENC_DMA_TX_ERR      DMA_ISR_TEIF3
#define ENC_DMA_RX_ERR      DMA_ISR_TEIF5
#define ENC_DMA_RX_HT       DMA_ISR_HTIF5
#define ENC_DMA_RX_TC       DMA_ISR_TCIF5

/* Board mapping stays visible for direct counter reads in the sample ISR. */
#define ENCODER_ABZ_TIM TIM3
#define ENCODER_ABZ_CPR_MAX 65536U

/* Called with interrupts masked while rebinding the encoder driver. */
void Encoder_Port_Release(void);
void Encoder_Port_SPI(void);
void Encoder_Port_ABZ(uint32_t Cpr);

#endif /* ENCODER_PORT_H */
