/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Enc_Cyc, ADC_Cyc and Fast_Cyc use the TIM1 underflow ISR as DWT T0. */
typedef struct
{
  uint32_t T0;
  uint32_t Snap_Cnt;
  uint32_t TIM_ISR_Cyc;
  uint32_t TIM_ISR_Max;
  uint32_t SPI_RX_Cnt;
  /* SPI_1 is RX DMA HT; SPI_2 is RX DMA TC. */
  uint32_t SPI_1_Cyc;
  uint32_t SPI_1_ISR_Cyc;
  uint32_t SPI_1_Flag;
  uint32_t SPI_1_CNDTR;
  uint32_t SPI_2_Cyc;
  uint32_t SPI_2_ISR_Cyc;
  uint32_t SPI_2_Flag;
  uint32_t SPI_2_CNDTR;
  uint32_t Enc_Cyc;
  uint32_t ADC_Cyc;
  uint32_t ADC_Run_Cyc;
  uint32_t ADC_Run_Max;
  uint32_t ADC_ISR_Cyc;
  uint32_t ADC_ISR_Max;
  uint32_t Fast_Cyc;
  uint32_t Fast_Max;
  uint32_t Enc_Late;
  uint32_t Enc_Miss;
  uint32_t Deadline_Miss;

} Fast_Time_T;


extern volatile Fast_Time_T Fast_Time;
extern volatile Fast_Time_T Fast_Snap;
extern volatile uint32_t Fast_Snap_Ready;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R_EN_Pin GPIO_PIN_13
#define R_EN_GPIO_Port GPIOC
#define LCD_RES_Pin GPIO_PIN_14
#define LCD_RES_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_15
#define LCD_DC_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI1_CSN_Pin GPIO_PIN_2
#define SPI1_CSN_GPIO_Port GPIOD
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB
#define LCD_BLK_Pin GPIO_PIN_7
#define LCD_BLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
