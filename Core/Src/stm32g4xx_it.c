/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Encoder.h"
#include "Motor_ADC.h"
#include "control_params.h"
#include "motor_thread.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* At 20 kHz, discard 100 ms startup data and capture after 100 ms steady run. */
#define FAST_SNAP_WARMUP_CYC    2000U
#define FAST_SNAP_CAPTURE_CYC   4000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint32_t Motor_Div_Cnt = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_tim2_up;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
  Encoder_DMA_IRQHandler();
  return;

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_up);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  uint32_t T0;
  uint32_t Fast_T0;
  uint32_t Cyc;

  if (((ADC1->ISR & ADC_ISR_JEOS) == 0U) ||
      ((ADC1->IER & ADC_IER_JEOSIE) == 0U))
  {
    /* Keep the generated HAL path for unexpected shared ADC interrupts. */
    goto ADC_HAL_IRQ;
  }

  T0 = DWT->CYCCNT;
  Fast_T0 = Fast_Time.T0;

  /* JEOC is also set by the final rank; clear both before running the loop. */
  ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;

  if (Fast_T0 != UINT32_MAX)
  {
    Fast_Time.ADC_Cyc = T0 - Fast_T0;

    if (Fast_Time.Enc_Cyc == UINT32_MAX)
    {
      Fast_Time.Enc_Late++;
    }
  }

  Fast_Loop();

  Cyc = DWT->CYCCNT - T0;
  Fast_Time.ADC_ISR_Cyc = Cyc;

  if (Cyc > Fast_Time.ADC_ISR_Max)
  {
    Fast_Time.ADC_ISR_Max = Cyc;
  }

  if (Fast_T0 != UINT32_MAX)
  {
    Cyc = DWT->CYCCNT - Fast_T0;
    Fast_Time.Fast_Cyc = Cyc;

    if (Cyc > Fast_Time.Fast_Max)
    {
      Fast_Time.Fast_Max = Cyc;
    }

    if (Fast_Time.T0 != Fast_T0)
    {
      Fast_Time.Deadline_Miss++;
    }

    if (Fast_Snap_Ready == 0U)
    {
      Fast_Time.Snap_Cnt++;

      if (Fast_Time.Snap_Cnt == FAST_SNAP_WARMUP_CYC)
      {
        Fast_Time.TIM_ISR_Max = 0U;
        Fast_Time.ADC_Run_Max = 0U;
        Fast_Time.ADC_ISR_Max = 0U;
        Fast_Time.Fast_Max = 0U;
        Fast_Time.Enc_Late = 0U;
        Fast_Time.Enc_Miss = 0U;
        Fast_Time.Deadline_Miss = 0U;
      }
      else if ((Fast_Time.Snap_Cnt >= FAST_SNAP_CAPTURE_CYC) &&
               (Fast_Time.Enc_Cyc != UINT32_MAX))
      {
        /* Timing is closed before this one-time diagnostic copy. */
        Fast_Snap = Fast_Time;
        __DMB();
        Fast_Snap_Ready = 1U;
      }
    }
  }

  return;

ADC_HAL_IRQ:

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
  uint32_t T0;
  uint32_t Measure;

  T0 = DWT->CYCCNT;
  Measure = 0U;

  if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) &&
      ((TIM1->CR1 & TIM_CR1_DIR) == 0U))
  {
    Measure = 1U;
    Fast_Time.T0 = T0;
    /* ADC IRQ uses this sentinel to detect late encoder feedback. */
    Fast_Time.Enc_Cyc = UINT32_MAX;
    Fast_Time.SPI_RX_Cnt = 0U;
    Fast_Time.SPI_1_Cyc = UINT32_MAX;
    Fast_Time.SPI_1_ISR_Cyc = UINT32_MAX;
    Fast_Time.SPI_1_Flag = 0U;
    Fast_Time.SPI_1_CNDTR = UINT32_MAX;
    Fast_Time.SPI_2_Cyc = UINT32_MAX;
    Fast_Time.SPI_2_ISR_Cyc = UINT32_MAX;
    Fast_Time.SPI_2_Flag = 0U;
    Fast_Time.SPI_2_CNDTR = UINT32_MAX;

    if (++Motor_Div_Cnt >=
        (uint32_t)(CUR_FREQ_HZ_DEFAULT / SPD_FREQ_HZ_DEFAULT))
    {
      Motor_Div_Cnt = 0U;

      if (Motor_Ready != 0U)
      {
        (void)tx_semaphore_put(&Motor_Sem);
      }
    }

  }
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  if (Measure != 0U)
  {
    T0 = DWT->CYCCNT - T0;
    Fast_Time.TIM_ISR_Cyc = T0;

    if (T0 > Fast_Time.TIM_ISR_Max)
    {
      Fast_Time.TIM_ISR_Max = T0;
    }
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  if (((TIM1->SR & TIM_SR_CC4IF) != 0U) &&
      ((TIM1->DIER & TIM_DIER_CC4IE) != 0U))
  {
    TIM1->SR = ~TIM_SR_CC4IF;

    if ((TIM1->CR1 & TIM_CR1_DIR) == 0U)
    {
      Encoder_Start();
    }

    return;
  }
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
