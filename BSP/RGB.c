#include "RGB.h"

#include "tim.h"


/* TIM2 runs at 160 MHz: one bit is 2 us, T0H is 0.281 us and T1H is 0.9 us. */
#define RGB_DATA_BITS      24U
#define RGB_RESET_CNT      128U  /* 256 us low level */
#define RGB_DATA_POS       RGB_RESET_CNT
#define RGB_BUF_LEN        (RGB_RESET_CNT + RGB_DATA_BITS + RGB_RESET_CNT)
#define RGB_T0H_CNT        45U
#define RGB_T1H_CNT        144U


static uint32_t RGB_Buf[RGB_BUF_LEN];


static void RGB_DMA_Stop(DMA_HandleTypeDef *Dma)
{
    TIM_HandleTypeDef *Tim = (TIM_HandleTypeDef *)Dma->Parent;

    (void)HAL_TIM_DMABurst_WriteStop(Tim, TIM_DMA_UPDATE);
    __HAL_TIM_SET_COMPARE(Tim, TIM_CHANNEL_1, 0U);
    Tim->Instance->EGR = TIM_EGR_UG;
    (void)HAL_TIM_PWM_Stop(Tim, TIM_CHANNEL_1);
}


void RGB_Set(uint8_t R, uint8_t G, uint8_t B)
{
    DMA_HandleTypeDef *Dma = htim2.hdma[TIM_DMA_ID_UPDATE];
    uint32_t GRB;
    uint32_t Mask;
    uint32_t i;

    if ((HAL_TIM_DMABurstState(&htim2) != HAL_DMA_BURST_STATE_READY) ||
        (Dma->State != HAL_DMA_STATE_READY))
    {
        return;
    }

    GRB = ((uint32_t)G << 16U) | ((uint32_t)R << 8U) | (uint32_t)B;
    Mask = 1UL << (RGB_DATA_BITS - 1U);

    for (i = 0U; i < RGB_DATA_BITS; i++)
    {
        RGB_Buf[RGB_DATA_POS + i] = ((GRB & Mask) != 0U) ? RGB_T1H_CNT : RGB_T0H_CNT;
        Mask >>= 1U;
    }

    /* Load the first period as active and the second into the CCR preload register. */
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, RGB_Buf[0]);
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, RGB_Buf[1]);

    if (HAL_TIM_DMABurst_MultiWriteStart(&htim2,
                                         TIM_DMABASE_CCR1,
                                         TIM_DMA_UPDATE,
                                         &RGB_Buf[2],
                                         TIM_DMABURSTLENGTH_1TRANSFER,
                                         RGB_BUF_LEN - 2U) != HAL_OK)
    {
        (void)HAL_TIM_DMABurst_WriteStop(&htim2, TIM_DMA_UPDATE);
        return;
    }

    Dma->XferCpltCallback = RGB_DMA_Stop;
    Dma->XferHalfCpltCallback = NULL;
    Dma->XferErrorCallback = RGB_DMA_Stop;
    __HAL_DMA_DISABLE_IT(Dma, DMA_IT_HT);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        (void)HAL_TIM_DMABurst_WriteStop(&htim2, TIM_DMA_UPDATE);
    }
}
