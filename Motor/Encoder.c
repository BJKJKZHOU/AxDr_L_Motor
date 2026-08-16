#include "Encoder.h"

#include "Math.h"
#include "Motor_Type.h"
#include "control_params.h"
#include "main.h"


#define MT6816_REG_ANGLE_LSB    0x03U
#define MT6816_REG_ANGLE_MSB    0x04U
#define MT6816_CMD_READ         0x80U

#define ENC_CPR                 16384.0f
#define RAW_TO_RAD              (TWO_PI_F / ENC_CPR)

/* MT6816 requires at least 100 ns from CSN low to the first SCK edge. */
#define ENC_CSN_LOW_CYC         16U
/* 200 ns CSN high time and 1 us SPI end timeout at 160 MHz. */
#define ENC_CSN_HIGH_CYC        32U
#define ENC_SPI_END_CYC         160U


volatile Encoder_T Encoder = {0};


static volatile uint16_t Rx[2];
static float Theta_Pre = 0.0f;
static float Delta_Sum = 0.0f;
static uint32_t Speed_Div_Cnt = 0U;
static uint8_t Step = 0U;
static uint8_t Busy = 0U;
static uint8_t Pos_Valid = 0U;


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


void Encoder_DMA_Config(void)
{
    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
    Rx[0] = 0U;
    Rx[1] = 0U;
    Step = 0U;
    Busy = 0U;
    Pos_Valid = 0U;
    Delta_Sum = 0.0f;
    Speed_Div_Cnt = 0U;
    Motor_Run.Wm = 0.0f;

    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    DMA_Rearm();
}


void Encoder_Start(void)
{
    uint32_t Wait_T0;

    if ((Busy != 0U) || (DMA1_Channel5->CNDTR != 2U))
    {
        Fast_Time.Enc_Miss++;
        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        Step = 0U;
        Busy = 0U;
        DMA_Rearm();
    }

    if ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
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

    *(__IO uint16_t *)&SPI1->DR =
        (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_LSB) << 8);
}


void Encoder_DMA_IRQHandler(void)
{
    float Theta;
    float Delta;
    float Wm_Raw;
    uint16_t Data;
    uint32_t T0;
    uint32_t Wait_T0;
    uint32_t Flags;

    T0 = DWT->CYCCNT;
    Flags = DMA1->ISR;

    if ((Flags & DMA_ISR_TEIF5) != 0U)
    {
        DMA1->IFCR = DMA_IFCR_CGIF5;
        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        Step = 0U;
        Busy = 0U;
        Fast_Time.Enc_Miss++;
        DMA_Rearm();
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
            SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
            Step = 0U;
            Busy = 0U;
            Fast_Time.Enc_Miss++;
            DMA_Rearm();
            Fast_Time.SPI_1_ISR_Cyc = DWT->CYCCNT - T0;
            return;
        }

        Wait_T0 = DWT->CYCCNT;

        while (((SPI1->SR & SPI_SR_BSY) != 0U) &&
               ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
        {
        }

        if ((SPI1->SR & (SPI_SR_BSY | SPI_SR_TXE)) != SPI_SR_TXE)
        {
            SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
            Step = 0U;
            Busy = 0U;
            Fast_Time.Enc_Miss++;
            DMA_Rearm();
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

        *(__IO uint16_t *)&SPI1->DR =
            (uint16_t)((uint16_t)(MT6816_CMD_READ | MT6816_REG_ANGLE_MSB) << 8);

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
        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        Step = 0U;
        Busy = 0U;
        Fast_Time.Enc_Miss++;
        DMA_Rearm();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    Wait_T0 = DWT->CYCCNT;

    while (((SPI1->SR & SPI_SR_BSY) != 0U) &&
           ((DWT->CYCCNT - Wait_T0) < ENC_SPI_END_CYC))
    {
    }

    if ((SPI1->SR & SPI_SR_BSY) != 0U)
    {
        SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;
        Step = 0U;
        Busy = 0U;
        Fast_Time.Enc_Miss++;
        DMA_Rearm();
        Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
        return;
    }

    SPI1_CSN_GPIO_Port->BSRR = SPI1_CSN_Pin;

    Data = (uint16_t)(((Rx[0] & 0x00FFU) << 8) | (Rx[1] & 0x00FFU));

    Encoder.Raw = Data >> 2;
    Theta = (float)Encoder.Raw * RAW_TO_RAD;

    if (Motor_Cal.Enc_Dir < 0)
    {
        Theta = Angle_Wrap(-Theta);
    }

    if (Pos_Valid != 0U)
    {
        Delta = Theta - Theta_Pre;

        if (Delta < -PI_F)
        {
            Motor_Run.Turn++;
            Delta += TWO_PI_F;
        }
        else if (Delta > PI_F)
        {
            Motor_Run.Turn--;
            Delta -= TWO_PI_F;
        }

        Delta_Sum += Delta;

        if (++Speed_Div_Cnt >=
            (uint32_t)(CUR_FREQ_HZ_DEFAULT / SPD_FREQ_HZ_DEFAULT))
        {
            Speed_Div_Cnt = 0U;
            Wm_Raw = Delta_Sum / SPD_TS;
            Delta_Sum = 0.0f;

            Motor_Run.Wm += SPD_FBK_ALPHA_DEFAULT
                          * (Wm_Raw - Motor_Run.Wm);
        }
    }
    else
    {
        Pos_Valid = 1U;
    }

    Theta_Pre = Theta;

    Encoder.Theta_m = Theta;
    Motor_Run.Theta_m = Theta;
    Motor_Run.Theta_e = Angle_Wrap((float)Motor_Para.Pp * Theta + Motor_Cal.Theta_Off);
    Fast_Time.Enc_Cyc = DWT->CYCCNT - Fast_Time.T0;

    Step = 0U;
    Busy = 0U;
    Fast_Time.SPI_2_ISR_Cyc = DWT->CYCCNT - T0;
}
