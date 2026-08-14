#include "Encoder.h"

#include "Math.h"
#include "Motor_Type.h"
#include "main.h"
#include "spi.h"


#define MT6816_REG_ANGLE_LSB    0x03U
#define MT6816_REG_ANGLE_MSB    0x04U
#define MT6816_CMD_READ         0x80U

#define ENC_CPR                 16384.0f
#define RAW_TO_RAD              (TWO_PI_F / ENC_CPR)


volatile Encoder_T Encoder = {0};


static uint16_t Tx;
static uint16_t Rx;
static uint16_t Data;
static float Theta_Pre = 0.0f;
static uint8_t Step = 0U;
static uint8_t Busy = 0U;
static uint8_t Pos_Valid = 0U;


static HAL_StatusTypeDef Reg_Read_DMA(uint8_t Reg)
{
    Tx = (uint16_t)((uint16_t)(MT6816_CMD_READ | Reg) << 8);
    Rx = 0U;

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);

    return HAL_SPI_TransmitReceive_DMA(&hspi1,
                                       (uint8_t *)&Tx,
                                       (uint8_t *)&Rx,
                                       1U);
}


void Encoder_Start(void)
{
    if (Busy != 0U)
    {
        return;
    }

    Busy = 1U;
    Step = 0U;
    Data = 0U;

    if (Reg_Read_DMA(MT6816_REG_ANGLE_LSB) != HAL_OK)
    {
        HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
        Busy = 0U;
    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    float Theta;
    float Delta;

    if (hspi->Instance != SPI1)
    {
        return;
    }

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

    if (Step == 0U)
    {
        Data = (uint16_t)((uint16_t)(uint8_t)Rx << 8);
        Step = 1U;

        if (Reg_Read_DMA(MT6816_REG_ANGLE_MSB) != HAL_OK)
        {
            HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
            Busy = 0U;
        }

        return;
    }

    Data |= (uint16_t)(uint8_t)Rx;

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
        }
        else if (Delta > PI_F)
        {
            Motor_Run.Turn--;
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

    Busy = 0U;
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
    Busy = 0U;
}
