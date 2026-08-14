#include "Encoder.h"

#include "main.h"
#include "spi.h"


#define MT6816_REG_ANGLE_LSB    0x03U
#define MT6816_REG_ANGLE_MSB    0x04U
#define MT6816_CMD_READ         0x80U

#define ENC_CPR                 16384.0f
#define TWO_PI                  6.2831853071795864769f
#define RAW_TO_RAD              (TWO_PI / ENC_CPR)


volatile Encoder_T Encoder = {0};


static uint8_t Reg_Read(uint8_t Reg)
{
    uint16_t Tx;
    uint16_t Rx = 0U;

    Tx = (uint16_t)((uint16_t)(MT6816_CMD_READ | Reg) << 8);

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive(&hspi1,
                               (uint8_t *)&Tx,
                               (uint8_t *)&Rx,
                               1U,
                               10U) != HAL_OK)
    {
        HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
        Error_Handler();
    }

    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);

    return (uint8_t)Rx;
}


void Encoder_Read(void)
{
    uint16_t Data;

    Data = (uint16_t)((uint16_t)Reg_Read(MT6816_REG_ANGLE_LSB) << 8);
    Data |= (uint16_t)Reg_Read(MT6816_REG_ANGLE_MSB);

    Encoder.Raw = Data >> 2;
    Encoder.Theta_m = (float)Encoder.Raw * RAW_TO_RAD;
}
