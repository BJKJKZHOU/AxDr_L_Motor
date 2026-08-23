#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct
{
    uint16_t Raw;
    float Theta_m;

    uint32_t Parity_Err;
    uint16_t No_Mag_Cnt;
    uint8_t No_Mag;
    uint8_t Fault;
} Encoder_T;

extern volatile Encoder_T Encoder;

void Encoder_DMA_Config(void);
void Encoder_DMA_IRQHandler(void);
void Encoder_Start(void);

#endif
