#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>


typedef struct
{
    uint16_t Raw;
    float Theta_m;
} Encoder_T;


extern volatile Encoder_T Encoder;


void Encoder_DMA_Config(void);
void Encoder_DMA_IRQHandler(void);
void Encoder_Start(void);


#endif
