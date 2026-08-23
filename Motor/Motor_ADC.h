#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#include <stdint.h>

typedef struct
{
    uint16_t Ia_Raw;
    uint16_t Ib_Raw;
    uint16_t Ic_Raw;

    uint16_t Ia_Off;
    uint16_t Ib_Off;
    uint16_t Ic_Off;

    float Ia_A;
    float Ib_A;
    float Ic_A;

    uint16_t Vbus_Raw;
    float Vbus_V;

} ADC_T;

extern volatile ADC_T ADC;

void ADC_Calib(void);
void ADC_Sample(void);
void ADC_Run(void);

#endif /* MOTOR_ADC_H */
