#ifndef FAST_PROFILE_H
#define FAST_PROFILE_H

#include <stdint.h>

#define FAST_PROFILE_SAMPLE_NUM 2048U

typedef struct
{
    uint32_t Cnt;
    uint32_t Sum;
    uint32_t Min;
    uint32_t Max;

} Fast_Profile_Stat_T;

typedef struct
{
    volatile uint8_t Request;
    volatile uint8_t Run;
    volatile uint8_t Ready;
    uint32_t Sample_Cnt;

    Fast_Profile_Stat_T ADC_Sample;
    Fast_Profile_Stat_T Motor_Fast;
    Fast_Profile_Stat_T Flux_Observer;
    Fast_Profile_Stat_T PLL;
    Fast_Profile_Stat_T IF_Start;
    Fast_Profile_Stat_T Current_Loop;
    Fast_Profile_Stat_T SVPWM;
    Fast_Profile_Stat_T PWM_Update;
    Fast_Profile_Stat_T Plot_Fast;
    Fast_Profile_Stat_T ADC_Run;

} Fast_Profile_T;

extern volatile Fast_Profile_T Fast_Profile;

void Fast_Profile_Request(void);
void Fast_Profile_Begin_Cycle(void);
void Fast_Profile_End_Cycle(void);
void Fast_Profile_Add(volatile Fast_Profile_Stat_T *Stat, uint32_t Cyc);

#endif /* FAST_PROFILE_H */
