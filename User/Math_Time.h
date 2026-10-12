/* Optional on-target math profiling; no control signals are changed. */
#ifndef MATH_TIME_H
#define MATH_TIME_H

#include "stm32g4xx.h"

enum
{
    MATH_WRAP_E,
    MATH_WRAP_ESO,
    MATH_SINCOS,
    MATH_LIM_I,
    MATH_LIM_U,
    MATH_PID_D,
    MATH_PID_Q,
    MATH_ESO,
    MATH_SVPWM,
    MATH_EMPTY,
    MATH_COUNT
};

typedef struct
{
    uint32_t Cyc;
    uint32_t Min;
    uint32_t Max;
    uint32_t Count;
    uint32_t Sum_Lo;
    uint32_t Sum_Hi;
    uint32_t Alt;       /* fmod path, vector changed, or Kd != 0. */
    uint32_t Alt_Count;
} Math_Time_T;

extern volatile Math_Time_T Math_Time[MATH_COUNT];
/* SWD stops accumulation before clearing or reading the counters. */
extern volatile uint32_t Math_Run;

/* Volatile DWT reads alone do not prevent ordinary loads/stores moving across
 * the timing boundary. These compiler barriers emit no MCU instructions. */
__STATIC_FORCEINLINE uint32_t Math_Clock(void)
{
    uint32_t Cyc;
    __COMPILER_BARRIER();
    Cyc = DWT->CYCCNT;
    __COMPILER_BARRIER();
    return Cyc;
}

#endif
