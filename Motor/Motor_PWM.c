#include "Motor_PWM.h"

#include "tim.h"


#define PWM_CCER_MASK  (TIM_CCER_CC1E  | TIM_CCER_CC1NE | \
                        TIM_CCER_CC2E  | TIM_CCER_CC2NE | \
                        TIM_CCER_CC3E  | TIM_CCER_CC3NE)


void PWM_Update(float DutyA, float DutyB, float DutyC)
{
    float Arr;

    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        return;
    }

    Arr = (float)TIM1->ARR;

    TIM1->CCR1 = (uint32_t)(DutyA * Arr);
    TIM1->CCR2 = (uint32_t)(DutyB * Arr);
    TIM1->CCR3 = (uint32_t)(DutyC * Arr);
}


void PWM_Enable(void)
{
    uint32_t Half;

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE);

    Half = TIM1->ARR / 2U;
    TIM1->CCR1 = Half;
    TIM1->CCR2 = Half;
    TIM1->CCR3 = Half;

    SET_BIT(TIM1->CCER, PWM_CCER_MASK);

    /* AOE opens all six outputs at the UEV that activates the 50% CCR values. */
    SET_BIT(TIM1->BDTR, TIM_BDTR_AOE);

    while ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
    }

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE);
}


void PWM_Disable(void)
{
    uint32_t Half;

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE);
    CLEAR_BIT(TIM1->CCER, PWM_CCER_MASK);

    Half = TIM1->ARR / 2U;
    TIM1->CCR1 = Half;
    TIM1->CCR2 = Half;
    TIM1->CCR3 = Half;
}
