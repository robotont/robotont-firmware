#include "timerif.h"

#include "peripheral.h"

void timerif_init()
{
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
}

void timerif_setEffort(TIM_HandleTypeDef *timer_handler)
{
}

void timerif_enablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
}

void timerif_disablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
}

void timerif_getCounter(TIM_HandleTypeDef *timer_handler)
{
}
