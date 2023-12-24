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

    // gpios are SET in periodelapsedCallback and RESET in pulseFinishedCallback
    HAL_TIM_Base_Start_IT(TIMER_PWM_M0);
    HAL_TIM_Base_Start_IT(TIMER_PWM_M1);
    HAL_TIM_Base_Start_IT(TIMER_PWM_M2);
    HAL_TIM_PWM_Start_IT(TIMER_PWM_M0, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(TIMER_PWM_M1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(TIMER_PWM_M2, TIM_CHANNEL_1);

    // counter value get via polling (timerif_getCounter)
    HAL_TIM_Encoder_Start(TIMER_ENC_M0, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(TIMER_ENC_M1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(TIMER_ENC_M2, TIM_CHANNEL_ALL);
}

void timerif_setEffort(TIM_HandleTypeDef *timer_handler, int16_t effort)
{
    __HAL_TIM_SET_COMPARE(timer_handler, TIM_CHANNEL_1, effort);
}

void timerif_enablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
    HAL_TIM_PWM_Start_IT(timer_handler, TIM_CHANNEL_1);
}

void timerif_disablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
    HAL_TIM_PWM_Stop_IT(timer_handler, TIM_CHANNEL_1);
}

int16_t timerif_getCounter(TIM_HandleTypeDef *timer_handler)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(timer_handler);
}
