#include "timerif.h"

#include "peripheral.h"
#include "system_hal.h"

static TimerCallbackType period_elapsed_callback;
static TimerCallbackType pulse_finished_callback;

void timerif_init()
{
    period_elapsed_callback = NULL;
    pulse_finished_callback = NULL;
    
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

void timerif_setPeriodElapsedCallback(TimerCallbackType callback)
{
    period_elapsed_callback = callback;
}

void timerif_setPulseFinishedCallback(TimerCallbackType callback)
{
    pulse_finished_callback = callback;
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

/* Auto-generated code with Cube MX */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Set PWM pin to high depending on which pwm timer triggered the interrupt
    if (htim->Instance == htim11.Instance)
    {
        HAL_GPIO_WritePin(PIN_M0_EN1_GPIO_Port, PIN_M0_EN1_Pin, SET);
        // ioif_writePin(&hm0.pwm_pin, true);
    }
    else if (htim->Instance == htim13.Instance)
    {
        HAL_GPIO_WritePin(PIN_M1_EN1_GPIO_Port, PIN_M1_EN1_Pin, SET);
        // ioif_writePin(&hm1.pwm_pin, true);
    }
    else if (htim->Instance == htim14.Instance)
    {
        HAL_GPIO_WritePin(PIN_M2_EN1_GPIO_Port, PIN_M2_EN1_Pin, SET);
        // ioif_writePin(&hm2.pwm_pin, true);
    }

    if (period_elapsed_callback != NULL)
    {
        period_elapsed_callback(htim);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim11.Instance) // motor 0
    {
        HAL_GPIO_WritePin(PIN_M0_EN1_GPIO_Port, PIN_M0_EN1_Pin, RESET);
        // ioif_writePin(&hm0.pwm_pin, false);
    }
    else if (htim->Instance == htim13.Instance) // motor 1
    {
        HAL_GPIO_WritePin(PIN_M1_EN1_GPIO_Port, PIN_M1_EN1_Pin, RESET);
        // ioif_writePin(&hm1.pwm_pin, false);
    }
    else if (htim->Instance == htim14.Instance) // motor 2
    {
        HAL_GPIO_WritePin(PIN_M2_EN1_GPIO_Port, PIN_M2_EN1_Pin, RESET);
        // ioif_writePin(&hm2.pwm_pin, false);
    }

    if (pulse_finished_callback != NULL)
    {
        pulse_finished_callback(htim);
    }
}