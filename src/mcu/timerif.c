/**
 * @file timerif.c
 * @brief Timer interface wrapper over CubeMX generated HAL functions
 *
 * Only particular timers are in use, they are pre-defined in .ioc file:
 * Timers 2, 3 and 4 in encoder mode
 * Timers 11, 13, 14 in PWM IT mode
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "timerif.h"

#include <stdbool.h>

#include "peripheral.h"
#include "system_hal.h"

static TimerCallbackType period_elapsed_callback;
static TimerCallbackType pulse_finished_callback;

/**
 * @brief Initializes timer module
 */
void timerif_init()
{
    static bool is_initialized = false;
    if (!is_initialized)
    {
        period_elapsed_callback = NULL;
        pulse_finished_callback = NULL;

        MX_TIM1_Init();
        MX_TIM2_Init();
        MX_TIM3_Init();
        MX_TIM4_Init();
        MX_TIM11_Init();
        MX_TIM13_Init();
        MX_TIM14_Init();

        is_initialized = true;
    }
}

/**
 * @brief Starts pre-defined timers and timer interrups
 */
void timerif_initInterrupts(void)
{
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

/**
 * @brief Sets function, that is called inside `HAL_TIM_PeriodElapsedCallback` interrupt
 */
void timerif_setPeriodElapsedCallback(TimerCallbackType callback)
{
    period_elapsed_callback = callback;
}

/**
 * @brief Sets function, that is called inside `HAL_TIM_PWM_PulseFinishedCallback` interrupt
 */
void timerif_setPulseFinishedCallback(TimerCallbackType callback)
{
    pulse_finished_callback = callback;
}

/**
 * @brief Sets PWM duty cycle in range from 0 to 100
 * @note  Using formula: CCR (compare capture register raw value) = duty_cycle * timer_period / 100%
 */
void timerif_setDutyCycle(TIM_HandleTypeDef *timer_handler, uint8_t duty_cycle)
{
    uint32_t ccr_value = timer_handler->Init.Period * duty_cycle / 100;
    __HAL_TIM_SET_COMPARE(timer_handler, TIM_CHANNEL_1, ccr_value);
}

/**
 * @brief Enables PWM pulse generation
 */
void timerif_enablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
    HAL_TIM_PWM_Start_IT(timer_handler, TIM_CHANNEL_1);
}

/**
 * @brief Disables PWM pulse generation
 */
void timerif_disablePwmInterrupts(TIM_HandleTypeDef *timer_handler)
{
    HAL_TIM_PWM_Stop_IT(timer_handler, TIM_CHANNEL_1);
}

/**
 * @brief Returns TIMER ENC counter value
 */
int16_t timerif_getCounter(TIM_HandleTypeDef *timer_handler)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(timer_handler);
}

/**
 * @brief Resets TIMER ENC counter to 0
 */
void timerif_resetCounter(TIM_HandleTypeDef *timer_handler)
{
    __HAL_TIM_SET_COUNTER(timer_handler, 0u);
}

/*======================================================================================================================
 * Timer PWM ISR handlers. Naming comes from CUBE HAL
======================================================================================================================*/

/**
 * @brief Interrupt, that called, when PWM signal goes from LOW to HIGH
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (period_elapsed_callback != NULL)
    {
        period_elapsed_callback(htim);
    }
}

/**
 * @brief Interrupt, that called, when PWM signal goes from HIGH to LOW
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (pulse_finished_callback != NULL)
    {
        pulse_finished_callback(htim);
    }
}