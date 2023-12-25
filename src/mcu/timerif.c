/**
 * @file timerif.c
 * @brief Timer interface wrapper over CubeMX generated MX_USB HAL
 *
 * Only particula timers are in use, they are pre-defined in .ioc file:
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
void timerif_initInterrups(void)
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
 * @brief Sets PWM effort value in the timer compare register
 * @note Use only PWM timers
 */
void timerif_setEffort(TIM_HandleTypeDef *timer_handler, int16_t effort)
{
    __HAL_TIM_SET_COMPARE(timer_handler, TIM_CHANNEL_1, effort);
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