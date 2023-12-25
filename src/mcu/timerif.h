
#ifndef TIMERIF
#define TIMERIF

#include "peripheral.h"
#include "system_hal.h"

typedef void (*TimerCallbackType)(TIM_HandleTypeDef *timer_handler); /* Callback, that called, when PWM changes state */

#define TIMER_PWM_M0    &htim11
#define TIMER_PWM_M1    &htim13
#define TIMER_PWM_M2    &htim14
#define TIMER_ENC_M0    &htim2
#define TIMER_ENC_M1    &htim3
#define TIMER_ENC_M2    &htim4

void timerif_init();

void timerif_setPeriodElapsedCallback(TimerCallbackType callback);
void timerif_setPulseFinishedCallback(TimerCallbackType callback);

void timerif_setEffort(TIM_HandleTypeDef *timer_handler, int16_t effort);
void timerif_enablePwmInterrupts(TIM_HandleTypeDef *timer_handler);
void timerif_disablePwmInterrupts(TIM_HandleTypeDef *timer_handler);

int16_t timerif_getCounter(TIM_HandleTypeDef *timer_handler);

#endif