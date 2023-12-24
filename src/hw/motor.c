#include "motor.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ioif.h"
#include "motor_cfg.h"
#include "peripheral.h"
#include "stm32f4xx_hal.h"

void motor_init(MotorHandleType *ptr_motor, MotorCfgType *ptr_motor_config, TIM_HandleTypeDef *pwm_timer)
{
    ioif_init();

    ptr_motor->ptr_motor_config = ptr_motor_config;
    ptr_motor->effort = 0;
    ptr_motor->effort_limit = 300;
    ptr_motor->linear_velocity = 0;
    ptr_motor->linear_velocity_setpoint = 0;
    ptr_motor->pwm_port = ptr_motor_config->en1_port;
    ptr_motor->pwm_pin = ptr_motor_config->en1_pin;
    ptr_motor->effort_output_reg = pwm_timer->Instance->CCR1; // TODO remove direct register write
    ptr_motor->last_enc_update = 0;
    ptr_motor->htim = pwm_timer;

    // Start timers for motor PWM generation (gpios are SET in periodelapsedCallback and RESET in pulseFinishedCallback)
    HAL_TIM_Base_Start_IT(pwm_timer);
    HAL_TIM_PWM_Start_IT(pwm_timer, TIM_CHANNEL_1);

    // Disable chip
    motor_disable(ptr_motor);
    motor_update(ptr_motor);
}

void motor_update(MotorHandleType *ptr_motor)
{
    double effort_epsilon = 100; // this is a counter value from where the motor exceeds its internal friction, also
                                 // instabilities in PWM generation occured with lower values.

    if (ptr_motor->effort > effort_epsilon)
    {
        // Forward
        ptr_motor->pwm_port = ptr_motor->ptr_motor_config->en1_port;
        ptr_motor->pwm_pin = ptr_motor->ptr_motor_config->en1_pin;
        *(ptr_motor->effort_output_reg) = abs(ptr_motor->effort);
        HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->en2_port, ptr_motor->ptr_motor_config->en2_pin, RESET);
        // motor_enable(ptr_motor);
        HAL_TIM_PWM_Start_IT(ptr_motor->htim, TIM_CHANNEL_1);
    }
    else if (ptr_motor->effort < -effort_epsilon)
    {
        // Reverse
        ptr_motor->pwm_port = ptr_motor->ptr_motor_config->en2_port;
        ptr_motor->pwm_pin = ptr_motor->ptr_motor_config->en2_pin;
        *(ptr_motor->effort_output_reg) = abs(ptr_motor->effort);
        HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->en1_port, ptr_motor->ptr_motor_config->en1_pin, RESET);
        // motor_enable(ptr_motor);
        HAL_TIM_PWM_Start_IT(ptr_motor->htim, TIM_CHANNEL_1);
    }
    else
    {
        // effort is inbetween [-epsilon...epsilon]
        // HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, RESET);
        // //Disable driver
        *(ptr_motor->effort_output_reg) = effort_epsilon;
        HAL_TIM_PWM_Stop_IT(ptr_motor->htim, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(ptr_motor->pwm_port, ptr_motor->pwm_pin, RESET);

        // motor_disable(ptr_motor);
    }

    // Compute velocity

    // Calculate the relation between an encoder pulse and
    // linear speed on the wheel where it contacts the ground
    // CCW is positive when looking from the motor towards the wheel
    #if 0
    if (ptr_motor->last_enc_update)
    {
        float dt_sec = (HAL_GetTick() - ptr_motor->last_enc_update) / 1000.0f;
        float pulse_to_speed_ratio = 1.0f / ENCODER_CPR / MOTOR_GEAR_RATIO * 2.0f * M_PI / dt_sec * MOTOR_WHEEL_RADIUS;
        ptr_motor->linear_velocity = ptr_motor->ptr_sw_enc->counter * pulse_to_speed_ratio;
    }
    ptr_motor->last_enc_update = HAL_GetTick();
    ptr_motor->ptr_sw_enc->counter = 0; // reset counter
    #endif
}

void motor_enable(MotorHandleType *ptr_motor)
{
    HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, SET);
}

void motor_disable(MotorHandleType *ptr_motor)
{
    HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, RESET);
}

void motor_debug(MotorHandleType *ptr_motor)
{
    // printf("Vel: %ld\t", ptr_motor->linear_velocity);
    // printf("Effort: %ld\t", ptr_motor->effort);
}