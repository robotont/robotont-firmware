/**
 * @file motor.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

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
#include "timerif.h"

#define EFFORT_EPSILON 100

/**
 * @brief
 *
 * @param motor_handler
 * @param pinout
 * @param pwm_timer
 */
void motor_init(MotorHandleType *motor_handler, MotorPinoutType *pinout, TIM_HandleTypeDef *pwm_timer)
{
    motor_handler->pinout = pinout;
    motor_handler->pwm_pin = pinout->en1_pin;
    motor_handler->pwm_timer = pwm_timer;
    motor_handler->effort = 0.0f;
    motor_handler->linear_velocity = 0.0f;
    motor_handler->linear_velocity_setpoint = 0.0f;
    motor_handler->last_enc_update = 0u;

    motor_disable(motor_handler);
}

/**
 * @brief
 *
 * @param ptr_motor
 */
void motor_update(MotorHandleType *motor_handler)
{
    uint16_t effort = (int16_t)abs(motor_handler->effort);

    if ((effort < EFFORT_EPSILON) && (effort > -EFFORT_EPSILON))
    {
        motor_disable(motor_handler);
    }
    else
    {
        motor_enable(motor_handler);
        timerif_setEffort(motor_handler->pwm_timer, (int16_t)motor_handler->effort);
    }
    
    /*
    int16_t effort;
    double effort_epsilon = 100; // this is a counter value from where the motor exceeds its internal friction, also
                                 // instabilities in PWM generation occured with lower values.

    if (motor_handler->effort >= effort_epsilon)
    {
        // Forward
        motor_handler->pwm_pin = motor_handler->pinout->en1_pin;
        // effort = (int16_t)abs(motor_handler->effort);
        // timerif_setEffort(motor_handler->pwm_timer, effort);
        *(motor_handler->effort_output_reg) = (int16_t)abs(motor_handler->effort);
        ioif_writePin(&motor_handler->pinout->en2_pin, false);
        // motor_enable(motor_handler);
        HAL_TIM_PWM_Start_IT(motor_handler->pwm_timer, TIM_CHANNEL_1);
    }
    else if (motor_handler->effort <= -effort_epsilon)
    {
        // Reverse
        motor_handler->pwm_pin = motor_handler->pinout->en2_pin;
        // effort = (int16_t)abs(motor_handler->effort);
        // timerif_setEffort(motor_handler->pwm_timer, effort);
        *(motor_handler->effort_output_reg) = (int16_t)abs(motor_handler->effort);
        ioif_writePin(&motor_handler->pinout->en1_pin, false);
        // motor_enable(motor_handler);
        HAL_TIM_PWM_Start_IT(motor_handler->pwm_timer, TIM_CHANNEL_1);
    }
    else
    {
        // effort is inbetween [-epsilon...epsilon]
        // Disable driver
        // effort = (int16_t)abs(motor_handler->effort);
        // timerif_setEffort(motor_handler->pwm_timer, effort);
        *(motor_handler->effort_output_reg) = (int16_t)effort_epsilon;
        HAL_TIM_PWM_Stop_IT(motor_handler->pwm_timer, TIM_CHANNEL_1);
        ioif_writePin(&motor_handler->pwm_pin, false);
    }
    */

// Compute velocity

// Calculate the relation between an encoder pulse and
// linear speed on the wheel where it contacts the ground
// CCW is positive when looking from the motor towards the wheel
#if 0 // TODO continue this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (motor_handler->last_enc_update)
    {
        float dt_sec = (HAL_GetTick() - motor_handler->last_enc_update) / 1000.0f;
        float pulse_to_speed_ratio = 1.0f / MOTOR_ENC_CPR / MOTOR_GEAR_RATIO * 2.0f * M_PI / dt_sec * MOTOR_WHEEL_OUTER_R;
        motor_handler->linear_velocity = motor_handler->ptr_sw_enc->counter * pulse_to_speed_ratio;
    }
    motor_handler->last_enc_update = HAL_GetTick();
    motor_handler->ptr_sw_enc->counter = 0; // reset counter
#endif
}

/**
 * @brief
 *
 * @param ptr_motor
 */
void motor_enable(MotorHandleType *ptr_motor)
{
    timerif_enablePwmInterrupts(ptr_motor->pwm_timer);
    ioif_writePin(&ptr_motor->pinout->nsleep_pin, true);
}

/**
 * @brief
 *
 * @param ptr_motor
 */
void motor_disable(MotorHandleType *ptr_motor)
{
    timerif_disablePwmInterrupts(ptr_motor->pwm_timer);
    ioif_writePin(&ptr_motor->pinout->nsleep_pin, false);
}

/**
 * @brief
 *
 * @param ptr_motor
 */
void motor_debug(MotorHandleType *ptr_motor)
{
    // printf("Vel: %ld\t", ptr_motor->linear_velocity);
    // printf("Effort: %ld\t", ptr_motor->effort);
}