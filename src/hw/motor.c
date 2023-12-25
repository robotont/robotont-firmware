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

void motor_init(MotorHandleType *motor_handler, MotorPinoutType *pinout, TIM_HandleTypeDef *pwm_timer)
{

    motor_handler->pinout = pinout;
    motor_handler->pwm_pin = pinout->en1_pin;

    motor_handler->data->effort = 0;
    motor_handler->data->linear_velocity = 0;
    motor_handler->data->linear_velocity_setpoint = 0;
    motor_handler->data->last_enc_update = 0;

    motor_handler->pwm_timer = pwm_timer;
    motor_handler->effort_output_reg = pwm_timer->Instance->CCR1; // TODO remove direct register write

    motor_disable(motor_handler);
    motor_update(motor_handler);

    ioif_writePin(&motor_handler->pinout->en2_pin, false);
    motor_enable(motor_handler);
}

void motor_update(MotorHandleType *ptr_motor)
{
    double effort_epsilon = 100; // this is a counter value from where the motor exceeds its internal friction, also
                                 // instabilities in PWM generation occured with lower values.

    if (ptr_motor->data->effort > effort_epsilon)
    {
        // Forward
        ptr_motor->pwm_pin = ptr_motor->pinout->en1_pin;
        *(ptr_motor->effort_output_reg) = abs(ptr_motor->data->effort);
        ioif_writePin(&ptr_motor->pinout->en2_pin, false);
        // motor_enable(ptr_motor);
        HAL_TIM_PWM_Start_IT(ptr_motor->pwm_timer, TIM_CHANNEL_1);
    }
    else if (ptr_motor->data->effort < -effort_epsilon)
    {
        // Reverse
        ptr_motor->pwm_pin = ptr_motor->pinout->en2_pin;
        *(ptr_motor->effort_output_reg) = abs(ptr_motor->data->effort);
        ioif_writePin(&ptr_motor->pinout->en1_pin, false);
        // motor_enable(ptr_motor);
        HAL_TIM_PWM_Start_IT(ptr_motor->pwm_timer, TIM_CHANNEL_1);
    }
    else
    {
        // effort is inbetween [-epsilon...epsilon]
        //Disable driver
        *(ptr_motor->effort_output_reg) = effort_epsilon;
        HAL_TIM_PWM_Stop_IT(ptr_motor->pwm_timer, TIM_CHANNEL_1);
        ioif_writePin(&ptr_motor->pwm_pin, false);
        // motor_disable(ptr_motor);
    }

    // Compute velocity

    // Calculate the relation between an encoder pulse and
    // linear speed on the wheel where it contacts the ground
    // CCW is positive when looking from the motor towards the wheel
    #if 0 // TODO continue this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (ptr_motor->last_enc_update)
    {
        float dt_sec = (HAL_GetTick() - ptr_motor->last_enc_update) / 1000.0f;
        float pulse_to_speed_ratio = 1.0f / MOTOR_ENC_CPR / MOTOR_GEAR_RATIO * 2.0f * M_PI / dt_sec * MOTOR_WHEEL_OUTER_R;
        ptr_motor->linear_velocity = ptr_motor->ptr_sw_enc->counter * pulse_to_speed_ratio;
    }
    ptr_motor->last_enc_update = HAL_GetTick();
    ptr_motor->ptr_sw_enc->counter = 0; // reset counter
    #endif
}

void motor_enable(MotorHandleType *ptr_motor)
{
    ioif_writePin(&ptr_motor->pinout->nsleep_pin, true);
}

void motor_disable(MotorHandleType *ptr_motor)
{
    ioif_writePin(&ptr_motor->pinout->nsleep_pin, false);
}

void motor_debug(MotorHandleType *ptr_motor)
{
    // printf("Vel: %ld\t", ptr_motor->linear_velocity);
    // printf("Effort: %ld\t", ptr_motor->effort);
}