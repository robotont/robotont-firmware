/**
 * @file motor.c
 * @brief HW motor driver.
 *
 * Main task is to update speed of the motor.
 * Two timers used for each motor: PWM Timer for pulse generation and Ecnoder Timer for reading wheel position.
 * PWM pulse generated in timer interrupt context.
 * Duty cycle value range is (-100.0, 100.0). Negative sign means negative rotation direction.
 * Encoder value read in polling context and speed calculated based on counter change speed.
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

#define DUTY_CYCLE_EPSILON       10 /* If duty cycle value is less, then PWM pulse is not strong enough to run the motor */
#define DUTY_CYCLE_LIMIT_DEFAULT 25 /* If duty cycle value is greater, then motors can be dangerous */

static uint8_t duty_cycle_limit;

/**
 * @brief Initializes given motor
 */
void motor_init(MotorHandleType *motor_handler, MotorPinoutType *pinout, TIM_HandleTypeDef *pwm_timer,
                TIM_HandleTypeDef *enc_timer)
{
    motor_handler->pinout = pinout;
    motor_handler->pwm_pin = pinout->en1_pin;
    motor_handler->pwm_timer = pwm_timer;
    motor_handler->enc_timer = enc_timer;
    motor_handler->duty_cycle = 0.0f;
    motor_handler->linear_velocity = 0.0f;
    motor_handler->linear_velocity_setpoint = 0.0f;
    motor_handler->prev_enc_timestamp = 0u;

    duty_cycle_limit = DUTY_CYCLE_LIMIT_DEFAULT;
    motor_disable(motor_handler);
}

/**
 * @brief Updates motor speed based on `motor_handler.duty_cycle`. PWM duty cycle calculated in the upper layer.
 */
void motor_update(MotorHandleType *motor_handler)
{
    double dt_sec;
    double linear_velocity;
    double pulse_to_speed_ratio;
    uint16_t duty_cycle;
    int16_t enc_counter;
    uint32_t current_timestamp;

    /* Updates PWM pins based on duty cycle value. Positive direction is pin EN1, negative - EN2 */
    if (motor_handler->duty_cycle >= DUTY_CYCLE_EPSILON)
    {
        motor_enable(motor_handler);
        motor_handler->pwm_pin = motor_handler->pinout->en1_pin;
        ioif_writePin(&motor_handler->pinout->en2_pin, false);
    }
    else if (motor_handler->duty_cycle <= -DUTY_CYCLE_EPSILON)
    {
        motor_enable(motor_handler);
        motor_handler->pwm_pin = motor_handler->pinout->en2_pin;
        ioif_writePin(&motor_handler->pinout->en1_pin, false);
    }
    else
    {
        motor_disable(motor_handler);
        ioif_writePin(&motor_handler->pinout->en1_pin, false);
        ioif_writePin(&motor_handler->pinout->en2_pin, false);
    }
    duty_cycle = (uint8_t)abs(motor_handler->duty_cycle);
    if (duty_cycle > duty_cycle_limit)
    {
        duty_cycle = duty_cycle_limit;
    }
    timerif_setDutyCycle(motor_handler->pwm_timer, duty_cycle);

    /* Calculates wheel rotation speed based on counter pulse value */
    if (motor_handler->prev_enc_timestamp != 0)
    {
        enc_counter = timerif_getCounter(motor_handler->enc_timer);
        current_timestamp = system_hal_timestamp();

        dt_sec = (current_timestamp - motor_handler->prev_enc_timestamp) / 1000.0f;
        pulse_to_speed_ratio = 1.0f / MOTOR_ENC_CPR / MOTOR_GEAR_RATIO * 2.0f * M_PI / dt_sec * MOTOR_WHEEL_OUTER_R;
        linear_velocity = enc_counter * pulse_to_speed_ratio;

        motor_handler->linear_velocity = linear_velocity;
    }
    timerif_resetCounter(motor_handler->enc_timer);
    motor_handler->prev_enc_timestamp = system_hal_timestamp();
}

/**
 * @brief Enables PWM interrupt on given motor and wakes-up motor
 */
void motor_enable(MotorHandleType *motor_handler)
{
    timerif_enablePwmInterrupts(motor_handler->pwm_timer);
    ioif_writePin(&motor_handler->pinout->nsleep_pin, true);
}

/**
 * @brief Disables PWM interrupt on given motor and puts motor into sleep mode
 */
void motor_disable(MotorHandleType *motor_handler)
{
    timerif_disablePwmInterrupts(motor_handler->pwm_timer);
    ioif_writePin(&motor_handler->pinout->nsleep_pin, false);
}

/**
 * @brief Sets motor's duty cycle maximum allowed value (%)
 */
void motor_setDutyCycleLimit(MotorHandleType *motor_handler, uint8_t limit)
{
    duty_cycle_limit = limit;
}
