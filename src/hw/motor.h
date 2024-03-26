/**
 * @file motor.h
 * @brief HW motor driver.
 *
 * Main task is to update speed of the motor.
 * Two timers used for each motor: PWM Timer for pulse generation and Ecnoder Timer for reading wheel position.
 * PWM pulse generated in timer interrupt context.
 * Encoder value read in polling context and speed calculated based on counter change speed.
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "ioif.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    IoPinType nsleep_pin;
    IoPinType en1_pin;
    IoPinType en2_pin;
    IoPinType fault_pin;
    IoPinType ipropi_pin;
} MotorPinoutType;

typedef struct
{
} MotorDataType;

typedef struct
{
    MotorPinoutType *pinout;         /* Pin configuration*/
    TIM_HandleTypeDef *enc_timer;    /* Timer, that counts encoder rotations */
    TIM_HandleTypeDef *pwm_timer;    /* Timer, that generates PWM signal */
    IoPinType pwm_pin;               /* PWM signal output pin, that will run motors */
    double linear_velocity;          /* Actual linear velocity */
    double linear_velocity_setpoint; /* Desired linear velocity */
    double duty_cycle;               /* PWM duty cycle. Note: PID input is double; Mark (+/-) defines direction */
    uint32_t prev_enc_timestamp;     /* Time of the encoder update in order to calculate pulse width (and speed) */

} MotorHandleType;

void motor_init(MotorHandleType *motor_handler, MotorPinoutType *pinout, TIM_HandleTypeDef *pwm_timer,
                TIM_HandleTypeDef *enc_timer);
void motor_update(MotorHandleType *motor_handler);
void motor_enable(MotorHandleType *motor_handler);
void motor_disable(MotorHandleType *motor_handler);
void motor_setDutyCycleLimit(MotorHandleType *motor_handler, uint8_t limit);

#endif