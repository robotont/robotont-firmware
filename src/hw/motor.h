/**
 * @file motor.h
 * @brief 
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
    double linear_velocity;
    double linear_velocity_setpoint;
    double effort;
    uint32_t last_enc_update;
} MotorDataType;

typedef struct
{
    MotorPinoutType *pinout;
    MotorDataType *data;

    IoPinType pwm_pin;

    volatile uint32_t *effort_output_reg; // TODO remove
    TIM_HandleTypeDef *pwm_timer;

} MotorHandleType;

void motor_init(MotorHandleType *ptr_motor, MotorPinoutType *ptr_pinout, TIM_HandleTypeDef *pwm_timer);
void motor_update(MotorHandleType *ptr_motor);
void motor_debug(MotorHandleType *ptr_motor);
void motor_enable(MotorHandleType *ptr_motor);
void motor_disable(MotorHandleType *ptr_motor);

#endif