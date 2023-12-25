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
    double linear_velocity;          /* Actual linear velocity */
    double linear_velocity_setpoint; /* Desired linear velocity */
    double effort;                   /* PWM effort */
    uint32_t last_enc_update;        /* Time of the encoder update in order to calculate pulse width (and speed) */
} MotorDataType;

typedef struct
{
    MotorPinoutType *pinout;              /* Pin configuration*/
    MotorDataType *data;                  /* Runtime parameters*/
    volatile uint32_t *effort_output_reg; // TODO remove
    TIM_HandleTypeDef *pwm_timer;         /* Timer, that generates PWM signal */
    IoPinType pwm_pin;                    /* PWM signal output pin, that will run motors */

    // TODO encoder timer

} MotorHandleType;

void motor_init(MotorHandleType *ptr_motor, MotorPinoutType *ptr_pinout, TIM_HandleTypeDef *pwm_timer);
void motor_update(MotorHandleType *ptr_motor);
void motor_debug(MotorHandleType *ptr_motor);
void motor_enable(MotorHandleType *ptr_motor);
void motor_disable(MotorHandleType *ptr_motor);

#endif