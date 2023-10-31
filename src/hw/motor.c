#include "motor.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "stm32f4xx_hal.h"
#include "peripheral.h"

#define GEAR_RATIO   18.75f // gearbox reduction ratio
#define WHEEL_RADIUS 0.035f // wheel outer radius

void motor_setConfig(MotorCfgType *ptr_motor1_config, MotorCfgType *ptr_motor2_config, MotorCfgType *ptr_motor3_config)
{
    // TODO refactor
    ptr_motor1_config->nsleep_port = PIN_M0_NSLEEP_GPIO_Port;
    ptr_motor1_config->en1_port = PIN_M0_EN1_GPIO_Port;
    ptr_motor1_config->en2_port = PIN_M0_EN2_GPIO_Port;
    ptr_motor1_config->fault_port = PIN_M0_FAULT_GPIO_Port;
    ptr_motor1_config->ipropi_port = PIN_M0_IPROPI_GPIO_Port;
    ptr_motor1_config->nsleep_pin = PIN_M0_NSLEEP_Pin;
    ptr_motor1_config->en1_pin = PIN_M0_EN1_Pin;
    ptr_motor1_config->en2_pin = PIN_M0_EN2_Pin;
    ptr_motor1_config->fault_pin = PIN_M0_FAULT_Pin;
    ptr_motor1_config->ipropi_pin = PIN_M0_IPROPI_Pin;
    ptr_motor1_config->enc_cpr = 64;
    ptr_motor1_config->wheel_pos_r = 0.145;
    ptr_motor1_config->wheel_pos_phi = M_PI / 3.0f;

    ptr_motor2_config->nsleep_port = PIN_M1_NSLEEP_GPIO_Port;
    ptr_motor2_config->en1_port = PIN_M1_EN1_GPIO_Port;
    ptr_motor2_config->en2_port = PIN_M1_EN2_GPIO_Port;
    ptr_motor2_config->fault_port = PIN_M1_FAULT_GPIO_Port;
    ptr_motor2_config->ipropi_port = PIN_M1_IPROPI_GPIO_Port;
    ptr_motor2_config->nsleep_pin = PIN_M1_NSLEEP_Pin;
    ptr_motor2_config->en1_pin = PIN_M1_EN1_Pin;
    ptr_motor2_config->en2_pin = PIN_M1_EN2_Pin;
    ptr_motor2_config->fault_pin = PIN_M1_FAULT_Pin;
    ptr_motor2_config->ipropi_pin = PIN_M1_IPROPI_Pin;
    ptr_motor2_config->enc_cpr = 64;
    ptr_motor2_config->wheel_pos_r = 0.145;
    ptr_motor2_config->wheel_pos_phi = M_PI;

    ptr_motor3_config->nsleep_port = PIN_M2_NSLEEP_GPIO_Port;
    ptr_motor3_config->en1_port = PIN_M2_EN1_GPIO_Port;
    ptr_motor3_config->en2_port = PIN_M2_EN2_GPIO_Port;
    ptr_motor3_config->fault_port = PIN_M2_FAULT_GPIO_Port;
    ptr_motor3_config->ipropi_port = PIN_M2_IPROPI_GPIO_Port;
    ptr_motor3_config->nsleep_pin = PIN_M2_NSLEEP_Pin;
    ptr_motor3_config->en1_pin = PIN_M2_EN1_Pin;
    ptr_motor3_config->en2_pin = PIN_M2_EN2_Pin;
    ptr_motor3_config->fault_pin = PIN_M2_FAULT_Pin;
    ptr_motor3_config->ipropi_pin = PIN_M2_IPROPI_Pin;
    ptr_motor3_config->enc_cpr = 64;
    ptr_motor3_config->wheel_pos_r = 0.145;
    ptr_motor3_config->wheel_pos_phi = 5.0f / 3.0f * M_PI;
}

void motor_init(MotorType *ptr_motor, MotorCfgType *ptr_motor_config, EncoderType *ptr_sw_enc, volatile uint32_t *effort_output_reg,
               TIM_HandleTypeDef *htim)
{


    ptr_motor->ptr_motor_config = ptr_motor_config;
    ptr_motor->ptr_sw_enc = ptr_sw_enc;
    ptr_motor->effort = 0;
    ptr_motor->effort_limit = 300;
    ptr_motor->linear_velocity = 0;
    ptr_motor->linear_velocity_setpoint = 0;
    ptr_motor->pwm_port = ptr_motor_config->en1_port;
    ptr_motor->pwm_pin = ptr_motor_config->en1_pin;
    ptr_motor->effort_output_reg = effort_output_reg;
    ptr_motor->last_enc_update = 0;
    ptr_motor->htim = htim;

    // Disable chip
    HAL_GPIO_WritePin(ptr_motor_config->nsleep_port, ptr_motor_config->nsleep_pin, RESET);
}

void motor_update(MotorType *ptr_motor)
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
        // HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, RESET); //Disable driver
        *(ptr_motor->effort_output_reg) = effort_epsilon;
        HAL_TIM_PWM_Stop_IT(ptr_motor->htim, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(ptr_motor->pwm_port, ptr_motor->pwm_pin, RESET);

        // motor_disable(ptr_motor);
    }

    // Compute velocity

    // Calculate the relation between an encoder pulse and
    // linear speed on the wheel where it contacts the ground
    // CCW is positive when looking from the motor towards the wheel
    if (ptr_motor->last_enc_update)
    {
        float dt_sec = (HAL_GetTick() - ptr_motor->last_enc_update) / 1000.0f;
        float pulse_to_speed_ratio = 1.0f / ptr_motor->ptr_motor_config->enc_cpr / GEAR_RATIO * 2.0f * M_PI / dt_sec * WHEEL_RADIUS;
        ptr_motor->linear_velocity = ptr_motor->ptr_sw_enc->counter * pulse_to_speed_ratio;
    }
    ptr_motor->last_enc_update = HAL_GetTick();
    ptr_motor->ptr_sw_enc->counter = 0; // reset counter
}

void motor_enable(MotorType *ptr_motor)
{
    HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, SET);
}

void motor_disable(MotorType *ptr_motor)
{
    HAL_GPIO_WritePin(ptr_motor->ptr_motor_config->nsleep_port, ptr_motor->ptr_motor_config->nsleep_pin, RESET);
}

void motor_debug(MotorType *ptr_motor)
{
    // printf("Vel: %ld\t", ptr_motor->linear_velocity);
    // printf("Effort: %ld\t", ptr_motor->effort);
}