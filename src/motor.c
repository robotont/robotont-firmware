#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "motor.h"

#define _USE_MATH_DEFINES
#include <math.h>

void MotorInit(motor_t *hm, motor_config_t* cfg, sw_enc_t* enc, volatile uint32_t* effort_output_reg, TIM_HandleTypeDef* htim)
{
    hm->cfg = cfg;
    hm->enc = enc;
    hm->effort = 0;
    hm->effort_limit = 300;
    hm->linear_velocity = 0;
    hm->linear_velocity_setpoint = 0;
    hm->pwm_port = cfg->en1_port;
    hm->pwm_pin = cfg->en1_pin;
    hm->effort_output_reg = effort_output_reg;
    hm->last_enc_update = 0;
    hm->htim = htim;

    // Disable chip
    HAL_GPIO_WritePin(cfg->nsleep_port, cfg->nsleep_pin, RESET);
}

void MotorUpdate(motor_t* hm)
{
    double effort_epsilon = 100; // this is a counter value from where the motor exceeds its internal friction, also instabilities in PWM generation occured with lower values.

    if (hm->effort > effort_epsilon)
    {
        // Forward
        hm->pwm_port = hm->cfg->en1_port;
        hm->pwm_pin = hm->cfg->en1_pin;
        *(hm->effort_output_reg) = abs(hm->effort);
        HAL_GPIO_WritePin(hm->cfg->en2_port, hm->cfg->en2_pin, RESET);
        //MotorEnable(hm);
        HAL_TIM_PWM_Start_IT(hm->htim, TIM_CHANNEL_1);
    }
    else if (hm->effort < -effort_epsilon)
    {
        // Reverse
        hm->pwm_port = hm->cfg->en2_port;
        hm->pwm_pin = hm->cfg->en2_pin;
        *(hm->effort_output_reg) = abs(hm->effort);
        HAL_GPIO_WritePin(hm->cfg->en1_port, hm->cfg->en1_pin, RESET);
        //MotorEnable(hm);
        HAL_TIM_PWM_Start_IT(hm->htim, TIM_CHANNEL_1);
    }
    else
    {
        // effort is inbetween [-epsilon...epsilon]
        // HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, RESET); //Disable driver
        *(hm->effort_output_reg) = effort_epsilon;
        HAL_TIM_PWM_Stop_IT(hm->htim, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(hm->pwm_port, hm->pwm_pin, RESET);

        //MotorDisable(hm);
    }


    // Compute velocity

    // Calculate the relation between an encoder pulse and 
    // linear speed on the wheel where it contacts the ground
    // CCW is positive when looking from the motor towards the wheel
    if (hm->last_enc_update) 
    {
        float dt_sec = (HAL_GetTick() - hm->last_enc_update) / 1000.0f;
        float pulse_to_speed_ratio = 1.0f / hm->cfg->enc_cpr / hm->cfg->gear_ratio * 2 * M_PI / dt_sec * hm->cfg->wheel_radius;
        hm->linear_velocity = hm->enc->counter * pulse_to_speed_ratio;
    }
    hm->last_enc_update = HAL_GetTick();
    hm->enc->counter = 0; // reset counter
}

    void MotorEnable(motor_t* hm)
    {
        HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, SET);
    }

    void MotorDisable(motor_t* hm)
    {
        HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, RESET);
    }


void MotorDebug(motor_t* hm)
{
    //printf("Vel: %ld\t", hm->linear_velocity);
    //printf("Effort: %ld\t", hm->effort);
}