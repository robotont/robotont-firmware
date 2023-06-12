#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "motor.h"

#define _USE_MATH_DEFINES
#include <math.h>

void MotorInit(motor_t *hm, motor_config_t* cfg, sw_enc_t* enc)
{
    hm->cfg = cfg;
    hm->enc = enc;
    hm->effort = 0;
    hm->linear_velocity = 0;
    hm->linear_velocity_setpoint = 0;
    hm->pwm_port = cfg->en1_port;
    hm->pwm_pin = cfg->en1_pin;

    // Enable chip
    HAL_GPIO_WritePin(cfg->nsleep_port, cfg->nsleep_pin, SET);
}

void MotorUpdate(motor_t* hm)
{
    float effort_epsilon = 1;

    if (hm->effort > effort_epsilon)
    {
        // Forward
        hm->pwm_port = hm->cfg->en1_port;
        hm->pwm_pin = hm->cfg->en1_pin;
        TIM3->CCR1 = abs(hm->effort);
        HAL_GPIO_WritePin(hm->cfg->en2_port, hm->cfg->en2_pin, RESET);
        HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, SET); //Enable driver
    }
    else if (hm->effort < -effort_epsilon)
    {
        // Reverse
        hm->pwm_port = hm->cfg->en2_port;
        hm->pwm_pin = hm->cfg->en2_pin;
        TIM3->CCR1 = abs(hm->effort);
        HAL_GPIO_WritePin(hm->cfg->en1_port, hm->cfg->en1_pin, RESET);
        HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, SET); //Enable driver
    }
    else
    {
        // effort is inbetween [-epsilon...epsilon]
        HAL_GPIO_WritePin(hm->cfg->nsleep_port, hm->cfg->nsleep_pin, RESET); //Disable driver
        TIM3->CCR1 = 0;
    }



    // Compute velocity

    // Calculate the relation between an encoder pulse and 
    // linear speed on the wheel where it contacts the ground
    // CCW is positive when looking from the motor towards the wheel
    float pulse_to_speed_ratio = -1.0f / hm->cfg->enc_cpr / hm->cfg->gear_ratio * 2 * M_PI / hm->cfg->pid_dt * hm->cfg->wheel_radius;
    hm->linear_velocity = hm->enc->counter * pulse_to_speed_ratio;
    hm->enc->counter = 0; // reset counter
}


void MotorDebug(motor_t* hm)
{
    //printf("Vel: %ld\t", hm->linear_velocity);
    //printf("Effort: %ld\t", hm->effort);
}