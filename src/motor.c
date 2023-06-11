#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "motor.h"


void MotorInit(motor_t *hm, motor_config_t* cfg, sw_enc_t* enc)
{
    hm->cfg = cfg;
    hm->enc = enc;
    hm->effort = 0;
    hm->linear_velocity = 0;
    hm->linear_velocity_setpoint = 0;

    // Enable chip
    HAL_GPIO_WritePin(cfg->nsleep_port, cfg->nsleep_pin, SET);
}

void MotorUpdate(motor_t* hm)
{
    //hm->effort = 0.1;
    TIM3->CCR1 = hm->effort;
}


void MotorDebug(motor_t* hm)
{
    //printf("Vel: %ld\t", hm->linear_velocity);
    //printf("Effort: %ld\t", hm->effort);
}