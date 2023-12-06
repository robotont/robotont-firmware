#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "sw_enc.h"
#include "gpioif.h"

typedef struct
{
    GPIO_TypeDef *nsleep_port;
    GPIO_TypeDef *en1_port;
    GPIO_TypeDef *en2_port;
    GPIO_TypeDef *fault_port;
    GPIO_TypeDef *ipropi_port;
    uint16_t nsleep_pin;
    uint16_t en1_pin;
    uint16_t en2_pin;
    uint16_t fault_pin;
    uint16_t ipropi_pin;
} MotorCfgType;

typedef struct
{
    MotorCfgType *ptr_motor_config;
    EncoderType *ptr_sw_enc;
    double linear_velocity;
    double linear_velocity_setpoint;
    double effort;
    double effort_limit;
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    volatile uint32_t *effort_output_reg;
    TIM_HandleTypeDef *htim;
    uint32_t last_enc_update;

} MotorType; 

void motor_init(MotorType *ptr_motor, MotorCfgType *ptr_motor_config, EncoderType *ptr_sw_enc,
                volatile uint32_t *effort_output_reg, TIM_HandleTypeDef *htim);
void motor_update(MotorType *ptr_motor);
void motor_debug(MotorType *ptr_motor);
void motor_enable(MotorType *ptr_motor);
void motor_disable(MotorType *ptr_motor);

#endif