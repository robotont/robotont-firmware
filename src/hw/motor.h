#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

#include "ioif.h"
#include "stm32f4xx_hal.h"
#include "sw_enc.h"
#include "ioif.h"

// TODO remove
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
    IoPinType nsleep_pin;
    IoPinType en1_pin;
    IoPinType en2_pin;
    IoPinType fault_pin;
    IoPinType ipropi_pin;

} MotorPinoutType;


typedef struct
{
    MotorPinoutType *pinout;

    MotorCfgType *ptr_motor_config;

    
    double linear_velocity;
    double linear_velocity_setpoint;
    double effort;
    double effort_limit;
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    volatile uint32_t *effort_output_reg;
    TIM_HandleTypeDef *htim;
    uint32_t last_enc_update;

} MotorHandleType;

void motor_init(MotorHandleType *ptr_motor, MotorCfgType *ptr_motor_config, TIM_HandleTypeDef *pwm_timer);
void motor_update(MotorHandleType *ptr_motor);
void motor_debug(MotorHandleType *ptr_motor);
void motor_enable(MotorHandleType *ptr_motor);
void motor_disable(MotorHandleType *ptr_motor);

#endif