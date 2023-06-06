#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "sw_enc.h"

typedef struct
{
    GPIO_TypeDef* nsleep_port;
    GPIO_TypeDef* en1_port;
    GPIO_TypeDef* en2_port;
    GPIO_TypeDef* fault_port;
    GPIO_TypeDef* ipropi_port;
    uint16_t nsleep_pin;
    uint16_t en1_pin;
    uint16_t en2_pin;
    uint16_t fault_pin;
    uint16_t ipropi_pin;
    //Default PID parameters
    float pid_k_p;
    float pid_tau_i;
    float pid_tau_d;
    float pid_dt;

    uint16_t enc_cpr; // encoder counts per revolution
    float gear_ratio; // gearbox reduction ratio
    float wheel_radius; // wheel outer radius

    // wheel position in polar coordinates
    float wheel_pos_r; // distance from center
    float wheel_pos_phi; // angle relative to x-axis (forward)
} motor_config_t;

typedef struct
{
    motor_config_t* cfg;
    sw_enc_t* enc;
    int32_t velocity;
    uint32_t effort;
} motor_t;

void MotorInit(motor_t *hm, motor_config_t* cfg, sw_enc_t* enc);
void MotorUpdate(motor_t* hm);
void MotorDebug(motor_t* hm);

#endif