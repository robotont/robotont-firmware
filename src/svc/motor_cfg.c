#include "motor_cfg.h"

#include "peripheral.h"

void motor_cfg_setConfig(MotorCfgType *ptr_m0_config, MotorCfgType *ptr_m1_config, MotorCfgType *ptr_m2_config)
{
    ptr_m0_config->nsleep_port = PIN_M0_NSLEEP_GPIO_Port;
    ptr_m0_config->en1_port = PIN_M0_EN1_GPIO_Port;
    ptr_m0_config->en2_port = PIN_M0_EN2_GPIO_Port;
    ptr_m0_config->fault_port = PIN_M0_FAULT_GPIO_Port;
    ptr_m0_config->ipropi_port = PIN_M0_IPROPI_GPIO_Port;
    ptr_m0_config->nsleep_pin = PIN_M0_NSLEEP_Pin;
    ptr_m0_config->en1_pin = PIN_M0_EN1_Pin;
    ptr_m0_config->en2_pin = PIN_M0_EN2_Pin;
    ptr_m0_config->fault_pin = PIN_M0_FAULT_Pin;
    ptr_m0_config->ipropi_pin = PIN_M0_IPROPI_Pin;

    ptr_m1_config->nsleep_port = PIN_M1_NSLEEP_GPIO_Port;
    ptr_m1_config->en1_port = PIN_M1_EN1_GPIO_Port;
    ptr_m1_config->en2_port = PIN_M1_EN2_GPIO_Port;
    ptr_m1_config->fault_port = PIN_M1_FAULT_GPIO_Port;
    ptr_m1_config->ipropi_port = PIN_M1_IPROPI_GPIO_Port;
    ptr_m1_config->nsleep_pin = PIN_M1_NSLEEP_Pin;
    ptr_m1_config->en1_pin = PIN_M1_EN1_Pin;
    ptr_m1_config->en2_pin = PIN_M1_EN2_Pin;
    ptr_m1_config->fault_pin = PIN_M1_FAULT_Pin;

    ptr_m2_config->nsleep_port = PIN_M2_NSLEEP_GPIO_Port;
    ptr_m2_config->en1_port = PIN_M2_EN1_GPIO_Port;
    ptr_m2_config->en2_port = PIN_M2_EN2_GPIO_Port;
    ptr_m2_config->fault_port = PIN_M2_FAULT_GPIO_Port;
    ptr_m2_config->ipropi_port = PIN_M2_IPROPI_GPIO_Port;
    ptr_m2_config->nsleep_pin = PIN_M2_NSLEEP_Pin;
    ptr_m2_config->en1_pin = PIN_M2_EN1_Pin;
    ptr_m2_config->en2_pin = PIN_M2_EN2_Pin;
    ptr_m2_config->fault_pin = PIN_M2_FAULT_Pin;
    ptr_m2_config->ipropi_pin = PIN_M2_IPROPI_Pin;
}
