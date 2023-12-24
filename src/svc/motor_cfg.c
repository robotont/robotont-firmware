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

void motor_configurePinout(MotorPinoutType *pinout_m0, MotorPinoutType *pinout_m1, MotorPinoutType *pinout_m2)
{
    pinout_m0->nsleep_pin.ptr_port = PIN_M0_NSLEEP_GPIO_Port;
    pinout_m0->en1_pin.ptr_port = PIN_M0_EN1_GPIO_Port;
    pinout_m0->en2_pin.ptr_port = PIN_M0_EN2_GPIO_Port;
    pinout_m0->fault_pin.ptr_port = PIN_M0_FAULT_GPIO_Port;
    pinout_m0->ipropi_pin.ptr_port = PIN_M0_IPROPI_GPIO_Port;
    pinout_m0->nsleep_pin.pin_number = PIN_M0_NSLEEP_Pin;
    pinout_m0->en1_pin.pin_number = PIN_M0_EN1_Pin;
    pinout_m0->en2_pin.pin_number = PIN_M0_EN2_Pin;
    pinout_m0->fault_pin.pin_number = PIN_M0_FAULT_Pin;
    pinout_m0->ipropi_pin.pin_number = PIN_M0_IPROPI_Pin;

    pinout_m1->nsleep_pin.ptr_port = PIN_M1_NSLEEP_GPIO_Port;
    pinout_m1->en1_pin.ptr_port = PIN_M1_EN1_GPIO_Port;
    pinout_m1->en2_pin.ptr_port = PIN_M1_EN2_GPIO_Port;
    pinout_m1->fault_pin.ptr_port = PIN_M1_FAULT_GPIO_Port;
    pinout_m1->ipropi_pin.ptr_port = PIN_M1_IPROPI_GPIO_Port;
    pinout_m1->nsleep_pin.pin_number = PIN_M1_NSLEEP_Pin;
    pinout_m1->en1_pin.pin_number = PIN_M1_EN1_Pin;
    pinout_m1->en2_pin.pin_number = PIN_M1_EN2_Pin;
    pinout_m1->fault_pin.pin_number = PIN_M1_FAULT_Pin;

    pinout_m2->nsleep_pin.ptr_port = PIN_M2_NSLEEP_GPIO_Port;
    pinout_m2->en1_pin.ptr_port = PIN_M2_EN1_GPIO_Port;
    pinout_m2->en2_pin.ptr_port = PIN_M2_EN2_GPIO_Port;
    pinout_m2->fault_pin.ptr_port = PIN_M2_FAULT_GPIO_Port;
    pinout_m2->ipropi_pin.ptr_port = PIN_M2_IPROPI_GPIO_Port;
    pinout_m2->nsleep_pin.pin_number = PIN_M2_NSLEEP_Pin;
    pinout_m2->en1_pin.pin_number = PIN_M2_EN1_Pin;
    pinout_m2->en2_pin.pin_number = PIN_M2_EN2_Pin;
    pinout_m2->fault_pin.pin_number = PIN_M2_FAULT_Pin;
    pinout_m2->ipropi_pin.pin_number = PIN_M2_IPROPI_Pin;
}
