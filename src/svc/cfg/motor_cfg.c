/**
 * @file motor_cfg.c
 * @brief PWM motor configuration source file, contains pinout selection
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "motor_cfg.h"

#include "peripheral.h"

/**
 * @brief Assigns all motor pins in the configuration struct
 *
 * @param pinout_m0 [out] Motor 0 pinout struct
 * @param pinout_m1 [out] Motor 1 pinout struct
 * @param pinout_m2 [out] Motor 2 pinout struct
 */
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
