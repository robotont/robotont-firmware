/**
 * @file movement.h
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <stdint.h>

#include "motor.h"
#include "sw_enc.h"
#include "system_hal.h"

void movement_init(MotorHandleType *m0_handler, MotorHandleType *m1_handler, MotorHandleType *m2_handler);

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsEF(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght);

void movement_update();

void movement_pwmHighCallback(TIM_HandleTypeDef *timer_handler);
void movement_pwmLowCallback(TIM_HandleTypeDef *timer_handler);

#endif
