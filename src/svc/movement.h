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

void movement_init(MotorType *ptr_m0, MotorType *ptr_m1, MotorType *ptr_m2);

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsEF(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght);

void movement_update(uint32_t update_time_ms);

#endif
