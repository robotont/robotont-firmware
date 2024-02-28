/**
 * @file movement.h
 * @brief Service. Makes robot move using omnimotional movement
 *
 * Robot is able to move by controlling speeds of three PWM motors.
 * Speed is set by the user via serial (USB). If no data received within ~1s, then robot will stopped automaticly.
 * Robot calculates and sends back odometry data via serial as well.
 * All data is transfered as a string (char array) in the format "ARG:VALUE_0:...:VALUE_N\r\n".
 *
 * @note "\r\n" is used as a packet separator, so data will be collected until those symbols are received.
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <stdint.h>

#include "motor.h"
#include "system_hal.h"

void movement_init();

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsDC(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght);

void movement_update();

void movement_pwmHighCallback(TIM_HandleTypeDef *timer_handler);
void movement_pwmLowCallback(TIM_HandleTypeDef *timer_handler);

#endif
