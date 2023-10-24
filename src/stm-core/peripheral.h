/**
 * @file peripheral.h
 * @brief Contains auto-generated CUBEMX code. // TODO Create interface for each perepheral
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef _PERIPHERAL_H_
#define _PERIPHERAL_H_

#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart3;

void peripheral_init(void);

#endif