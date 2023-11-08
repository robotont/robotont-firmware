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

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsEF(uint8_t *ptr_data, uint16_t lenght);
void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght);


#endif
