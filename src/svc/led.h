/**
 * @file led.h
 * @brief
 *
 * @author Raimo Köidam (raimokoidam@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>
#include "ARGB.h"

void led_init();
void led_handleCommandsLD(uint8_t *ptr_data, uint16_t lenght);
void led_update();

#endif
