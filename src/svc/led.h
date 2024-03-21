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

typedef enum 
{
    LED_MODE_SPIN,          // LED spin green
    LED_MODE_PULSE,         // Leds pulse on/off
    LED_MODE_COLORS_SMOOTH, // Change colours
    LED_MODE_WHEEL_COLORS,  // Behind the wheel change colours
    LED_MODE_COLORS_RGB,    // Switch all leds between red, green, blue
    LED_MODE_COLORS_SPIN,   // Change colours spin
    LED_MODE_MOTOR_SPEEDS,  // Motor speed changes wheel colors
    LED_MODE_SCAN_RANGES    // Laser scan ranges
} LEDMode;

LEDMode led_mode;

void led_init();
void led_handleCommandsLD(uint8_t *ptr_data, uint16_t lenght);
void led_handleCommandsLM(uint8_t *ptr_data, uint16_t lenght);
void led_handleCommandsLS(uint8_t *ptr_data, uint16_t lenght);
void led_update();

#endif