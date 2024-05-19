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
    NONE,          /* No mode */
    SPIN,          /* LED spin */
    PULSE,         /* Leds pulse on/off */
    COLORS_SMOOTH, /* Change colours */
    WHEEL_COLORS,  /* Behind the wheel change colours */
    COLORS_RGB,    /* Switch all leds between red, green, blue */
    COLORS_SPIN,   /* Change colours spin */
    MOTOR_DUTY,    /* Motor duty cycle changes wheel colors */
    MOTOR_SPEEDS,  /* Motor speed changes wheel colors */
    SCAN_RANGES    /* Laser scan ranges */
} LEDMode;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t speed; /* Update speed in Hz (1 - 50) */
    uint8_t scan_ranges[3];
} ModeParameters;

LEDMode led_mode;
ModeParameters led_mode_params;

void led_init();
void led_handleCommandsLD(uint8_t *ptr_data, uint16_t lenght);
void led_handleCommandsLM(uint8_t *ptr_data, uint16_t lenght);
void led_handleCommandsLS(uint8_t *ptr_data, uint16_t lenght);
void led_update();

#endif