/**
 * @file main.c
 * @brief Application layer code. Defines, how each service communicates with each other
 * @note Example, of how to modules should communicate with each others: via getters and setters (Pseudocode):
 *
    status = battery_monitor_getStatus();
    if (status == STATUS_12V_OVERVOLTAGE)
    {
        led_blinkRed();
        movement_stop();
    }
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com) ... TODO: Add other's who contributed to this file
 * @copyright Copyright (c) 2024 Tartu Ülikool
 */

#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmd.h"
#include "led.h"
#include "menu.h"
#include "movement.h"
#include "peripheral.h"
#include "system_hal.h"

#define DEBUG_LED_PERIOD_PRESCALER 10u

IoPinType led_green;
IoPinType led_red;

static void initDebugLeds(void)
{
    led_green.pin_number = PIN_LED_G_Pin;
    led_green.ptr_port = PIN_LED_G_GPIO_Port;
    led_red.pin_number = PIN_LED_R_Pin;
    led_red.ptr_port = PIN_LED_R_GPIO_Port;
    ioif_togglePin(&led_green); // First toggle needed, to allow LEDs to blink alternately
}

static void toggleDebugLeds(void)
{
    ioif_togglePin(&led_green);
    ioif_togglePin(&led_red);
}

int main(void)
{
    system_hal_init();
    peripheral_init(); // Initialized peripheral, that don't have interface yet; To be removed in the future

    cmd_init();
    led_init();
    menu_init();
    movement_init();
    initDebugLeds();

    uint32_t debug_counter = 1u;
    uint32_t last_tick = system_hal_timestamp();
    uint32_t current_tick;

    while (true)
    {
        current_tick = system_hal_timestamp();
        if (current_tick >= last_tick + MAIN_LOOP_DT_MS)
        {
            last_tick = current_tick;
            debug_counter++;

            // Service layer modules update
            movement_update();
            led_update();
            menu_update();

            // Debug
            if (debug_counter % DEBUG_LED_PERIOD_PRESCALER == 0)
            {
                toggleDebugLeds();
            }
        }
    }
}
