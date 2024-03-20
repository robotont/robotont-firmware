#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmd.h"
#include "ioif.h"
#include "motor.h"
#include "movement.h"
#include "odom.h"
#include "peripheral.h"
#include "pid.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "system_hal.h"
#include "timerif.h"
#include "usbif.h"
#include "led.h"
#include "measurements.h"
#include "menu.h"

#define ATtiny_ADR 0x08
uint8_t I2C1_data[8];

int main(void)
{
    system_hal_init();
    peripheral_init();
    ioif_init();
    timerif_init();
    i2cif_init();

    HAL_I2C_EnableListen_IT(&hi2c2); // TODO measurements_init()
    cmd_init();
    movement_init();
    led_init();
    menu_init();

    uint32_t counter = 1u;
    uint32_t last_tick = system_hal_timestamp();
    uint32_t current_tick;

    IoPinType led_green;
    led_green.pin_number = PIN_LED_G_Pin;
    led_green.ptr_port = PIN_LED_G_GPIO_Port;

    IoPinType led_red;
    led_red.pin_number = PIN_LED_R_Pin;
    led_red.ptr_port = PIN_LED_R_GPIO_Port;
    ioif_togglePin(&led_green);
    if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
    {
	    ioif_writePin(&led_red, 1);
    }

    while (true)
    {
        current_tick = system_hal_timestamp();
        if (current_tick >= last_tick + MAIN_LOOP_DT_MS)
        {
            last_tick = current_tick;
            counter++;

            /* Service layer modules update */
            movement_update();

            /**
            @brief Example, of how to modules should communicate with each others: via getters and setters (Pseudocode)

            status = battery_monitor_getStatus();
            if (status == STATUS_12V_OVERVOLTAGE)
            {
                led_blinkRed();
                movement_stop();
            }

            */

            // TODO [implementation] here goes "led_update()", "oled_update()" ...
            led_update();

            /* Debug info */
            if (counter % 10u == 0)
            {   
                menu_update();
                ioif_togglePin(&led_green);
                ioif_togglePin(&led_red);
                
            }

        }
    }
}
