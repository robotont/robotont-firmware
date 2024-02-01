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

#define MAX_LIN_VEL 0.4 // m/s
#define MAX_ANG_VEL 1.0 // rad/s

int main(void)
{
    system_hal_init();
    peripheral_init();
    ioif_init();
    timerif_init();
    i2cif_init();

    cmd_init();
    movement_init();

    uint32_t counter = 0; // for debugging purposes
    uint32_t duty = 0;    // for debugging purposes

    uint32_t last_tick = HAL_GetTick();
    uint32_t current_tick;

    counter = 1;
    duty = 50;

    IoPinType led_green;
    led_green.pin_number = PIN_LED_G_Pin;
    led_green.ptr_port = PIN_LED_G_GPIO_Port;

    IoPinType led_red;
    led_red.pin_number = PIN_LED_R_Pin;
    led_red.ptr_port = PIN_LED_R_GPIO_Port;
    ioif_togglePin(&led_green);

    int16_t effort = 90;
    // MX_I2C1_Init();
    MX_I2C2_Init();
    // MX_I2C3_Init();
    ssd1306_Init();

    while (true)
    {
        ssd1306_TestFPS();
        system_hal_delay(5000);
#if 0
        current_tick = HAL_GetTick();
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

            /* Debug info */
            if (counter % 100u == 0)
            {
                ioif_togglePin(&led_green);
                ioif_togglePin(&led_red);
                // printf("Main_delay:%ld %ld\r\n", current_tick, last_tick);

                // effort += 10;
                // if (effort > 160)
                // {
                //     effort = 90;
                // }
                // timerif_setEffort(TIMER_PWM_M0, effort);
                // timerif_setEffort(TIMER_PWM_M1, effort);
                // timerif_setEffort(TIMER_PWM_M2, effort);
            }

            // printf("%05d %05d %05d\r\n", timerif_getCounter(TIMER_ENC_M0), timerif_getCounter(TIMER_ENC_M1),
            //        timerif_getCounter(TIMER_ENC_M2));
        }
#endif
    }
}
