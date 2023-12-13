#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmd.h"
#include "motor.h"
#include "movement.h"
#include "odom.h"
#include "peripheral.h"
#include "pid.h"
#include "sw_enc.h"
#include "system_hal.h"
#include "usbif.h"
#include "gpioif.h"
#include "led.h"

#define MAX_LIN_VEL 0.4 // m/s
#define MAX_ANG_VEL 1.0 // rad/s

MotorType hm0, hm1, hm2;
EncoderType henc0, henc1, henc2;

int main(void)
{
    system_hal_init();
    peripheral_init();

    sw_enc_init(&henc0, PIN_M0_ENCA_GPIO_Port, PIN_M0_ENCA_Pin, PIN_M0_ENCB_GPIO_Port, PIN_M0_ENCB_Pin);
    sw_enc_init(&henc1, PIN_M1_ENCA_GPIO_Port, PIN_M1_ENCA_Pin, PIN_M1_ENCB_GPIO_Port, PIN_M1_ENCB_Pin);
    sw_enc_init(&henc2, PIN_M2_ENCA_GPIO_Port, PIN_M2_ENCA_Pin, PIN_M2_ENCB_GPIO_Port, PIN_M2_ENCB_Pin);
    // Start timer for reading encoders
    HAL_TIM_Base_Start_IT(&htim14);

    // Service layer
    cmd_init();
    movement_init(&hm0, &hm1, &hm2, &henc0, &henc1, &henc2);
    led_init();

    HAL_Delay(1000); // TODO investigate, is this required?

    uint32_t counter = 0; // for debugging purposes
    uint32_t duty = 0;    // for debugging purposes

    uint32_t last_tick = HAL_GetTick();
    uint32_t current_tick;

    counter = 1;
    duty = 50;

    GpioPinType led_green;
    led_green.pin_number = PIN_LED_Pin;
    led_green.ptr_port = PIN_LED_GPIO_Port;

    while (true)
    {
        current_tick = HAL_GetTick();
        if (current_tick >= last_tick + MAIN_LOOP_DT_MS)
        {
            last_tick = current_tick;
            counter++;

            /* Service layer modules update */
            movement_update();
            led_update();
            // TODO [implementation] here goes "led_update()", "oled_update()" ...

            /* Debug info */
            if (counter % 100 == 0)
            {
                gpioif_togglePin(&led_green);
                printf("Main_delay:%ld %ld\r\n", current_tick, last_tick);
            }
        }
    }
}

/**
 * @brief  Retargets the C library printf function to the VIRTUAL COM PORT.
 * @param  None
 * @retval None
 */
int _write(int file, char *ptr, int len)
{
    static uint8_t rc = USBD_OK;

    // loop commented out for a non-blocking behavior
    // do {
    rc = CDC_Transmit_FS((unsigned char *)ptr, len);
    //} while (USBD_BUSY == rc);
    if (USBD_FAIL == rc)
    {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}

// TODO interrupts handlers

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Set PWM pin to high depending on which pwm timer triggered the interrupt
    if (htim->Instance == htim3.Instance)
    {
        HAL_GPIO_WritePin(hm0.pwm_port, hm0.pwm_pin, SET);
    }
    else if (htim->Instance == htim11.Instance)
    {
        HAL_GPIO_WritePin(hm1.pwm_port, hm1.pwm_pin, SET);
    }
    else if (htim->Instance == htim13.Instance)
    {
        HAL_GPIO_WritePin(hm2.pwm_port, hm2.pwm_pin, SET);
    }
    // Check if the timer for the encoder interrupt triggered
    else if (htim->Instance == htim14.Instance)
    {
        sw_enc_interrupt(&henc0);
        sw_enc_interrupt(&henc1);
        sw_enc_interrupt(&henc2);
    }
}

/**
 * @brief  This function is executed when the PWM pulses finish. Depending on which timer triggered the interrupt, the
 * corresponding PWM pin is set to low.
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim3.Instance) // motor 0
    {
        HAL_GPIO_WritePin(hm0.pwm_port, hm0.pwm_pin, RESET);
    }
    else if (htim->Instance == htim11.Instance) // motor 1
    {
        HAL_GPIO_WritePin(hm1.pwm_port, hm1.pwm_pin, RESET);
    }
    else if (htim->Instance == htim13.Instance) // motor 2
    {
        HAL_GPIO_WritePin(hm2.pwm_port, hm2.pwm_pin, RESET);
    }
}
