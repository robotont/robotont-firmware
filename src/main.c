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

    // Service layer
    cmd_init();
    movement_init(&hm0, &hm1, &hm2, &henc0, &henc1, &henc2);

    // Start timers for motor PWM generation (gpios are SET in periodelapsedCallback and RESET in pulseFinishedCallback)
    HAL_TIM_Base_Start_IT(&htim3); // motor 0
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim11); // motor 1
    HAL_TIM_PWM_Start_IT(&htim11, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim13); // motor 2
    HAL_TIM_PWM_Start_IT(&htim13, TIM_CHANNEL_1);

    // Start timer for reading encoders
    HAL_TIM_Base_Start_IT(&htim14);
    // HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);f

    HAL_Delay(1000);

    uint32_t counter = 0; // for debugging purposes
    uint32_t duty = 0;    // for debugging purposes

    uint32_t last_tick = HAL_GetTick();
    uint32_t current_tick;

    counter = 1;
    duty = 50;
    while (true)
    {
        current_tick = HAL_GetTick();
        if (current_tick > last_tick + MAIN_LOOP_DT_MS)
        {
            last_tick = current_tick;
            counter++;

            /* Service layer modules update */
            movement_update();
            // TODO [implementation] here goes "led_update()", "oled_update()" ...

            /* Debug info */
            if (counter % 100 == 0)
            {
                HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
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
