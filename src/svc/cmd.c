/**
 * @file cmd.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "cmd.h"

#include "movement.h"
#include "usbif.h"

#define ARG_ROBOT_SPEED    0x5253 // "RS"
#define ARG_MOTOR_SPEED    0x4D53 // "MS"
#define ARG_ODOM_RESET     0x4F52 // "OR"
#define ARG_EFFORT_CONTROL 0x4546 // "EF"
#define ARG_LED_CONTROL    0x4C44 // "LD"
#define ARG_LED_MODE       0x4C4D // "LM"

/**
 * @brief Inits usbif
 */
void cmd_init(void)
{
    usbif_init();
    usbif_setUpperLayerCallback((ReceiveCallbackType)cmd_receiveData);
}

/**
 * @brief
 * @param ptr_data
 * @param lenght
 */
void cmd_receiveData(uint8_t *ptr_data, uint16_t lenght)
{
    uint16_t cmd_argument  = (ptr_data[0] << 8U) | ptr_data[1];
    switch (cmd_argument)
    {
        case ARG_ROBOT_SPEED:
            movement_handleCommandsRS(&ptr_data[3], lenght - 3U);
            break;

        case ARG_MOTOR_SPEED:
            movement_handleCommandsMS(&ptr_data[3], lenght - 3U);
            break;

        case ARG_ODOM_RESET:
            movement_handleCommandsOR(&ptr_data[3], lenght - 3U);
            break;

        case ARG_EFFORT_CONTROL:
            movement_handleCommandsEF(&ptr_data[3], lenght - 3U);
            break;
        
        case ARG_LED_CONTROL:
            led_handleCommandsLD(&ptr_data[3], lenght - 3U);
            break;
        
        case ARG_LED_MODE:
            led_handleCommandsLM(&ptr_data[3], lenght - 3U);
            break;

        default:
            // TODO: notify user about bad argument?
            break;
    }
}

/**
 * @brief
 * @param ptr_data
 * @param lenght
 */
void cmd_transmitData(uint8_t *ptr_data, uint16_t lenght)
{
    #pragma "not implemented"
}