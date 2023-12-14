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

/**
 * @brief Inits usbif and sets usbif callback to `cmd_handleUsbData`
 */
void cmd_init(void)
{
    usbif_init();
    usbif_setUpperLayerCallback((ReceiveCallbackType)cmd_handleUsbData);
}

/**
 * @brief USB RX interrupt handler.
 * Takes raw string, cuts off 1st argument and CR+LF, sends data to the corresponding module
 * @note Called within ISR context from lower layer (usbcdc -> usbif -> cmd)
 * @param ptr_data Raw string in the format `ARG:VALUE_1:...:VALUE_N/r/n`
 * @param lenght Lenght of the whole string (including CR + LF)
 */
void cmd_handleUsbData(uint8_t *ptr_data, uint16_t lenght)
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
    usbif_transmit(ptr_data, lenght);
}
