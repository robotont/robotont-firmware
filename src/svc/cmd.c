/**
 * @file cmd.h
 * @brief Service. Processes incomind and outcoming command packets.
 *
 * RX: Parses CMD arguments and sends data to the required service (e.g. "MS" -> movement service)
 * TX: Data transmit is performed by printf, that is configured in this module
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "cmd.h"

#include "led.h"
#include "movement.h"
#include "usbif.h"

#define ARG_ROBOT_SPEED        0x5253 /* "RS" */
#define ARG_MOTOR_SPEED        0x4D53 /* "MS" */
#define ARG_ODOM_RESET         0x4F52 /* "OR" */
#define ARG_DUTY_CYCLE_CONTROL 0x4443 /* "DC" */
#define ARG_LED_CONTROL        0x4C44 /* "LD" */
#define ARG_LED_MODE           0x4C4D /* "LM" */
#define ARG_LED_SEGMENT        0x4C53 /* "LS" */

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
 * Takes raw string, cuts off 1st argument ("XX:") sends data to the corresponding module
 * @note Called within ISR context from lower layer (usbcdc -> usbif -> cmd)
 * @note CR + LF already excluded in the `usbif`
 * @param ptr_data Raw string in the format `ARG:VALUE_1:...:VALUE_N`
 * @param lenght Lenght of the raw string
 */
void cmd_handleUsbData(uint8_t *ptr_data, uint16_t lenght)
{
    uint16_t cmd_argument = (ptr_data[0] << 8u) | ptr_data[1];
    switch (cmd_argument)
    {
        case ARG_ROBOT_SPEED:
            movement_handleCommandsRS(&ptr_data[3], lenght - 3u);
            break;

        case ARG_MOTOR_SPEED:
            movement_handleCommandsMS(&ptr_data[3], lenght - 3u);
            break;

        case ARG_ODOM_RESET:
            movement_handleCommandsOR(&ptr_data[3], lenght - 3u);
            break;

        case ARG_DUTY_CYCLE_CONTROL:
            movement_handleCommandsDC(&ptr_data[3], lenght - 3u);
            break;

        case ARG_LED_CONTROL:
            led_handleCommandsLD(&ptr_data[3], lenght - 3U);
            break;

        case ARG_LED_MODE:
            led_handleCommandsLM(&ptr_data[3], lenght - 3U);
            break;

        case ARG_LED_SEGMENT:
            led_handleCommandsLS(&ptr_data[3], lenght - 3U);
            break;

        default:
            // TODO: notify user about bad argument?
            break;
    }
}

/**
 * @brief  Retargets the C library printf function to the VIRTUAL COM PORT.
 */
int _write(int file, char *ptr_data, int len)
{
    (void)file; // Not used
    return usbif_transmit((uint8_t *)ptr_data, (uint16_t)len);
}