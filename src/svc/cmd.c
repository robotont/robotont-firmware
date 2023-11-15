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

uint8_t last_packet[APP_RX_DATA_SIZE];
uint16_t last_packet_length = 0;

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
    uint16_t cmd_argument;

    memcpy(last_packet, ptr_data, lenght); // TODO get rid of global var, use callbacks
    last_packet_length = lenght;

    cmd_argument = (ptr_data[0] << 8U) | ptr_data[1];
    ptr_data += 3U; // 2 bytes of argument and 1 byte of separator ":". //! P.S. This is MISRA violation tho
    lenght -= 3U;

    switch (cmd_argument)
    {
        case ARG_ROBOT_SPEED:
            movement_handleCommandsRS(ptr_data, lenght);
            break;

        case ARG_MOTOR_SPEED:
            movement_handleCommandsMS(ptr_data, lenght);
            break;

        case ARG_ODOM_RESET:
            movement_handleCommandsOR(ptr_data, lenght);
            break;

        case ARG_EFFORT_CONTROL:
            movement_handleCommandsEF(ptr_data, lenght);
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