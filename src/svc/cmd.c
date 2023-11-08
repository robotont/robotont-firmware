/**
 * @file cmd.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "cmd.h"
#include "usbif.h"
#include "movement.h"

uint8_t last_packet[APP_RX_DATA_SIZE];
uint16_t last_packet_length = 0;

/**
 * @brief
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
    memcpy(last_packet, ptr_data, lenght); // TODO get rid of global var, use callbacks
    last_packet_length = lenght;

    
    // Command: RS (Robot Speed)
    // Command: MS (Motor Speed)
    // Command: EF (Effort control)
    // Command: OR (Odom Reset)

    // Print out some debugging information at 10 lower rate.

    // If no velocity command has been received within the timeout period, stop all motors

    // Update motors (PID)

    // Send odometry command to the on-board computer

    // Wait until the desired loop time has elapsed

}

/**
 * @brief
 * @param ptr_data
 * @param lenght
 */
void cmd_transmitData(uint8_t *ptr_data, uint16_t lenght)
{
}