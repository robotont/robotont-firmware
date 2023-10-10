/**
 * @file cmd.h
 * @brief 
 * 
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023
 */

#ifndef __CMD_H__
#define __CMD_H__

#include <stdint.h>
#include "usbif.h"

#define CMD_ARG_ROBOT_SPEED     "RS"
#define CMD_ARG_MOTOR_SPEED     "MS"
#define CMD_ARG_ODOM_RESET      "OR"
#define CMD_ARG_ODOM_FEEDBACK   "ODOM"
#define CMD_ARG_DEBUG           "DEBUG"

// TODO think about proper handling
// typedef enum {
//     CMD_RX_SERIAL_DATA,
//     CMD_TX_SERIAL_DATA,
// } CmdEventType;

void cmd_init(void);
void cmd_receiveSerialData(uint8_t *ptr_data, uint16_t lenght); /* Called within ISR context from lower layer */
void cmd_transmitSerialData(uint8_t *ptr_data, uint16_t lenght);

#endif