/**
 * @file cmd.h
 * @brief 
 * 
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef __CMD_H__
#define __CMD_H__

#include <stdint.h>
#include "usbif.h"

// TODO think about proper handling
// typedef enum {
//     CMD_RX_SERIAL_DATA,
//     CMD_TX_SERIAL_DATA,
// } CmdEventType;

extern uint16_t last_packet_length;
extern uint8_t last_packet[USBIF_BUFFER_SIZE];

void cmd_init(void);
void cmd_receiveData(uint8_t *ptr_data, uint16_t lenght); /* Called within ISR context from lower layer */
void cmd_transmitData(uint8_t *ptr_data, uint16_t lenght);

#endif