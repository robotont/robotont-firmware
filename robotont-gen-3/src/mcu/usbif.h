/**
 * @file usbif.h
 * @brief 
 * 
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023
 */

#ifndef __USBIF_H__
#define __USBIF_H__

#include <stdint.h>
#include "usbd_cdc_if.h"

#define USBIF_BUFFER_SIZE APP_RX_DATA_SIZE

typedef struct UsbifPacketType
{
    uint8_t buffer[USBIF_BUFFER_SIZE];
    uint16_t lenght;
} UsbifPacketType;

// TODO replace with callback to the upper layer (CMD_HANDLER)
extern uint16_t bytes_received;
extern uint16_t last_packet_length;
extern uint8_t packet_buf[APP_RX_DATA_SIZE];
extern uint8_t last_packet[APP_RX_DATA_SIZE];
// -----------------------------------------------------------

void usbif_init(void);
uint8_t usbif_transmit(uint8_t *ptr_data, uint16_t lenght); // TODO replace arg to PacketType packet
uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght);  /* Called withing ISR context from lower layer*/
void usbif_setUpperLayerCallback(ReceiveCallbackType rx_callback);

#endif