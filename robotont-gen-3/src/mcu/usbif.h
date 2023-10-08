#ifndef __USBIF_H__
#define __USBIF_H__

#include <stdint.h>

#define USBIF_RX_BUFFER_SIZE 2048u

typedef struct UsbifPacketType
{
    uint16_t bytes_received;
    uint16_t last_packet_length;
    uint8_t packet_buf[USBIF_RX_BUFFER_SIZE];
    uint8_t last_packet[USBIF_RX_BUFFER_SIZE];
} UsbifPacketType;

uint8_t usbif_transmit(uint8_t *ptr_data, uint16_t lenght);
uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght);

#endif