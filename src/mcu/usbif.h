/**
 * @file usbif.h
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
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

void usbif_init(void);
uint8_t usbif_transmit(uint8_t *ptr_data, uint16_t lenght);
uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght);
void usbif_setUpperLayerCallback(ReceiveCallbackType rx_callback);

#endif