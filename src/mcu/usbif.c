/**
 * @file usbif.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "usbif.h"
#include <stdbool.h>
#include "usbd_def.h"

// TODO replace with callback to the upper layer (CMD_HANDLER)
uint8_t last_packet[APP_RX_DATA_SIZE];
uint16_t last_packet_length = 0;
// TODO-------------------------------------------------------

ReceiveCallbackType callback_to_the_upper_layer;

/**
 * @brief
 *
 */
void usbif_init(void)
{
    ReceiveCallbackType rx_callback = (ReceiveCallbackType)usbif_receive;
    usbd_cdc_setUpperLayerCallback(rx_callback);
    callback_to_the_upper_layer = NULL;
}

/**
 * @brief
 *
 * @param ptr_data
 * @param lenght
 * @return uint8_t
 */
uint8_t usbif_transmit(uint8_t *ptr_data, uint16_t lenght)
{
    uint8_t rc = USBD_OK;
    uint8_t retval = lenght;

    rc = CDC_Transmit_FS((unsigned char *)ptr_data, lenght);
    if (USBD_FAIL == rc)
    {
        retval = 0;
    }

    return retval;
}

/**
 * @brief
 *
 * @param ptr_data
 * @param lenght
 * @return uint8_t
 *
 * @note Called in ISR context from usb_cdc_if module
 */
uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght)
{
    static uint8_t rx_buffer[APP_RX_DATA_SIZE];
    static uint16_t rx_buffer_length = 0u;
    static bool is_message_complete = false;

    for (uint16_t i = 0u; i < lenght; i++)
    {
        rx_buffer[rx_buffer_length++] = ptr_data[i];
        if (ptr_data[i] == '\r' || ptr_data[i] == '\n')
        {
            memcpy(last_packet, rx_buffer,
                   rx_buffer_length); // TODO replace with callback to the upper layer (CMD_HANDLER)
            last_packet_length = rx_buffer_length;

            is_message_complete = true;
            break;
        }
    }

    if (is_message_complete && callback_to_the_upper_layer != NULL)
    {
        callback_to_the_upper_layer(rx_buffer, rx_buffer_length - 2u); // Exclude CR+LF
        rx_buffer_length = 0u;
    }

    return 0u;
}

/**
 * @brief // TODO to be called from CMD module.
 *
 * @param rx_callback
 */
void usbif_setUpperLayerCallback(ReceiveCallbackType rx_callback)
{
    callback_to_the_upper_layer = rx_callback;
}
