/**
 * @file usbif.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "usbif.h"
#include <stdbool.h>
#include "usb_device.h"
#include "usbd_def.h"

static ReceiveCallbackType priv_callback;

/**
 * @brief
 *
 */
void usbif_init(void)
{
    priv_callback = NULL;
    MX_USB_DEVICE_Init();
    usbd_cdc_setUpperLayerCallback((ReceiveCallbackType)usbif_receive);
}

/**
 * @brief
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
 * @param ptr_data
 * @param lenght
 * @return uint8_t
 * @note Called in ISR context from usb_cdc_if module
 */
uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght)
{
    static uint8_t rx_buffer[USBIF_BUFFER_SIZE];
    static uint16_t rx_buffer_length = 0u;
    static bool is_message_complete = false;

    for (uint16_t i = 0u; i < lenght; i++)
    {
        rx_buffer[rx_buffer_length++] = ptr_data[i];
        if (ptr_data[i] == '\r' || ptr_data[i] == '\n')
        {
            is_message_complete = true;
        }
    }

    if (is_message_complete && priv_callback != NULL)
    {
        priv_callback(rx_buffer, rx_buffer_length - 2u); // Exclude CR+LF
        rx_buffer_length = 0u;
    }

    return 0u;
}

/**
 * @brief
 * @param rx_callback
 */
void usbif_setUpperLayerCallback(ReceiveCallbackType rx_callback)
{
    priv_callback = rx_callback;
}
