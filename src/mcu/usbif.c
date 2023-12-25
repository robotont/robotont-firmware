/**
 * @file usbif.c
 * @brief USB interface wrapper over CubeMX generated MX_USB HAL
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "usbif.h"

#include <stdbool.h>

#include "usb_device.h"
#include "usbd_def.h"

static ReceiveCallbackType receive_callback;

/**
 * @brief Initializes USB module
 */
void usbif_init(void)
{
    static bool is_initialized = false;
    if (!is_initialized)
    {
        receive_callback = NULL;
        MX_USB_DEVICE_Init();
        usbd_cdc_setUpperLayerCallback((ReceiveCallbackType)usbif_receive);

        is_initialized = true;
    }
}

/**
 * @brief Transmits data via USB using CDC_Transmit_FS
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
 * @brief   Handles "USB receive data" event
 * @details
 * If termination chars received (CR+LF), then packet marked as complete and data sent to the upper layer
 * Othervise, chars stored in the buffer.
 * @note    Called within ISR context from usb_cdc_if module
 * @note    Buffer size (i.e. maximum allowed packet lenght) is 2048 bytes
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

    if (is_message_complete && receive_callback != NULL)
    {
        receive_callback(rx_buffer, rx_buffer_length - 2u); // Exclude CR+LF
        rx_buffer_length = 0u;
    }

    return 0u;
}

/**
 * @brief Sets funtions, that is called in interrupt, when USB data received
 */
void usbif_setUpperLayerCallback(ReceiveCallbackType rx_callback)
{
    receive_callback = rx_callback;
}
