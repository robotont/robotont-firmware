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

uint8_t usbif_transmit_blocking(uint8_t *ptr_data, uint16_t length)
{
    uint32_t timeout = system_hal_timestamp() + 10;  // 10ms timeout
    uint8_t rc;
    
    do {
        rc = CDC_Transmit_FS((unsigned char *)ptr_data, length);
        if (rc == USBD_OK) return length;
        if (rc == USBD_FAIL) return 0;
        // USBD_BUSY - keep trying
    } while (system_hal_timestamp() < timeout);
    
    return 0;  // Timeout
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

    for (uint16_t i = 0u; i < lenght; i++)
    {
        // Bounds check
        if (rx_buffer_length >= USBIF_BUFFER_SIZE)
        {
            // Buffer full - reset and discard
            rx_buffer_length = 0u;
            continue;
        }

        uint8_t c = ptr_data[i];
        rx_buffer[rx_buffer_length++] = c;

        // Check for end of message
        if (c == '\r' || c == '\n')
        {
            if (receive_callback != NULL && rx_buffer_length > 1)
            {
                // Safe length calculation
                uint16_t msg_len = rx_buffer_length;
                // Strip trailing CR/LF
                while (msg_len > 0 && (rx_buffer[msg_len - 1] == '\r' || rx_buffer[msg_len - 1] == '\n'))
                {
                    msg_len--;
                }
                
                if (msg_len > 0)
                {
                    receive_callback(rx_buffer, msg_len);
                }
            }
            // Reset buffer after processing
            rx_buffer_length = 0u;
        }
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
