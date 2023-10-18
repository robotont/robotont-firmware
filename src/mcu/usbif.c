/**
 * @file usbif.c
 * @brief 
 * 
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */
#include "usbif.h"
#include "usbd_def.h"

// TODO replace with callback to the upper layer (CMD_HANDLER)
uint16_t bytes_received = 0;
uint16_t last_packet_length = 0;
uint8_t packet_buf[APP_RX_DATA_SIZE];
uint8_t last_packet[APP_RX_DATA_SIZE];
// -----------------------------------------------------------

ReceiveCallbackType callback_to_the_upper_layer;

/**
 * @brief
 *
 */
void usbif_init(void)
{
    ReceiveCallbackType rx_callback = (ReceiveCallbackType)usbif_receive;
    usbd_cdc_setUpperLayerCallback(rx_callback);
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
    static uint8_t rc = USBD_OK;
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
    // walk through the buffer and check for command termination
    for (uint16_t i = 0; i < lenght; i++)
    {
        if (ptr_data[i] == '\r' || ptr_data[i] == '\n') // Packet complete
        {
            // complete
            // memset(last_packet, '\0',APP_RX_DATA_SIZE);
            last_packet_length = bytes_received + i + 1;
            if (last_packet_length > 3)
            {
                memcpy(last_packet, packet_buf, last_packet_length);
            }
            else
            {
                memset(last_packet, '\0', APP_RX_DATA_SIZE);
                last_packet_length = 0;
            }

            // TODO check, if UserRxBufferFS need to be cleared
            // UserRxBufferFS[0] = '\0';
            // memset(UserRxBufferFS, '\0', APP_RX_DATA_SIZE);
            bytes_received = 0;
            break;
        }
        else
        {
            packet_buf[bytes_received++] = ptr_data[i];
            packet_buf[bytes_received] = '\0';
        }
    }

    // TODO // parse arguments in the CMD module, cut off argument and
    // TODO // send data to the sub-modules (motor control, led, oled etc)
    // if (callback_to_the_upper_layer != NULL)
    // {
    //     callback_to_the_upper_layer(ptr_data, lenght);
    // }
    return 0;
}
