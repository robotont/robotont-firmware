#include "usbif.h"
#include "usbd_def.h"

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

uint8_t usbif_receive(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO callback to upper layer
}
