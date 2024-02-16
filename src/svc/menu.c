#include "menu.h"

#include <stdint.h>
#include <stdio.h>

void menu_init(void)
{
    EXTICallbackType my_callback = (EXTICallbackType)menu_blahFunction;
    ioif_setRotaryEncoderCallback(my_callback);
}

void menu_blahFunction(uint16_t pin_number)
{
    printf("Wow! You just got interrupt from pin %d", pin_number);
    // Do something smart with this event
}
