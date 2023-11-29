#include "gpioif.h"

void gpioif_init(void)
{
    // TODO [implementation]
    // since a lot of modules use GPIO, MX_GPIO init called in main.c inside peripheral module
    // init inside each module in the future
}

void gpioif_writePin(GpioPinType *ptr_pin, bool is_active)
{
    HAL_GPIO_WritePin(ptr_pin->ptr_port, ptr_pin->pin_number, is_active);
}

bool gpioif_isActive(GpioPinType *ptr_pin)
{
    return (bool)HAL_GPIO_ReadPin(ptr_pin->ptr_port, ptr_pin->pin_number);
}

void gpioif_togglePin(GpioPinType *ptr_pin)
{
    static bool is_active = false; // todo hal toogle
    HAL_GPIO_WritePin(ptr_pin->ptr_port, ptr_pin->pin_number, is_active);
    is_active = !is_active;
}
