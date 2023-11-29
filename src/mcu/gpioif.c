#include "gpioif.h"

#include "peripheral.h"

void gpioif_init(void)
{
    MX_GPIO_Init();
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
    HAL_GPIO_TogglePin(ptr_pin->ptr_port, ptr_pin->pin_number);
}
