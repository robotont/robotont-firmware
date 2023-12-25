#include "ioif.h"

#include "peripheral.h"

void ioif_init(void)
{
    static bool is_initialized = false;
    if (!is_initialized)
    {
        MX_GPIO_Init();
        is_initialized = true;
    }
}

void ioif_writePin(IoPinType *ptr_pin, bool is_active)
{
    HAL_GPIO_WritePin(ptr_pin->ptr_port, ptr_pin->pin_number, is_active);
}

bool ioif_isActive(IoPinType *ptr_pin)
{
    return (bool)HAL_GPIO_ReadPin(ptr_pin->ptr_port, ptr_pin->pin_number);
}

void ioif_togglePin(IoPinType *ptr_pin)
{
    HAL_GPIO_TogglePin(ptr_pin->ptr_port, ptr_pin->pin_number);
}
