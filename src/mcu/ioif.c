/**
 * @file ioif.c
 * @brief GPIO interface wrapper over CubeMX generated MX_USB HAL
 *
 * Packs togethers `GPIO_PORT` and `GPIO_Pin` into one IoPinType struct.
 * As a result, GPIO operation require keeping track on one variable (pin) instead of two (port and pin).
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "ioif.h"

#include "peripheral.h"

/**
 * @brief Initializes timer module
 */
void ioif_init(void)
{
    static bool is_initialized = false;
    if (!is_initialized)
    {
        MX_GPIO_Init();
        is_initialized = true;
    }
}

/**
 * @brief Set pin value to the LOW or HIGH
 */
void ioif_writePin(IoPinType *ptr_pin, bool is_active)
{
    HAL_GPIO_WritePin(ptr_pin->ptr_port, ptr_pin->pin_number, is_active);
}

/**
 * @brief Read pin value (HIGH = true, LOW = false)
 */
bool ioif_isActive(IoPinType *ptr_pin)
{
    return (bool)HAL_GPIO_ReadPin(ptr_pin->ptr_port, ptr_pin->pin_number);
}

/**
 * @brief Revert pin state
 */
void ioif_togglePin(IoPinType *ptr_pin)
{
    HAL_GPIO_TogglePin(ptr_pin->ptr_port, ptr_pin->pin_number);
}
