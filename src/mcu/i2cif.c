/**
 * @file i2cif.c
 * @brief I2C interface wrapper over CubeMX generated HAL functions
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2024 Tartu Ülikool
 */

#include "i2cif.h"

#include <stdbool.h>

static I2CCallbackType error_callback;

/**
 * @brief Initializes i2c module
 */
void i2cif_init(void)
{
    static bool is_initialized = false;
    if (!is_initialized)
    {
        error_callback = NULL;
        MX_I2C1_Init();
        MX_I2C2_Init();
        MX_I2C3_Init();
        is_initialized = true;
    }
}

/**
 * @brief Sets function, that is called inside `HAL_I2C_ErrorCallback` interrupt
 */
void i2cif_setErrorCallback(I2CCallbackType callback)
{
    error_callback = callback;
}

/**
 * @brief Transmits data over I2C to the specific device memory address
 * @note  Master mode
 */
void i2cif_memoryWrite(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint16_t mem_addr, uint16_t mem_size,
                       uint8_t *ptr_data, uint16_t data_size, uint32_t timeout_ms)
{
    (void)HAL_I2C_Mem_Write(i2c_handler, slave_addr, mem_addr, mem_size, ptr_data, data_size, timeout_ms);
}

/*======================================================================================================================
 * I2C ISR handlers. Naming comes from CUBE HAL
======================================================================================================================*/

/**
 * @brief Interrupt, that called, when error occurs on the I2C bus
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c_handler)
{
    if (error_callback != NULL)
    {
        error_callback(i2c_handler);
    }
}


