/**
 * @file i2cif.h
 * @brief I2C interface wrapper over CubeMX generated HAL functions
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2024 Tartu Ülikool
 */

#include "peripheral.h"

#define I2C_HANDLER_1            (&hi2c1)
#define I2C_HANDLER_2            (&hi2c2)
#define I2C_HANDLER_3            (&hi2c3)

#define I2CIF_DEFAULT_TIMEOUT_MS HAL_MAX_DELAY

typedef void (*I2CCallbackType)(I2C_HandleTypeDef *i2c_handler); /* Callback, that called, when error occurs */

void i2cif_init(void);
void i2cif_memoryWrite(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint16_t mem_addr,
                                    uint16_t mem_size, uint8_t *ptr_data, uint16_t data_size, uint32_t timeout);

void i2cif_masterRead(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint8_t *ptr_data, uint16_t data_size, uint32_t timeout_ms);