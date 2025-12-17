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
#define I2CIF_RX_BUFFER_SIZE 25
//#define I2CIF_RX_BUFFER_SIZE 29

typedef void (*I2CCallbackType)(I2C_HandleTypeDef *i2c_handler); /* Callback that is called when error occurs */
typedef void (*I2CRxCallbackType)(I2C_HandleTypeDef *i2c_handler, uint8_t *ptr_data); /* Callback that called when data is received */

void i2cif_init(void);
void i2cif_setErrorCallback(I2CCallbackType callback);
void i2cif_setReceiveCallback(I2CRxCallbackType callback);
void i2cif_memoryWrite(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint16_t mem_addr,
                                    uint16_t mem_size, uint8_t *ptr_data, uint16_t data_size, uint32_t timeout);

void i2cif_masterRead(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint8_t *ptr_data, uint16_t data_size, uint32_t timeout_ms);