/**
 * @file ioif.h
 * @brief GPIO interface wrapper over CubeMX generated HAL functions
 *
 * Packs togethers `GPIO_PORT` and `GPIO_Pin` into one IoPinType struct.
 * As a result, GPIO operation require keeping track on one variable (pin) instead of two (port and pin).
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef IOIF_H
#define IOIF_H

#include <stdbool.h>
#include <stdint.h>

#include "peripheral.h"

typedef void (*EXTICallbackType)(uint16_t pin_number);

typedef struct
{
    GPIO_TypeDef *ptr_port;
    uint16_t pin_number;
} IoPinType;

void ioif_init(void);
void ioif_writePin(IoPinType *ptr_pin, bool is_active);
bool ioif_isActive(IoPinType *ptr_pin);
void ioif_togglePin(IoPinType *ptr_pin);
void ioif_setRotaryEncoderCallback(EXTICallbackType callback);

#endif