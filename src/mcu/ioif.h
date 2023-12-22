#ifndef IOIF_H
#define IOIF_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef *ptr_port;
    uint16_t pin_number;
} IoPinType;

void ioif_init(void);
void ioif_writePin(IoPinType *ptr_pin, bool is_active);
bool ioif_isActive(IoPinType *ptr_pin);
void ioif_togglePin(IoPinType *ptr_pin);

#endif