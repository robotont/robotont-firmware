#ifndef GPIOIF_H
#define GPIOIF_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef *ptr_port;
    uint16_t pin_number;
} GpioPinType;

void gpioif_init(void);
void gpioif_writePin(GpioPinType *ptr_pin, bool is_active);
bool gpioif_isActive(GpioPinType *ptr_pin);
void gpioif_togglePin(GpioPinType *ptr_pin);

#endif