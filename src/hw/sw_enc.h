#ifndef __SW_ENC_H__
#define __SW_ENC_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    GPIO_TypeDef* a_port;
    GPIO_TypeDef* b_port;
    uint16_t a_pin;
    uint16_t b_pin;

    int32_t counter;
    uint8_t direction;
    uint8_t lut_index;
} EncoderType;

void swEncoderInit(EncoderType *henc, GPIO_TypeDef* enca_port, uint16_t enca_pin, 
                                   GPIO_TypeDef* encb_port, uint16_t encb_pin);
void swEncoderInterrupt(EncoderType* henc);
void swEncoderDebug(EncoderType* henc);

#endif