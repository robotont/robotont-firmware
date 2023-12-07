// Implementation adapted from
// https://electronics.stackexchange.com/questions/360637/quadrature-encoder-most-efficient-software-implementation

#include "sw_enc.h"

#include <stdint.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "gpioif.h"

const int32_t counter_lut[32] = {
    // Direction = 1
    0,  // 00 to 00
    -1, // 00 to 01
    +1, // 00 to 10
    +2, // 00 to 11

    +1, // 01 to 00
    0,  // 01 to 01
    +2, // 01 to 10
    -1, // 01 to 11

    -1, // 10 to 00
    +2, // 10 to 01
    0,  // 10 to 10
    +1, // 10 to 11

    +2, // 11 to 00
    +1, // 11 to 01
    -1, // 11 to 10
    0,  // 11 to 11

    // Direction = 0
    0,  // 00 to 00
    -1, // 00 to 01
    +1, // 00 to 10
    -2, // 00 to 11

    +1, // 01 to 00
    0,  // 01 to 01
    -2, // 01 to 10
    -1, // 01 to 11

    -1, // 10 to 00
    -2, // 10 to 01
    0,  // 10 to 10
    +1, // 10 to 11

    -2, // 11 to 00
    +1, // 11 to 01
    -1, // 11 to 10
    0,  // 11 to 11
};

void sw_enc_init(EncoderType *ptr_enc, GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin)
{
    gpioif_init();
    
    ptr_enc->a_port = a_port;
    ptr_enc->a_pin = a_pin;
    ptr_enc->b_port = b_port;
    ptr_enc->b_pin = b_pin;

    ptr_enc->counter = 0;
    ptr_enc->direction = 0;
    ptr_enc->lut_index = 0;
}

void sw_enc_interrupt(EncoderType *ptr_enc)
{
    ptr_enc->lut_index |=
        HAL_GPIO_ReadPin(ptr_enc->a_port, ptr_enc->a_pin) << 1 | HAL_GPIO_ReadPin(ptr_enc->b_port, ptr_enc->b_pin);
    ptr_enc->counter += counter_lut[ptr_enc->lut_index];

    if (counter_lut[ptr_enc->lut_index] != 0)
    {
        ptr_enc->direction = (counter_lut[ptr_enc->lut_index] > 0) ? 1 : 0;
    }

    // Prepare for next iteration by shifting current state
    // bits to old state bits and also the direction bit
    ptr_enc->lut_index = (((ptr_enc->lut_index) << 2) & 0b1100) | ((ptr_enc->direction) << 4);
}

void sw_enc_debug(EncoderType *ptr_enc)
{
    printf("ENCA: %d\t", HAL_GPIO_ReadPin(ptr_enc->a_port, ptr_enc->a_pin));
    printf("ENCB: %d\t", HAL_GPIO_ReadPin(ptr_enc->b_port, ptr_enc->b_pin));
    printf("CNT: %ld\r\n", ptr_enc->counter);
}