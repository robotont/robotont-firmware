// Implementation adapted from
// https://electronics.stackexchange.com/questions/360637/quadrature-encoder-most-efficient-software-implementation

#include "sw_enc.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

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

void swEncoderInit(sw_enc_t * henc, GPIO_TypeDef * a_port, uint16_t a_pin, GPIO_TypeDef * b_port, uint16_t b_pin)
{
    henc->a_port = a_port;
    henc->a_pin = a_pin;
    henc->b_port = b_port;
    henc->b_pin = b_pin;

    henc->counter = 0;
    henc->direction = 0;
    henc->lut_index = 0;
}

void swEncoderInterrupt(sw_enc_t * henc)
{
    henc->lut_index |= HAL_GPIO_ReadPin(henc->a_port, henc->a_pin) << 1 | HAL_GPIO_ReadPin(henc->b_port, henc->b_pin);
    henc->counter += counter_lut[henc->lut_index];

    if (counter_lut[henc->lut_index] != 0)
    {
        henc->direction = (counter_lut[henc->lut_index] > 0) ? 1 : 0;
    }

    // Prepare for next iteration by
    // shifting current state bits to
    // old state bits and also the
    // direction bit
    henc->lut_index = (((henc->lut_index) << 2) & 0b1100) | ((henc->direction) << 4);
}

void swEncoderDebug(sw_enc_t * henc)
{
    printf("ENCA: %d\t", HAL_GPIO_ReadPin(henc->a_port, henc->a_pin));
    printf("ENCB: %d\t", HAL_GPIO_ReadPin(henc->b_port, henc->b_pin));
    printf("CNT: %ld\r\n", henc->counter);
}