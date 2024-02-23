#ifndef MENU_H
#define MENU_H

#include "ioif.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

#define PIN_ENC_SW GPIO_PIN_14
#define PIN_ENC_B GPIO_PIN_15
#define PIN_ENC_A GPIO_PIN_6

void menu_init(void);
void menu_processInput(uint16_t pin_number);
void menu_update();

#endif