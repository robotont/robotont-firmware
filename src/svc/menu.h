/**
 * @file menu.h
 * @brief Service. Displays information and allows for changing various parameters on the display using rotary encoder as input
 *
 * 
 * 
 * @author Andres Sakk (andres.sakk@ut.ee)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */
#ifndef MENU_H
#define MENU_H

#include "ssd1306.h"

void menu_init();
void menu_update();

#endif