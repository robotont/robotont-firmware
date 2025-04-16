/**
 * @file menu.h
 * @brief Service. Displays information and allows for changing various parameters on the display using rotary encoder as input
 * 
 * @author Andres Sakk (andres.sakk@ut.ee)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */
#ifndef MENU_H
#define MENU_H

#include "ssd1306.h"

#define MAX_CONTAINERS 10
#define MAX_CONTAINER_NAME_LEN 32

void menu_init();
void menu_update();
void menu_updateContainers(char names[][MAX_CONTAINER_NAME_LEN], uint8_t count);

#endif