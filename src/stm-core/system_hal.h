/**
 * @file system_hal_init.h
 * @brief Contains auto-generated CUBEMX code
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef _SYSCLK_H_
#define _SYSCLK_H_

#include <stdint.h>

#define USE_FULL_ASSERT

void system_hal_init(void);
void system_hal_delay(uint32_t delay_ms);
uint32_t system_hal_timestamp(void);
void Error_Handler(void);

#endif