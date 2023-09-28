/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define PIN_POWEROFF_Pin           GPIO_PIN_2
#define PIN_POWEROFF_GPIO_Port     GPIOE
#define PIN_POWEROFF_REQ_Pin       GPIO_PIN_3
#define PIN_POWEROFF_REQ_GPIO_Port GPIOE
#define PIN_CHARGE_SENSE_Pin       GPIO_PIN_4
#define PIN_CHARGE_SENSE_GPIO_Port GPIOE
#define PIN_M2_IPROPI_Pin          GPIO_PIN_5
#define PIN_M2_IPROPI_GPIO_Port    GPIOE
#define PIN_M2_FAULT_Pin           GPIO_PIN_6
#define PIN_M2_FAULT_GPIO_Port     GPIOE
#define PIN_M2_NSLEEP_Pin          GPIO_PIN_13
#define PIN_M2_NSLEEP_GPIO_Port    GPIOC
#define PIN_M2_EN2_Pin             GPIO_PIN_14
#define PIN_M2_EN2_GPIO_Port       GPIOC
#define PIN_M2_EN1_Pin             GPIO_PIN_15
#define PIN_M2_EN1_GPIO_Port       GPIOC
#define PIN_M1_ENCA_Pin            GPIO_PIN_0
#define PIN_M1_ENCA_GPIO_Port      GPIOC
#define PIN_M1_ENCB_Pin            GPIO_PIN_1
#define PIN_M1_ENCB_GPIO_Port      GPIOC
#define PIN_M2_ENCA_Pin            GPIO_PIN_2
#define PIN_M2_ENCA_GPIO_Port      GPIOC
#define PIN_M2_ENCB_Pin            GPIO_PIN_3
#define PIN_M2_ENCB_GPIO_Port      GPIOC
#define PIN_M0_ENCA_Pin            GPIO_PIN_2
#define PIN_M0_ENCA_GPIO_Port      GPIOA
#define PIN_M0_ENCB_Pin            GPIO_PIN_3
#define PIN_M0_ENCB_GPIO_Port      GPIOA
#define PIN_M0_IPROPI_Pin          GPIO_PIN_7
#define PIN_M0_IPROPI_GPIO_Port    GPIOE
#define PIN_M0_FAULT_Pin           GPIO_PIN_8
#define PIN_M0_FAULT_GPIO_Port     GPIOE
#define PIN_M0_NSLEEP_Pin          GPIO_PIN_9
#define PIN_M0_NSLEEP_GPIO_Port    GPIOE
#define PIN_M0_EN2_Pin             GPIO_PIN_10
#define PIN_M0_EN2_GPIO_Port       GPIOE
#define PIN_M0_EN1_Pin             GPIO_PIN_11
#define PIN_M0_EN1_GPIO_Port       GPIOE
#define PIN_LED_Pin                GPIO_PIN_13
#define PIN_LED_GPIO_Port          GPIOE
#define PIN_LED_DATA_Pin           GPIO_PIN_14
#define PIN_LED_DATA_GPIO_Port     GPIOE
#define PIN_ROT_ENC_SW_Pin         GPIO_PIN_13
#define PIN_ROT_ENC_SW_GPIO_Port   GPIOD
#define PIN_ROT_ENC_B_Pin          GPIO_PIN_14
#define PIN_ROT_ENC_B_GPIO_Port    GPIOD
#define PIN_ROT_ENC_A_Pin          GPIO_PIN_15
#define PIN_ROT_ENC_A_GPIO_Port    GPIOD
#define PIN_ESTOP_Pin              GPIO_PIN_2
#define PIN_ESTOP_GPIO_Port        GPIOD
#define PIN_M1_IPROPI_Pin          GPIO_PIN_5
#define PIN_M1_IPROPI_GPIO_Port    GPIOB
#define PIN_M1_FAULT_Pin           GPIO_PIN_8
#define PIN_M1_FAULT_GPIO_Port     GPIOB
#define PIN_M1_NSLEEP_Pin          GPIO_PIN_9
#define PIN_M1_NSLEEP_GPIO_Port    GPIOB
#define PIN_M1_EN2_Pin             GPIO_PIN_0
#define PIN_M1_EN2_GPIO_Port       GPIOE
#define PIN_M1_EN1_Pin             GPIO_PIN_1
#define PIN_M1_EN1_GPIO_Port       GPIOE

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
