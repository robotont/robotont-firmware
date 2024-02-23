/**
 * @file peripheral.h
 * @brief Contains auto-generated CUBEMX code.
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef _PERIPHERAL_H_
#define _PERIPHERAL_H_

#include "stm32f4xx_hal.h"

#define PIN_M0_NSLEEP_Pin        GPIO_PIN_9
#define PIN_M0_NSLEEP_GPIO_Port  GPIOE
#define PIN_M0_EN1_Pin           GPIO_PIN_11
#define PIN_M0_EN1_GPIO_Port     GPIOE
#define PIN_M0_EN2_Pin           GPIO_PIN_10
#define PIN_M0_EN2_GPIO_Port     GPIOE
#define PIN_M0_FAULT_Pin         GPIO_PIN_8
#define PIN_M0_FAULT_GPIO_Port   GPIOE
#define PIN_M0_IPROPI_Pin        GPIO_PIN_7
#define PIN_M0_IPROPI_GPIO_Port  GPIOE

#define PIN_M1_NSLEEP_Pin        GPIO_PIN_9
#define PIN_M1_NSLEEP_GPIO_Port  GPIOB
#define PIN_M1_EN1_Pin           GPIO_PIN_1
#define PIN_M1_EN1_GPIO_Port     GPIOE
#define PIN_M1_EN2_Pin           GPIO_PIN_0
#define PIN_M1_EN2_GPIO_Port     GPIOE
#define PIN_M1_FAULT_Pin         GPIO_PIN_8
#define PIN_M1_FAULT_GPIO_Port   GPIOB
#define PIN_M1_IPROPI_Pin        GPIO_PIN_5
#define PIN_M1_IPROPI_GPIO_Port  GPIOB

#define PIN_M2_NSLEEP_Pin        GPIO_PIN_13
#define PIN_M2_NSLEEP_GPIO_Port  GPIOC
#define PIN_M2_EN1_Pin           GPIO_PIN_15
#define PIN_M2_EN1_GPIO_Port     GPIOC
#define PIN_M2_EN2_Pin           GPIO_PIN_14
#define PIN_M2_EN2_GPIO_Port     GPIOC
#define PIN_M2_FAULT_Pin         GPIO_PIN_6
#define PIN_M2_FAULT_GPIO_Port   GPIOE
#define PIN_M2_IPROPI_Pin        GPIO_PIN_5
#define PIN_M2_IPROPI_GPIO_Port  GPIOE

#define PIN_ROT_ENC_SW_Pin         GPIO_PIN_14
#define PIN_ROT_ENC_SW_GPIO_Port   GPIOD
#define PIN_ROT_ENC_SW_EXTI_IRQn   EXTI15_10_IRQn
#define PIN_ROT_ENC_B_Pin          GPIO_PIN_15
#define PIN_ROT_ENC_B_GPIO_Port    GPIOD
#define PIN_ROT_ENC_B_EXTI_IRQn    EXTI15_10_IRQn
#define PIN_ROT_ENC_A_Pin          GPIO_PIN_6
#define PIN_ROT_ENC_A_GPIO_Port    GPIOC
#define PIN_ESTOP_Pin              GPIO_PIN_2
#define PIN_ESTOP_GPIO_Port        GPIOD

#define PIN_LED_G_Pin            GPIO_PIN_13
#define PIN_LED_G_GPIO_Port      GPIOE
#define PIN_LED_R_Pin            GPIO_PIN_12
#define PIN_LED_R_GPIO_Port      GPIOE
#define PIN_LED_DATA_Pin         GPIO_PIN_14
#define PIN_LED_DATA_GPIO_Port   GPIOE

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM11_Init(void);
void MX_TIM13_Init(void);
void MX_TIM14_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_I2C3_Init(void);
void peripheral_init(void);
void MX_DMA_Init(void);

#endif
