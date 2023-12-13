/**
 * @file peripheral.c
 * @brief Contains auto-generated CUBEMX code. // TODO Create interface for each perepheral
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "peripheral.h"
#include "system_hal.h"

CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

void peripheral_init(void)
{
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM11_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART3_UART_Init();
}

// clang-format off

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, PIN_M2_NSLEEP_Pin | PIN_M2_EN2_Pin | PIN_M2_EN1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE,
                      PIN_M0_NSLEEP_Pin | PIN_M0_EN2_Pin | PIN_M0_EN1_Pin | PIN_LED_Pin | PIN_LED_DATA_Pin |
                          PIN_M1_EN2_Pin | PIN_M1_EN1_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PIN_M1_NSLEEP_GPIO_Port, PIN_M1_NSLEEP_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : PIN_POWEROFF_Pin PIN_POWEROFF_REQ_Pin PIN_M2_FAULT_Pin PIN_M0_FAULT_Pin */
    GPIO_InitStruct.Pin = PIN_POWEROFF_Pin | PIN_POWEROFF_REQ_Pin | PIN_M2_FAULT_Pin | PIN_M0_FAULT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_CHARGE_SENSE_Pin PIN_M2_IPROPI_Pin PIN_M0_IPROPI_Pin */
    GPIO_InitStruct.Pin = PIN_CHARGE_SENSE_Pin | PIN_M2_IPROPI_Pin | PIN_M0_IPROPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_M2_NSLEEP_Pin PIN_M2_EN2_Pin PIN_M2_EN1_Pin */
    GPIO_InitStruct.Pin = PIN_M2_NSLEEP_Pin | PIN_M2_EN2_Pin | PIN_M2_EN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_M1_ENCA_Pin PIN_M1_ENCB_Pin PIN_M2_ENCA_Pin PIN_M2_ENCB_Pin */
    GPIO_InitStruct.Pin = PIN_M1_ENCA_Pin | PIN_M1_ENCB_Pin | PIN_M2_ENCA_Pin | PIN_M2_ENCB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_M0_ENCA_Pin PIN_M0_ENCB_Pin */
    GPIO_InitStruct.Pin = PIN_M0_ENCA_Pin | PIN_M0_ENCB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_M0_NSLEEP_Pin PIN_M0_EN2_Pin PIN_M0_EN1_Pin PIN_LED_Pin
                             PIN_LED_DATA_Pin PIN_M1_EN2_Pin PIN_M1_EN1_Pin */
    GPIO_InitStruct.Pin = PIN_M0_NSLEEP_Pin | PIN_M0_EN2_Pin | PIN_M0_EN1_Pin | PIN_LED_Pin | PIN_LED_DATA_Pin |
                          PIN_M1_EN2_Pin | PIN_M1_EN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PIN_ROT_ENC_SW_Pin PIN_ROT_ENC_B_Pin PIN_ROT_ENC_A_Pin PIN_ESTOP_Pin */
    GPIO_InitStruct.Pin = PIN_ROT_ENC_SW_Pin | PIN_ROT_ENC_B_Pin | PIN_ROT_ENC_A_Pin | PIN_ESTOP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PIN_M1_IPROPI_Pin */
    GPIO_InitStruct.Pin = PIN_M1_IPROPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PIN_M1_IPROPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PIN_M1_FAULT_Pin */
    GPIO_InitStruct.Pin = PIN_M1_FAULT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PIN_M1_FAULT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PIN_M1_NSLEEP_Pin */
    GPIO_InitStruct.Pin = PIN_M1_NSLEEP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_M1_NSLEEP_GPIO_Port, &GPIO_InitStruct);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA interrupt init */
    /* DMA2_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 89;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
    Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
    Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
    Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
    Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
    Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function (for motors PWM generation)
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 80 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1; // 200 Hz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = { 0 };
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 80 - 1;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 2*1000 - 1;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM13 Initialization Function (for calling PID_Compute)
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = { 0 };
    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 80 - 1;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = 1000 - 1; // 1 kHz
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM14 Initialization Function (used for encoders)
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void)
{
    htim14.Instance = TIM14;
    //  htim14.Init.Prescaler = 1600-1;
    htim14.Init.Prescaler = 400 - 1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 2 - 1; // 10 kHz
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}