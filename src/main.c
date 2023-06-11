/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "sw_enc.h"
#include "motor.h"
#include <math.h>
#include <stdbool.h>
#include "pid.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Create encoders
sw_enc_t henc0, henc1, henc2;

// Create motors and their configuration datastructures
motor_t hm0, hm1, hm2;
motor_config_t mcfg0, mcfg1, mcfg2;

PID_TypeDef hPID;

//static uint8_t last_packet[APP_RX_DATA_SIZE];        // Buffer for incoming command
//uint8_t packet_buf[APP_RX_DATA_SIZE];        // Buffer for incoming command
//volatile uint8_t serial_arrived = 0;  // Number of bytes arrived
//uint16_t last_packet_length;
volatile uint32_t counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_test(motor_config_t* mcfg);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  swEncoderInit(&henc0, PIN_M0_ENCA_GPIO_Port, PIN_M0_ENCA_Pin, PIN_M0_ENCB_GPIO_Port, PIN_M0_ENCB_Pin);
  swEncoderInit(&henc1, PIN_M1_ENCA_GPIO_Port, PIN_M1_ENCA_Pin, PIN_M1_ENCB_GPIO_Port, PIN_M1_ENCB_Pin);
  swEncoderInit(&henc2, PIN_M2_ENCA_GPIO_Port, PIN_M2_ENCA_Pin, PIN_M2_ENCB_GPIO_Port, PIN_M2_ENCB_Pin);

  
  mcfg0.nsleep_port = PIN_M0_NSLEEP_GPIO_Port;
  mcfg0.en1_port = PIN_M0_EN1_GPIO_Port;
  mcfg0.en2_port = PIN_M0_EN2_GPIO_Port;
  mcfg0.fault_port = PIN_M0_FAULT_GPIO_Port;
  mcfg0.ipropi_port = PIN_M0_IPROPI_GPIO_Port;
  mcfg0.nsleep_pin = PIN_M0_NSLEEP_Pin;
  mcfg0.en1_pin = PIN_M0_EN1_Pin;
  mcfg0.en2_pin = PIN_M0_EN2_Pin;
  mcfg0.fault_pin = PIN_M0_FAULT_Pin;
  mcfg0.ipropi_pin = PIN_M0_IPROPI_Pin;
  mcfg0.pid_k_p = 0.5;
  mcfg0.pid_tau_i = 0;
  mcfg0.pid_tau_d = 0;
  mcfg0.pid_dt = 0.01;
  mcfg0.enc_cpr = 64;
  mcfg0.gear_ratio = 18.75;
  mcfg0.wheel_radius = 0.035;
  mcfg0.wheel_pos_r = 0.145;
  mcfg0.wheel_pos_phi = M_PI / 3.0f;

  mcfg1.nsleep_port = PIN_M1_NSLEEP_GPIO_Port;
  mcfg1.en1_port = PIN_M1_EN1_GPIO_Port;
  mcfg1.en2_port = PIN_M1_EN2_GPIO_Port;
  mcfg1.fault_port = PIN_M1_FAULT_GPIO_Port;
  mcfg1.ipropi_port = PIN_M1_IPROPI_GPIO_Port;
  mcfg1.nsleep_pin = PIN_M1_NSLEEP_Pin;
  mcfg1.en1_pin = PIN_M1_EN1_Pin;
  mcfg1.en2_pin = PIN_M1_EN2_Pin;
  mcfg1.fault_pin = PIN_M1_FAULT_Pin;
  mcfg1.ipropi_pin = PIN_M1_IPROPI_Pin;
  mcfg1.pid_k_p = 0.5;
  mcfg1.pid_tau_i = 0;
  mcfg1.pid_tau_d = 0;
  mcfg1.pid_dt = 0.01;
  mcfg1.enc_cpr = 64;
  mcfg1.gear_ratio = 18.75;
  mcfg1.wheel_radius = 0.035;
  mcfg1.wheel_pos_r = 0.145;
  mcfg1.wheel_pos_phi = M_PI;

  mcfg2.nsleep_port = PIN_M2_NSLEEP_GPIO_Port;
  mcfg2.en1_port = PIN_M2_EN1_GPIO_Port;
  mcfg2.en2_port = PIN_M2_EN2_GPIO_Port;
  mcfg2.fault_port = PIN_M2_FAULT_GPIO_Port;
  mcfg2.ipropi_port = PIN_M2_IPROPI_GPIO_Port;
  mcfg2.nsleep_pin = PIN_M2_NSLEEP_Pin;
  mcfg2.en1_pin = PIN_M2_EN1_Pin;
  mcfg2.en2_pin = PIN_M2_EN2_Pin;
  mcfg2.fault_pin = PIN_M2_FAULT_Pin;
  mcfg2.ipropi_pin = PIN_M2_IPROPI_Pin;
  mcfg2.pid_k_p = 0.5;
  mcfg2.pid_tau_i = 0;
  mcfg2.pid_tau_d = 0;
  mcfg2.pid_dt = 0.01;
  mcfg2.enc_cpr = 64;
  mcfg2.gear_ratio = 18.75;
  mcfg2.wheel_radius = 0.035;
  mcfg2.wheel_pos_r = 0.145;
  mcfg2.wheel_pos_phi = 5.0f / 3.0f * M_PI;

  MotorInit(&hm0, &mcfg0, &henc0);
  MotorInit(&hm1, &mcfg1, &henc1);
  MotorInit(&hm2, &mcfg2, &henc2);

  HAL_TIM_Base_Start_IT(&htim13); // Start timer for PID update
  HAL_TIM_Base_Start_IT(&htim14); // Start timer for reading encoders

  // Start timers for motor PWM generation
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t Text[] = "Hello from Robotont\r\n";
  HAL_Delay(1000);
  
  PID(&hPID, &hm0.linear_velocity, &hm0.effort, &hm0.linear_velocity_setpoint, 0.5, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetOutputLimits(&hPID, -1000, 1000);
  PID_SetSampleTime(&hPID,0.01);

  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
 
    //CDC_Transmit_FS(Text,20);
    //printf("henc0:\t"); swEncoderDebug(&henc0);
    //printf("henc1:\t"); swEncoderDebug(&henc1);
    //printf("henc2:\t"); swEncoderDebug(&henc2);
    //motor_test(&mcfg0);
    //printf("counter: %ld\n", counter);

    if (last_packet_length)
    {
      printf("processing packet (%d): %s\r\n", last_packet_length, (char *) last_packet);
      // Check if command is RS (Robot Speed)
      if (last_packet[0]=='R' && last_packet[1]=='S')
      {
        float lin_vel_x = 0;
        float lin_vel_y = 0;
        float ang_vel_z = 0;

        char * pch;
        pch = strtok ((char *) last_packet, ":");
        int arg=0;
        while (pch != NULL)
        {
          printf ("arg %d: %s (%f)\r\n",arg, pch, atof(pch));
          if (arg==1) {lin_vel_x = atof(pch);}
          else if (arg==2) {lin_vel_y = atof(pch);}
          else if (arg==3) {ang_vel_z = atof(pch);}
          pch = strtok (NULL, ":");
          arg++;
        }
        
        float lin_vel_dir = atan2(lin_vel_y, lin_vel_x);
        float lin_vel_mag = sqrt(lin_vel_x * lin_vel_x + lin_vel_y * lin_vel_y);
        hm0.linear_velocity_setpoint = lin_vel_mag * sin(lin_vel_dir - hm0.cfg->wheel_pos_phi) + hm0.cfg->wheel_pos_r * ang_vel_z;
        hm1.linear_velocity_setpoint = lin_vel_mag * sin(lin_vel_dir - hm1.cfg->wheel_pos_phi) + hm1.cfg->wheel_pos_r * ang_vel_z;
        hm2.linear_velocity_setpoint = lin_vel_mag * sin(lin_vel_dir - hm2.cfg->wheel_pos_phi) + hm2.cfg->wheel_pos_r * ang_vel_z;
      }
      // Check if command is MS (Motor Speed)
      else if(last_packet[0]=='M' && last_packet[1]=='S')
      {
        char * pch;
        pch = strtok ((char *) last_packet, ":");
        int arg=0;
        while (pch != NULL)
        {
          printf ("arg %d: %s (%f)\r\n",arg, pch, atof(pch));
          if (arg==1) {hm0.linear_velocity_setpoint = atof(pch);}
          else if (arg==2) {hm1.linear_velocity_setpoint = atof(pch);}
          else if (arg==3) {hm2.linear_velocity_setpoint = atof(pch);}
          pch = strtok (NULL, ":");
          arg++;
        }
      }
      last_packet[0] = '\0';  // indicate that packet has been processed
      last_packet_length = 0;
    }

    printf("Motor linear velocities: %f %f %f\r\n", hm0.linear_velocity_setpoint, hm1.linear_velocity_setpoint, hm2.linear_velocity_setpoint);
    //printf("last packet len: %d\r\n", last_packet_length);
    //printf("m0 lin vel setpoint: %.3f\r\n", hm0.linear_velocity_setpoint);
    //printf("m0 lin vel: %.3f\r\n", hm0.linear_velocity);
    //printf("m0 lin vel: %.3f\r\n\r\n", hm0.effort);
    
    
    // for (int i = 0; i<500;i+=100)
    // {
    //   // Adjust M0 pwm speed
      
      
    //   HAL_Delay(100); 
    // }
    // Adjust M0 pwm speed
    //TIM3->CCR1 = 300;
    
    HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void motor_test(motor_config_t* mcfg)
{
    HAL_GPIO_WritePin(mcfg->nsleep_port,mcfg->nsleep_pin, SET);
    HAL_GPIO_WritePin(mcfg->en1_port,mcfg->en1_pin, RESET);
    HAL_GPIO_WritePin(mcfg->en2_port,mcfg->en2_pin, SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(mcfg->en1_port,mcfg->en1_pin, RESET);
    HAL_GPIO_WritePin(mcfg->en2_port,mcfg->en2_pin, RESET);
    HAL_Delay(1000);

    HAL_GPIO_WritePin(mcfg->en1_port,mcfg->en1_pin, SET);
    HAL_GPIO_WritePin(mcfg->en2_port,mcfg->en2_pin, RESET);
    HAL_Delay(1000);

    HAL_GPIO_WritePin(mcfg->en1_port,mcfg->en1_pin, SET);
    HAL_GPIO_WritePin(mcfg->en2_port,mcfg->en2_pin, SET);
    HAL_Delay(1000);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
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
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1600-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1600-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */    //hm->effort = 0.1;


  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PIN_M2_NSLEEP_Pin|PIN_M2_EN2_Pin|PIN_M2_EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PIN_M0_NSLEEP_Pin|PIN_M0_EN2_Pin|PIN_M0_EN1_Pin|PIN_LED_Pin
                          |PIN_LED_DATA_Pin|PIN_M1_EN2_Pin|PIN_M1_EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_M1_NSLEEP_GPIO_Port, PIN_M1_NSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIN_POWEROFF_Pin PIN_POWEROFF_REQ_Pin PIN_M2_FAULT_Pin PIN_M0_FAULT_Pin */
  GPIO_InitStruct.Pin = PIN_POWEROFF_Pin|PIN_POWEROFF_REQ_Pin|PIN_M2_FAULT_Pin|PIN_M0_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_CHARGE_SENSE_Pin PIN_M2_IPROPI_Pin PIN_M0_IPROPI_Pin */
  GPIO_InitStruct.Pin = PIN_CHARGE_SENSE_Pin|PIN_M2_IPROPI_Pin|PIN_M0_IPROPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_M2_NSLEEP_Pin PIN_M2_EN2_Pin PIN_M2_EN1_Pin */
  GPIO_InitStruct.Pin = PIN_M2_NSLEEP_Pin|PIN_M2_EN2_Pin|PIN_M2_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_M1_ENCA_Pin PIN_M1_ENCB_Pin PIN_M2_ENCA_Pin PIN_M2_ENCB_Pin */
  GPIO_InitStruct.Pin = PIN_M1_ENCA_Pin|PIN_M1_ENCB_Pin|PIN_M2_ENCA_Pin|PIN_M2_ENCB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_M0_ENCA_Pin PIN_M0_ENCB_Pin */
  GPIO_InitStruct.Pin = PIN_M0_ENCA_Pin|PIN_M0_ENCB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_M0_NSLEEP_Pin PIN_M0_EN2_Pin PIN_M0_EN1_Pin PIN_LED_Pin
                           PIN_LED_DATA_Pin PIN_M1_EN2_Pin PIN_M1_EN1_Pin */
  GPIO_InitStruct.Pin = PIN_M0_NSLEEP_Pin|PIN_M0_EN2_Pin|PIN_M0_EN1_Pin|PIN_LED_Pin
                          |PIN_LED_DATA_Pin|PIN_M1_EN2_Pin|PIN_M1_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_ROT_ENC_SW_Pin PIN_ROT_ENC_B_Pin PIN_ROT_ENC_A_Pin PIN_ESTOP_Pin */
  GPIO_InitStruct.Pin = PIN_ROT_ENC_SW_Pin|PIN_ROT_ENC_B_Pin|PIN_ROT_ENC_A_Pin|PIN_ESTOP_Pin;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the VIRTUAL COM PORT.
  * @param  None
  * @retval None
  */
int _write(int file, char *ptr, int len) {
    static uint8_t rc = USBD_OK;

    // loop commented out for a non-blocking behavior
    //do {
        rc = CDC_Transmit_FS((unsigned char *)ptr, len);
    //} while (USBD_BUSY == rc);
    if (USBD_FAIL == rc) {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM14)
  {
    //HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
    //printf("Encoder0: %ld\n", henc0.counter);
    swEncoderInterrupt(&henc0);
    swEncoderInterrupt(&henc1);
    swEncoderInterrupt(&henc2);
  }
  else if(htim->Instance == TIM3)
  {
    //counter++;
    HAL_GPIO_WritePin(mcfg0.nsleep_port,mcfg0.nsleep_pin, SET);
    HAL_GPIO_WritePin(mcfg0.en1_port,mcfg0.en1_pin, RESET);
    HAL_GPIO_WritePin(mcfg0.en2_port,mcfg0.en2_pin, SET);
  }
  else if(htim->Instance == TIM13)
  {
    // timer for PID
    counter++;
    PID_Compute(&hPID);
    MotorUpdate(&hm0);
  }
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
  {
    counter++;
    HAL_GPIO_WritePin(mcfg0.en2_port,mcfg0.en2_pin, RESET);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
