#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include <sstream>
#include <vector>
#include "WS2812.h"
#include "PixelArray.h"

// Common parameters for all motors
#define ENC_CPR 64
#define GEAR_RATIO 18.75
#define WHEEL_RADIUS 0.035
#define WHEEL_POS_R 0.145
#define PID_KP 0.8
#define PID_TI 0.05
#define PID_TD 0.0
#define PID_DELTA_T 0.01
#define MAIN_DELTA_T 0.02

#define MAX_CMD_ARGS 100
#define SERIAL_BUF_SIZE 2048
#define MOTOR_COUNT 3
#define CMD_TIMEOUT_MS 1000  // If velocity command is not received within this period all motors are stopped.


// DMA stuff
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DigitalOut myled(LED2);
uint16_t loendur=0;
volatile uint8_t toggle = 0;
char serial_buf[1000];
// Include motor configurations
//#include "motor_config_v0_6.h"
#include "motor_config_v2_1.h"
uint8_t	rx_byte;

// Initialize motors
Motor m[] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Initialize odometry
Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);

// Timeout
Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;

// Variables for serial connection


volatile uint16_t serial_arrived = 0;  // Number of bytes arrived
volatile bool packet_received_b = false;
uint8_t b[1];

// For parsing command with arguments received over serial
std::vector<std::string> cmd;

// LED STRIP
// Set the number of pixels of the strip
#define WS2812_BUF 60

PixelArray px(WS2812_BUF);
WS2812 ws1(PA_15, WS2812_BUF, 1, 12, 6, 11);
//dma init ja muud func
extern "C"  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  
   loendur +=  10;
   HAL_UART_Receive_IT(&huart2, b, 1);

}

extern "C"  void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

extern "C" void DMA1_Channel7_IRQHandler(void)
{


  
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  
  static int8_t endflag = 0;
  if(endflag != 0){//2回目のコール
      huart2.gState=HAL_UART_STATE_READY;
      endflag = 0;
  }else{
      endflag++;
  }


}



extern "C"  void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  


}
extern "C"  void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

extern "C"  void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}


extern "C" void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);


}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/*
HAL_StatusTypeDef printUart(char* buf,int num){
    HAL_StatusTypeDef status=HAL_ERROR;
    if(hdma_usart2_tx.Instance->CNDTR!=0){
        return HAL_BUSY;
    }
    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txbuffer, num);

    return status;
}
*/
extern "C" void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  loendur+=10;
  myled = !myled ;
  if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		// RXNE flags automatically clears when reading RDR.

		// Store incoming byte
		rx_byte = USART2->RDR;
	
	}
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */

  // Store bytes from serial in our buffer until packet
  // termination byte 'enter', '\n', '\r' etc has arrived
  /*
  
  myled = 0;
  
  HAL_UART_Receive_IT(&huart2, &b, 1); 
  char c = (char) b[0];
  
  serial_buf[serial_arrived++] = c;
  serial_buf[serial_arrived] = '\0';
  if (serial_arrived >= SERIAL_BUF_SIZE - 1)
  {
    serial_arrived = 0;
  }

  if (c == '\n' || c == '\r')  // command terminated
  {
    if (serial_arrived > 3)
    {
      // signal that the packet is complete for processing
      packet_received_b = true;
    }
  }

  // if escape is received, clear the buffer and stop the motors for now
  if (c == 27)  // esc
  {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      m[i].stop();
    }
    serial_buf[0] = '\0';
    serial_arrived = 0;
  }
  
  */
}



// This method processes a received serial packet
void processPacket(const std::string& packet)
{
  std::istringstream ss(packet);
  std::string arg;
  cmd.clear();

  for (int i = 0; i <= MAX_CMD_ARGS; i++)
  {
    arg.clear();
    std::getline(ss, arg, ':');
    if (arg.length())
    {
      cmd.push_back(arg);
      // serial_pc.printf("Got arg %s\r\n", arg.c_str());
    }
    else
    {
      break;
    }
  }

  if (!cmd.size())
  {
    return;
  }

  // MS - Set motor speeds manually (linear speed on wheel m/s)
  /* MS:motor1_speed:motor2_speed:motor3_speed */
  if (cmd[0] == "MS")
  {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      float speed_setpoint = std::atof(cmd[i + 1].c_str());
      // serial_pc.printf("Setpoint %d, %f\r\n", i, speed_setpoint);
      m[i].setSpeedSetPoint(speed_setpoint);
    }
    cmd_timer.reset();
  }

  // RS - Set motor speeds based on robot velocities. We use ROS coordinate convention: x-forward,
  // y-left, theta-CCW rotation.
  /* RS:robot_speed_x(m/s):robot_speed_y(m/s):robot_speed_theta(rad/s) */
  else if (cmd[0] == "RS")
  {
    float lin_speed_x = std::atof(cmd[1].c_str());
    float lin_speed_y = std::atof(cmd[2].c_str());
    float angular_speed_z = std::atof(cmd[3].c_str());

    float lin_speed_dir = atan2(lin_speed_y, lin_speed_x);
    float lin_speed_mag = sqrt(lin_speed_x * lin_speed_x + lin_speed_y * lin_speed_y);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      float speed = lin_speed_mag * sin(lin_speed_dir - m[i].getWheelPosPhi()) + m[i].getWheelPosR() * angular_speed_z;
      if (abs(speed) < 1e-5)
      {
        m[i].stop();
      }
      else
      {
        m[i].setSpeedSetPoint(speed);
      }
    }
    cmd_timer.reset();
  }
  else if (cmd[0] == "PID")  // Update PID parameters
  {
    float k_p = 0.0f;
    float tau_i = 0.0f;
    float tau_d = 0.0f;
    // sscanf(ss.str().c_str(), "%f:%f:%f", &k_p, &tau_i, &tau_d);
    // for (uint8_t i = 0; i < 3; i++)
    //{
    //  m[i].setPIDTunings(k_p, tau_i, tau_d);
    //}
  }
  else if (cmd[0] == "LED")  // Update LED states for a segment.
  {
    ws1.useII(WS2812::GLOBAL);
    ws1.setII(0xFF);                            // Set intensity to use the full range
    uint8_t led_index = std::strtoul(cmd[1].c_str(), NULL, 10);  // led index. Value 0 = First led.
    // Color represented by 3 bytes: 0xFF0000 - red, 0x00FF00 - green, 0x0000FF blue (color).
    for (uint8_t i = 2; i < cmd.size(); i++)
    {
      uint32_t value = std::strtoul(cmd[i].c_str(), NULL, 10);
      px.Set(led_index, value);
      led_index++;
    }
    ws1.write(px.getBuf());
  }
}

// Process an incoming serial byte





void check_for_timeout()
{
  if ((cmd_timer.read_ms()) > CMD_TIMEOUT_MS)
  {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      m[i].stop();
    }
  }
}

int main()
{
  // Initialize serial connection
 /*
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN ****\r\n");
*/
  cmd_timeout_checker.attach(check_for_timeout, 0.1);
  cmd_timer.start();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  HAL_StatusTypeDef status;
  int retry;
  
  myled = 1;
  char txbuffer[100];
  static char txbuffer2[]="tuli minigi jama \n\r";
  // MAIN LOOP
  HAL_UART_Receive_IT(&huart2, b, 1);
  while (true)
  {


    HAL_StatusTypeDef status=HAL_ERROR;
    if(hdma_usart2_tx.Instance->CNDTR!=0){
        return HAL_BUSY;
    }
    snprintf(txbuffer, sizeof(txbuffer), "S: %d,%hu \n", loendur,rx_byte);
    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txbuffer, strlen(txbuffer));

    wait_us(1000000);
    if (packet_received_b)  // packet was completeted with \r \n
    {
      HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txbuffer2, strlen(txbuffer2));
      std::string packet(serial_buf);
      serial_buf[0] = '\0';
      serial_arrived = 0;
      //processPacket(packet);
      
      packet_received_b = false;
    }
      

    


    /*
    main_timer.reset();
    main_timer.start();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      // MOTOR DEBUG
      // serial_pc.printf("\r\n");
      //      serial_pc.printf("MOTOR %d: \r\n", i);
      //      serial_pc.printf("Speed[%d]: %f (%f): \r\n", i, m[i].getMeasuredSpeed(),
      //                       m[i].getSpeedSetPoint());
      //      // serial_pc.printf("Effort: %f: \r\n", m[i].getEffort());
      //      serial_pc.printf("Fault: %u: \r\n", m[i].getFaultPulseCount());
      //      serial_pc.printf("Current[%d]: %f: \r\n", i, m[i].getCurrent());
    }

    //    serial_pc.printf("Serial arrived: %d\r\n", serial_arrived);


*/ 
/*
    s6num = printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(), odom_.getOriZ(),
                     odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
                     */
                    /*
    while(1)
    { 
        status = printUart(  msg , strlen(msg) );
        if(status == HAL_OK)break;
        wait_us(10000);
        retry++;
        if(retry>10){
            
            break;
        }
    }
    retry = 0;

    // Synchronize to given MAIN_DELTA_T
    */
    

    //HAL_UART_Transmit_DMA(&huart2,  msg, strlen(msg));

    //wait_us(MAIN_DELTA_T * 1000 * 1000 - main_timer.read_us());
    
    }
}
