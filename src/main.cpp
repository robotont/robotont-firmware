#include "mbed.h"

#include <sstream>
#include <vector>


#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LL_DMA_CHANNEL_1                  ((uint32_t)0x00000001)
#define LL_DMA_CHANNEL_2                  ((uint32_t)0x00000002)
#define LL_DMA_CHANNEL_3                  ((uint32_t)0x00000003)
#define LL_DMA_CHANNEL_4                  ((uint32_t)0x00000004)
#define LL_DMA_CHANNEL_5                  ((uint32_t)0x00000005)
#define LL_DMA_CHANNEL_6                  ((uint32_t)0x00000006)
#define LL_DMA_CHANNEL_7                  ((uint32_t)0x00000007)

static const uint8_t CHANNEL_OFFSET_TAB[] =
{
  (uint8_t)(DMA1_Channel1_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel2_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel3_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel4_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel5_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel6_BASE - DMA1_BASE),
  (uint8_t)(DMA1_Channel7_BASE - DMA1_BASE)
};


void usart_init(void);
void usart_rx_check(void);
void usart_process_data(const uint8_t* data, size_t len);
//void usart_send_string(const unsigned char* str);
extern "C" void MX_DMA_Init(void);
extern "C"  void Error_Handler(void);
extern "C" void MX_USART2_UART_Init(void);
extern "C" uint32_t LL_DMA_GetDataLength(DMA_TypeDef *DMAx, uint32_t Channel);
extern "C" uint32_t LL_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Channel);
extern "C" uint32_t LL_DMA_IsActiveFlag_TC6(DMA_TypeDef* DMAx);
extern "C" void LL_DMA_ClearFlag_TC6(DMA_TypeDef* DMAx);
extern "C" uint32_t LL_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Channel);
extern "C" uint32_t LL_DMA_IsActiveFlag_HT6(DMA_TypeDef* DMAx);
extern "C" void LL_DMA_ClearFlag_HT6(DMA_TypeDef* DMAx);



UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;


DigitalOut led(LED1);


#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

static uint8_t usart_rx_dma_buffer[64];

int main()
{

    char hello[]="USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n";
    MX_DMA_Init();
    MX_USART2_UART_Init();
    LL_USART_Disable(USART2);
    LL_USART_ConfigNodeAddress(USART2 , LL_USART_ADDRESS_DETECT_7B, 0x0A);
    LL_USART_Enable(USART2);
    HAL_UART_Transmit(&huart2, (uint8_t *)hello, sizeof(hello), 500);
    led = 0;
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)usart_rx_dma_buffer, 64);







  while (true)
  {
    


  }
}


void
usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;
    

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
            //HAL_UART_Transmit(&huart2, (uint8_t *)usart_rx_dma_buffer[old_pos], pos - old_pos, 100);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
            //HAL_UART_Transmit(&huart2, (uint8_t *)usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos, 100);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
                //HAL_UART_Transmit(&huart2, (uint8_t *)usart_rx_dma_buffer[0], pos,100);
            }
        }
        old_pos = pos;                          /* Save current position as old */
    }
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const uint8_t* data, size_t len) {
    //const uint8_t* d = static_cast<const uint8_t *>(data);
    //const uint8_t* d = data;
    /*
     * This function is called on DMA TC and HT events, aswell as on UART IDLE (if enabled) line event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
	
    for (; len > 0; --len, ++data) {
        //HAL_UART_Transmit(&huart2, (uint8_t *)data, 1, 1);
        LL_USART_TransmitData8(USART2, *data);
        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send

void
usart_send_string(const unsigned char* str) {
    const uint8_t* str_uus = static_cast<const uint8_t *>(str);
    usart_process_data(str_uus, strlen(str));
}
 */


extern "C" void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(uartHandle->Instance==USART2){

     __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
     if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

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

}
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
  LL_USART_EnableIT_IDLE(USART2);
  LL_USART_EnableIT_CM(USART2);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);


}





extern "C"  void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

extern "C" void
DMA1_Channel6_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_6) && LL_DMA_IsActiveFlag_HT6(DMA1)) {
        LL_DMA_ClearFlag_HT6(DMA1);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_6) && LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

extern "C" void
USART2_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */

    }
    if ( LL_USART_IsEnabledIT_CM(USART2) && LL_USART_IsActiveFlag_CM(USART2)){
      LL_USART_ClearFlag_CM(USART2);
      usart_rx_check();
      led = !led;  
    }
    


    /* Implement other events when needed */
}

extern "C"  uint32_t LL_DMA_GetDataLength(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel-1])))->CNDTR, DMA_CNDTR_NDT));
}

extern "C" uint32_t LL_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel-1])))->CCR, DMA_CCR_TCIE) == (DMA_CCR_TCIE));
}

extern "C" uint32_t LL_DMA_IsActiveFlag_TC6(DMA_TypeDef* DMAx)
{
  return (READ_BIT(DMAx->ISR, DMA_ISR_TCIF6) == (DMA_ISR_TCIF6));
}

extern "C" void LL_DMA_ClearFlag_TC6(DMA_TypeDef* DMAx)
{
  SET_BIT(DMAx->IFCR, DMA_IFCR_CTCIF6);
}

extern "C" uint32_t LL_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Channel)
{
  return (READ_BIT(((DMA_Channel_TypeDef*)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel-1])))->CCR, DMA_CCR_HTIE) == (DMA_CCR_HTIE));
}
extern "C" uint32_t LL_DMA_IsActiveFlag_HT6(DMA_TypeDef* DMAx)
{
  return (READ_BIT(DMAx->ISR, DMA_ISR_HTIF6) == (DMA_ISR_HTIF6));
}

extern "C" void LL_DMA_ClearFlag_HT6(DMA_TypeDef* DMAx)
{
  SET_BIT(DMAx->IFCR, DMA_IFCR_CHTIF6);
}

