/*
 * mx.c
 *
 *  Created on: 2019/04/06
 *      Author: aruaru
 */
#include <stdlib.h>
#include "main.h"
#include "usart.h"
#include "mx.h"

#define TX_BUFFER_SIZE 4000
#define RX_BUFFER_SIZE 40

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

static char txbuffer[TX_BUFFER_SIZE];
static uint8_t rxbuf[RX_BUFFER_SIZE];
typedef struct{
    uint8_t *b;
    size_t size;
    int pos;
}RxBuffer_t;
RxBuffer_t rxBuffer;

/*=================================
 *  以下はstm32l4xx_it.cから
 ==================================*/


/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}
/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
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


/*
 * dmaを利用するための関数
 */

/*
 * ローカルのバッファーにコピー後に出力
 * 呼び出し元のバッファはなくなる可能性がある
 */
HAL_StatusTypeDef printUart(char* buf,int num){
    HAL_StatusTypeDef status=HAL_ERROR;
    if(hdma_usart2_tx.Instance->CNDTR!=0){
        return HAL_BUSY;
    }
    for(int i=0;i<num && i<4000 ;i++){
        *(txbuffer+i)=*buf++;
    }

    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)txbuffer, num);

    return status;
}
HAL_StatusTypeDef startRxUart(void){

    HAL_StatusTypeDef status;

    rxBuffer.size=RX_BUFFER_SIZE;
    rxBuffer.b = (uint8_t*)malloc(sizeof(char)*rxBuffer.size);
    if(rxBuffer.b==0)return HAL_ERROR;
    rxBuffer.pos = 0;

    status = HAL_UART_Receive_DMA(&huart2, rxBuffer.b, rxBuffer.size);
    return status;
}
int getChar(char* c){
    if(readable()>0){
        *c=*(rxBuffer.b+rxBuffer.pos++);
        rxBuffer.pos %= rxBuffer.size;
        return 1;
    }else{
        return 0;
    }
}
int readable(void){
    int cndtr,pos;
    cndtr = hdma_usart2_rx.Instance->CNDTR;
    pos = rxBuffer.size - cndtr;
    if( pos != rxBuffer.pos ){
        return 1;
    }else{
        return 0;
    }
}
int readCndtr(void){
    return (int) hdma_usart2_rx.Instance->CNDTR;
}
int readPos(void){
    return rxBuffer.pos;
}
