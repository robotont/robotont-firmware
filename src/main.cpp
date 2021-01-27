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

// Include motor configurations
//#include "motor_config_v0_6.h"
#include "motor_config_v2_1.h"

// Initialize motors
Motor m[] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Initialize odometry
Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);

// Timeout
Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker,odom_ticker;

// Variables for serial connection
RawSerial serial_pc(USBTX, USBRX);     // tx, rx
SPI spi(D11, D12, D13); // mosi, miso, sclk
char serial_buf[SERIAL_BUF_SIZE];      // Buffer for incoming serial data
volatile uint16_t serial_arrived = 0;  // Number of bytes arrived
volatile bool packet_received_b = false;
uint32_t summ = 0;
uint16_t lugejams =0;
uint16_t lugejaled =0;
uint16_t loendur =0;
uint16_t checksum_loendur =0;
bool odom_boolean = true;
bool tulemuse_saatmine = false;
// For parsing command with arguments received over serial
std::vector<std::string> cmd;

// LED STRIP
// Set the number of pixels of the strip
#define WS2812_BUF 60

PixelArray px(WS2812_BUF);
WS2812 ws1(PA_15, WS2812_BUF, 1, 12, 6, 11);

DigitalOut led(LED1);
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
#define LED_NO    10

#define LED_BUFFER_LENGTH (LED_NO*12)
const uint8_t leddata[256*4] = { // size = 256 * 3
0X44 , 0X44 , 0X44 , 0X44 , // 0
0X44 , 0X44 , 0X44 , 0X47 , // 1
0X44 , 0X44 , 0X44 , 0X74 , 
0X44 , 0X44 , 0X44 , 0X77 , 
0X44 , 0X44 , 0X47 , 0X44 , 
0X44 , 0X44 , 0X47 , 0X47 , 
0X44 , 0X44 , 0X47 , 0X74 , 
0X44 , 0X44 , 0X47 , 0X77 , 
0X44 , 0X44 , 0X74 , 0X44 , 
0X44 , 0X44 , 0X74 , 0X47 , 
0X44 , 0X44 , 0X74 , 0X74 , 
0X44 , 0X44 , 0X74 , 0X77 , 
0X44 , 0X44 , 0X77 , 0X44 , 
0X44 , 0X44 , 0X77 , 0X47 , 
0X44 , 0X44 , 0X77 , 0X74 , 
0X44 , 0X44 , 0X77 , 0X77 , 
0X44 , 0X47 , 0X44 , 0X44 , 
0X44 , 0X47 , 0X44 , 0X47 , 
0X44 , 0X47 , 0X44 , 0X74 , 
0X44 , 0X47 , 0X44 , 0X77 , 
0X44 , 0X47 , 0X47 , 0X44 , 
0X44 , 0X47 , 0X47 , 0X47 , 
0X44 , 0X47 , 0X47 , 0X74 , 
0X44 , 0X47 , 0X47 , 0X77 , 
0X44 , 0X47 , 0X74 , 0X44 , 
0X44 , 0X47 , 0X74 , 0X47 , 
0X44 , 0X47 , 0X74 , 0X74 , 
0X44 , 0X47 , 0X74 , 0X77 , 
0X44 , 0X47 , 0X77 , 0X44 , 
0X44 , 0X47 , 0X77 , 0X47 , 
0X44 , 0X47 , 0X77 , 0X74 , 
0X44 , 0X47 , 0X77 , 0X77 , 
0X44 , 0X74 , 0X44 , 0X44 , 
0X44 , 0X74 , 0X44 , 0X47 , 
0X44 , 0X74 , 0X44 , 0X74 , 
0X44 , 0X74 , 0X44 , 0X77 , 
0X44 , 0X74 , 0X47 , 0X44 , 
0X44 , 0X74 , 0X47 , 0X47 , 
0X44 , 0X74 , 0X47 , 0X74 , 
0X44 , 0X74 , 0X47 , 0X77 , 
0X44 , 0X74 , 0X74 , 0X44 , 
0X44 , 0X74 , 0X74 , 0X47 , 
0X44 , 0X74 , 0X74 , 0X74 , 
0X44 , 0X74 , 0X74 , 0X77 , 
0X44 , 0X74 , 0X77 , 0X44 , 
0X44 , 0X74 , 0X77 , 0X47 , 
0X44 , 0X74 , 0X77 , 0X74 , 
0X44 , 0X74 , 0X77 , 0X77 , 
0X44 , 0X77 , 0X44 , 0X44 , 
0X44 , 0X77 , 0X44 , 0X47 , 
0X44 , 0X77 , 0X44 , 0X74 , 
0X44 , 0X77 , 0X44 , 0X77 , 
0X44 , 0X77 , 0X47 , 0X44 , 
0X44 , 0X77 , 0X47 , 0X47 , 
0X44 , 0X77 , 0X47 , 0X74 , 
0X44 , 0X77 , 0X47 , 0X77 , 
0X44 , 0X77 , 0X74 , 0X44 , 
0X44 , 0X77 , 0X74 , 0X47 , 
0X44 , 0X77 , 0X74 , 0X74 , 
0X44 , 0X77 , 0X74 , 0X77 , 
0X44 , 0X77 , 0X77 , 0X44 , 
0X44 , 0X77 , 0X77 , 0X47 , 
0X44 , 0X77 , 0X77 , 0X74 , 
0X44 , 0X77 , 0X77 , 0X77 , 
0X47 , 0X44 , 0X44 , 0X44 , 
0X47 , 0X44 , 0X44 , 0X47 , 
0X47 , 0X44 , 0X44 , 0X74 , 
0X47 , 0X44 , 0X44 , 0X77 , 
0X47 , 0X44 , 0X47 , 0X44 , 
0X47 , 0X44 , 0X47 , 0X47 , 
0X47 , 0X44 , 0X47 , 0X74 , 
0X47 , 0X44 , 0X47 , 0X77 , 
0X47 , 0X44 , 0X74 , 0X44 , 
0X47 , 0X44 , 0X74 , 0X47 , 
0X47 , 0X44 , 0X74 , 0X74 , 
0X47 , 0X44 , 0X74 , 0X77 , 
0X47 , 0X44 , 0X77 , 0X44 , 
0X47 , 0X44 , 0X77 , 0X47 , 
0X47 , 0X44 , 0X77 , 0X74 , 
0X47 , 0X44 , 0X77 , 0X77 , 
0X47 , 0X47 , 0X44 , 0X44 , 
0X47 , 0X47 , 0X44 , 0X47 , 
0X47 , 0X47 , 0X44 , 0X74 , 
0X47 , 0X47 , 0X44 , 0X77 , 
0X47 , 0X47 , 0X47 , 0X44 , 
0X47 , 0X47 , 0X47 , 0X47 , 
0X47 , 0X47 , 0X47 , 0X74 , 
0X47 , 0X47 , 0X47 , 0X77 , 
0X47 , 0X47 , 0X74 , 0X44 , 
0X47 , 0X47 , 0X74 , 0X47 , 
0X47 , 0X47 , 0X74 , 0X74 , 
0X47 , 0X47 , 0X74 , 0X77 , 
0X47 , 0X47 , 0X77 , 0X44 , 
0X47 , 0X47 , 0X77 , 0X47 , 
0X47 , 0X47 , 0X77 , 0X74 , 
0X47 , 0X47 , 0X77 , 0X77 , 
0X47 , 0X74 , 0X44 , 0X44 , 
0X47 , 0X74 , 0X44 , 0X47 , 
0X47 , 0X74 , 0X44 , 0X74 , 
0X47 , 0X74 , 0X44 , 0X77 , 
0X47 , 0X74 , 0X47 , 0X44 , 
0X47 , 0X74 , 0X47 , 0X47 , 
0X47 , 0X74 , 0X47 , 0X74 , 
0X47 , 0X74 , 0X47 , 0X77 , 
0X47 , 0X74 , 0X74 , 0X44 , 
0X47 , 0X74 , 0X74 , 0X47 , 
0X47 , 0X74 , 0X74 , 0X74 , 
0X47 , 0X74 , 0X74 , 0X77 , 
0X47 , 0X74 , 0X77 , 0X44 , 
0X47 , 0X74 , 0X77 , 0X47 , 
0X47 , 0X74 , 0X77 , 0X74 , 
0X47 , 0X74 , 0X77 , 0X77 , 
0X47 , 0X77 , 0X44 , 0X44 , 
0X47 , 0X77 , 0X44 , 0X47 , 
0X47 , 0X77 , 0X44 , 0X74 , 
0X47 , 0X77 , 0X44 , 0X77 , 
0X47 , 0X77 , 0X47 , 0X44 , 
0X47 , 0X77 , 0X47 , 0X47 , 
0X47 , 0X77 , 0X47 , 0X74 , 
0X47 , 0X77 , 0X47 , 0X77 , 
0X47 , 0X77 , 0X74 , 0X44 , 
0X47 , 0X77 , 0X74 , 0X47 , 
0X47 , 0X77 , 0X74 , 0X74 , 
0X47 , 0X77 , 0X74 , 0X77 , 
0X47 , 0X77 , 0X77 , 0X44 , 
0X47 , 0X77 , 0X77 , 0X47 , 
0X47 , 0X77 , 0X77 , 0X74 , 
0X47 , 0X77 , 0X77 , 0X77 , 
0X74 , 0X44 , 0X44 , 0X44 , 
0X74 , 0X44 , 0X44 , 0X47 , 
0X74 , 0X44 , 0X44 , 0X74 , 
0X74 , 0X44 , 0X44 , 0X77 , 
0X74 , 0X44 , 0X47 , 0X44 , 
0X74 , 0X44 , 0X47 , 0X47 , 
0X74 , 0X44 , 0X47 , 0X74 , 
0X74 , 0X44 , 0X47 , 0X77 , 
0X74 , 0X44 , 0X74 , 0X44 , 
0X74 , 0X44 , 0X74 , 0X47 , 
0X74 , 0X44 , 0X74 , 0X74 , 
0X74 , 0X44 , 0X74 , 0X77 , 
0X74 , 0X44 , 0X77 , 0X44 , 
0X74 , 0X44 , 0X77 , 0X47 , 
0X74 , 0X44 , 0X77 , 0X74 , 
0X74 , 0X44 , 0X77 , 0X77 , 
0X74 , 0X47 , 0X44 , 0X44 , 
0X74 , 0X47 , 0X44 , 0X47 , 
0X74 , 0X47 , 0X44 , 0X74 , 
0X74 , 0X47 , 0X44 , 0X77 , 
0X74 , 0X47 , 0X47 , 0X44 , 
0X74 , 0X47 , 0X47 , 0X47 , 
0X74 , 0X47 , 0X47 , 0X74 , 
0X74 , 0X47 , 0X47 , 0X77 , 
0X74 , 0X47 , 0X74 , 0X44 , 
0X74 , 0X47 , 0X74 , 0X47 , 
0X74 , 0X47 , 0X74 , 0X74 , 
0X74 , 0X47 , 0X74 , 0X77 , 
0X74 , 0X47 , 0X77 , 0X44 , 
0X74 , 0X47 , 0X77 , 0X47 , 
0X74 , 0X47 , 0X77 , 0X74 , 
0X74 , 0X47 , 0X77 , 0X77 , 
0X74 , 0X74 , 0X44 , 0X44 , 
0X74 , 0X74 , 0X44 , 0X47 , 
0X74 , 0X74 , 0X44 , 0X74 , 
0X74 , 0X74 , 0X44 , 0X77 , 
0X74 , 0X74 , 0X47 , 0X44 , 
0X74 , 0X74 , 0X47 , 0X47 , 
0X74 , 0X74 , 0X47 , 0X74 , 
0X74 , 0X74 , 0X47 , 0X77 , 
0X74 , 0X74 , 0X74 , 0X44 , 
0X74 , 0X74 , 0X74 , 0X47 , 
0X74 , 0X74 , 0X74 , 0X74 , 
0X74 , 0X74 , 0X74 , 0X77 , 
0X74 , 0X74 , 0X77 , 0X44 , 
0X74 , 0X74 , 0X77 , 0X47 , 
0X74 , 0X74 , 0X77 , 0X74 , 
0X74 , 0X74 , 0X77 , 0X77 , 
0X74 , 0X77 , 0X44 , 0X44 , 
0X74 , 0X77 , 0X44 , 0X47 , 
0X74 , 0X77 , 0X44 , 0X74 , 
0X74 , 0X77 , 0X44 , 0X77 , 
0X74 , 0X77 , 0X47 , 0X44 , 
0X74 , 0X77 , 0X47 , 0X47 , 
0X74 , 0X77 , 0X47 , 0X74 , 
0X74 , 0X77 , 0X47 , 0X77 , 
0X74 , 0X77 , 0X74 , 0X44 , 
0X74 , 0X77 , 0X74 , 0X47 , 
0X74 , 0X77 , 0X74 , 0X74 , 
0X74 , 0X77 , 0X74 , 0X77 , 
0X74 , 0X77 , 0X77 , 0X44 , 
0X74 , 0X77 , 0X77 , 0X47 , 
0X74 , 0X77 , 0X77 , 0X74 , 
0X74 , 0X77 , 0X77 , 0X77 , 
0X77 , 0X44 , 0X44 , 0X44 , 
0X77 , 0X44 , 0X44 , 0X47 , 
0X77 , 0X44 , 0X44 , 0X74 , 
0X77 , 0X44 , 0X44 , 0X77 , 
0X77 , 0X44 , 0X47 , 0X44 , 
0X77 , 0X44 , 0X47 , 0X47 , 
0X77 , 0X44 , 0X47 , 0X74 , 
0X77 , 0X44 , 0X47 , 0X77 , 
0X77 , 0X44 , 0X74 , 0X44 , 
0X77 , 0X44 , 0X74 , 0X47 , 
0X77 , 0X44 , 0X74 , 0X74 , 
0X77 , 0X44 , 0X74 , 0X77 , 
0X77 , 0X44 , 0X77 , 0X44 , 
0X77 , 0X44 , 0X77 , 0X47 , 
0X77 , 0X44 , 0X77 , 0X74 , 
0X77 , 0X44 , 0X77 , 0X77 , 
0X77 , 0X47 , 0X44 , 0X44 , 
0X77 , 0X47 , 0X44 , 0X47 , 
0X77 , 0X47 , 0X44 , 0X74 , 
0X77 , 0X47 , 0X44 , 0X77 , 
0X77 , 0X47 , 0X47 , 0X44 , 
0X77 , 0X47 , 0X47 , 0X47 , 
0X77 , 0X47 , 0X47 , 0X74 , 
0X77 , 0X47 , 0X47 , 0X77 , 
0X77 , 0X47 , 0X74 , 0X44 , 
0X77 , 0X47 , 0X74 , 0X47 , 
0X77 , 0X47 , 0X74 , 0X74 , 
0X77 , 0X47 , 0X74 , 0X77 , 
0X77 , 0X47 , 0X77 , 0X44 , 
0X77 , 0X47 , 0X77 , 0X47 , 
0X77 , 0X47 , 0X77 , 0X74 , 
0X77 , 0X47 , 0X77 , 0X77 , 
0X77 , 0X74 , 0X44 , 0X44 , 
0X77 , 0X74 , 0X44 , 0X47 , 
0X77 , 0X74 , 0X44 , 0X74 , 
0X77 , 0X74 , 0X44 , 0X77 , 
0X77 , 0X74 , 0X47 , 0X44 , 
0X77 , 0X74 , 0X47 , 0X47 , 
0X77 , 0X74 , 0X47 , 0X74 , 
0X77 , 0X74 , 0X47 , 0X77 , 
0X77 , 0X74 , 0X74 , 0X44 , 
0X77 , 0X74 , 0X74 , 0X47 , 
0X77 , 0X74 , 0X74 , 0X74 , 
0X77 , 0X74 , 0X74 , 0X77 , 
0X77 , 0X74 , 0X77 , 0X44 , 
0X77 , 0X74 , 0X77 , 0X47 , 
0X77 , 0X74 , 0X77 , 0X74 , 
0X77 , 0X74 , 0X77 , 0X77 , 
0X77 , 0X77 , 0X44 , 0X44 , 
0X77 , 0X77 , 0X44 , 0X47 , 
0X77 , 0X77 , 0X44 , 0X74 , 
0X77 , 0X77 , 0X44 , 0X77 , 
0X77 , 0X77 , 0X47 , 0X44 , 
0X77 , 0X77 , 0X47 , 0X47 , 
0X77 , 0X77 , 0X47 , 0X74 , 
0X77 , 0X77 , 0X47 , 0X77 , 
0X77 , 0X77 , 0X74 , 0X44 , 
0X77 , 0X77 , 0X74 , 0X47 , 
0X77 , 0X77 , 0X74 , 0X74 , 
0X77 , 0X77 , 0X74 , 0X77 , 
0X77 , 0X77 , 0X77 , 0X44 , 
0X77 , 0X77 , 0X77 , 0X47 , 
0X77 , 0X77 , 0X77 , 0X74 , 
0X77 , 0X77 , 0X77 , 0X77 , 

};

uint8_t ws_buffer[LED_BUFFER_LENGTH];

void encode_byte( uint8_t data, int16_t buffer_index )
{
   int index = data * 4;
   ws_buffer[buffer_index++ ] = leddata[index++];
   ws_buffer[buffer_index++ ] = leddata[index++];
   ws_buffer[buffer_index++ ] = leddata[index++];
   ws_buffer[buffer_index++ ] = leddata[index++];
}
void generate_ws_buffer( uint8_t RData,uint8_t GData,uint8_t BData, int16_t led_no )
{
	//ws2812b
//G--R--B
//MSB first	
   int offset = led_no * 12;
   encode_byte( GData, offset );
   encode_byte( RData, offset+4 );
   encode_byte( BData, offset+8 );   
}
void Send_2812(void)
 {   
#if 1    
    HAL_SPI_Transmit_DMA( &hspi1, ws_buffer, LED_BUFFER_LENGTH ); 
    // wait until finished
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY ));
#else
    HAL_SPI_Transmit( &hspi1, ws_buffer, LED_BUFFER_LENGTH, 300 );
#endif
 } 
 
void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{ 
   int i;
   for(i=0;i< LED_NO;i++) {
      generate_ws_buffer( r, g, b, i );
   }
   Send_2812();
}
 void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
 {	 
   generate_ws_buffer( r, g, b, n );
   Send_2812();
}
/**
 * initialize MOSI pin to LOW.  Without this, first time transmit for first LED might be wrong.
 *
 */
void initLEDMOSI(void)
{
   uint8_t buffer0[2] = { 0, 0 };
   HAL_SPI_Transmit(&hspi1, buffer0, 1, 100 );
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

  	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);


}


extern "C" void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
extern "C"  void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}
extern "C"  void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}

extern "C" void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

extern "C"  void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }

}

// Process an incoming serial byte
void pc_rx_callback()
{
  // Store bytes from serial in our buffer until packet
  // termination byte 'enter', '\n', '\r' etc has arrived
  while (serial_pc.readable())
  {
    char c = serial_pc.getc();
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
  }
}


int main()
{
  // Initialize serial connection
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN1 ****\r\n");
  uint16_t data = {0x8e};
  
  MX_SPI1_Init();
  MX_DMA_Init();
  initLEDMOSI();
  while (true)
  {
    int8_t i;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    setAllPixelColor( 0, 0, 0);
    wait_us(200000);
    // red
    for ( i = 0; i < LED_NO; i++) {
       setPixelColor( i, 250, 0, 0 );
       wait_us(200000);
    }
    // green
    for ( i = 0; i < LED_NO; i++) {
       setPixelColor( i, 0, 250, 0 );
       wait_us(200000);
    }
    // blue
    for ( i = 0; i < LED_NO; i++) {
       setPixelColor( i, 0, 0, 250 );
       wait_us(200000);
    }
    
  }

}
