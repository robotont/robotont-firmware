#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include "Json.h"
#include "WS2812.h"
#include "PixelArray.h"
#include <sstream>
#include <vector>

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

#define MAX_CMD_ARGS 5
#define MOTOR_COUNT 3
#define CMD_TIMEOUT_MS 1000 // If velocity command is not received within this period all motors are stopped.

// Include motor configurations
//#include "motor_config_v0_6.h"
#include "motor_config_v2_1.h"

//values
int mv1;
int mv2;
int mv3;
int rv1;
int rv2;
int rv3;
int temp_led_value;

// LED STRIP
// Set the number of pixels of the strip
#define WS2812_BUF 60

PixelArray px(WS2812_BUF);
WS2812 ws1(PA_15, WS2812_BUF, 1, 12, 6, 11);

// Initialize motors
Motor m[] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Initialize odometry
Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);


// Timeout
Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;

// Variables for serial connection
RawSerial serial_pc(USBTX, USBRX);  // tx, rx
char serial_buf[4096];        // Buffer for incoming serial data
volatile uint32_t serial_arrived = 0;  // Number of bytes arrived
volatile bool packet_received_b = false;
volatile bool reset_timer = false;
// For parsing command with arguments received over serial


void processJsonPacket(const char * packet)
{   

  

   Json json ( packet, strlen (packet),128 );

    if ( !json.isValidJson () )
        {   
            serial_pc.printf("Invalid JSON");

            return;
        }

        if ( json.type (0) != JSMN_OBJECT )
        {
            serial_pc.printf( "Invalid JSON.  ROOT element is not Object");
            return;
            
        }

       // ROOT object should have '0' tokenIndex, and -1 parentIndex
        int motorValueIndex1 = json.findKeyIndex( "M1", 0 );
        int motorValueIndex2 = json.findKeyIndex( "M2", 0 );
        int motorValueIndex3 = json.findKeyIndex( "M3", 0 );
       

        int robotValueIndex1 = json.findKeyIndex( "R1", 0 );
        int robotValueIndex2 = json.findKeyIndex( "R2", 0 );
        int robotValueIndex3 = json.findKeyIndex( "R3", 0 );
        int ledArrayIndex = json.findKeyIndex( "LED", 0 );
        int ledcount = json.childCount(ledArrayIndex+1);




        if ( motorValueIndex1 == -1 || motorValueIndex2 == -1 || motorValueIndex3 == -1)
        {
            // Error handling part ...
            serial_pc.printf( "One motor value does not excist" );
            return;
        }
        else
        {

          if ( motorValueIndex1 > 0 || motorValueIndex2 > 0 || motorValueIndex3 > 0)
          {

              // Get motor values and update them
              json.tokenIntegerValue(motorValueIndex1+1,mv1);
              json.tokenIntegerValue(motorValueIndex2+1,mv2); 
              json.tokenIntegerValue(motorValueIndex3+1,mv3);  
              m[1].setSpeedSetPoint(mv1);
              m[2].setSpeedSetPoint(mv2);
              m[3].setSpeedSetPoint(mv3);
              cmd_timer.reset();

          }
        }
        


        if ( motorValueIndex1 == -1 || motorValueIndex2 == -1 || motorValueIndex3 == -1)
        {
            // Error handling part ...
            serial_pc.printf( "One motor value does not excist" );
            return;
        }
        else
        {


          if ( robotValueIndex1 > 0 || robotValueIndex2 > 0 || robotValueIndex3 > 0)
          {
              // Get robot value and update them 

              json.tokenIntegerValue(robotValueIndex1+1,rv1);
              json.tokenIntegerValue(robotValueIndex2+1,rv2); 
              json.tokenIntegerValue(robotValueIndex3+1,rv3);


          }
        }


        if ( ledArrayIndex == -1){
          serial_pc.printf( "led value does not excist " );
         return;
          
        }
        else
        {


          if(ledArrayIndex > 0)
          {
            ws1.useII(WS2812::GLOBAL);
            ws1.setII(0xFF); 
            for (int i = 0; i < ledcount; i++){
              json.tokenIntegerValue(ledArrayIndex+2+i,temp_led_value);
              px.Set(i, temp_led_value);

            }
          ws1.write(px.getBuf());
              

          }

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
    if (serial_arrived > 4095)
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
  serial_pc.baud(256000);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN ****\r\n");

  cmd_timeout_checker.attach(check_for_timeout, 0.1);
  cmd_timer.start();
  t.start();
  main_timer.start();

  // MAIN LOOP
  while (true)
  {
    
    if (reset_timer) main_timer.reset();

    t.reset();


    if (packet_received_b) // packet was completeted with \r \n
    { 
      processJsonPacket(serial_buf);

      serial_buf[0] = '\0';
      serial_arrived = 0;
      

      packet_received_b = false;
      
    }
    
    // Update odometry
    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
    serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(),
                     odom_.getOriZ(), odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
    // Synchronize to given MAIN_DELTA_T
    if (MAIN_DELTA_T-(MAIN_DELTA_T*0.4)*1000*1000-main_timer.read_us())
    {
      wait_us(MAIN_DELTA_T*1000*1000 - main_timer.read_us());
      reset_timer = true;
      
    }

  }
}
