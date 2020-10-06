#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include "Json.h"
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
volatile uint8_t serial_arrived = 0;  // Number of bytes arrived
volatile bool packet_received_b = false;
char motorValue1 [128];
char motorValue2 [128];
char motorValue3 [128];
char ledValue [2048];

// For parsing command with arguments received over serial
std::vector<std::string> cmd;
void processJsonPacket(const char * packet)
{
   Json json ( packet, strlen (packet) );
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
        int ledArrayIndex = json.findKeyIndex( "LED", 0 );
		
        if ( motorValueIndex1 == -1 || motorValueIndex2 == -1 || motorValueIndex3 == -1)
        {
            // Error handling part ...
            serial_pc.printf( "One motor value does not excist" );
            return;
        }
        else
        {

			//serial_pc.printf("%s", cityValueIndex);
          if ( motorValueIndex1 > 0 || motorValueIndex2 > 0 || motorValueIndex3 > 0)
          {
              const char * valueStart1  = json.tokenAddress (motorValueIndex1+1);
              int          valueLength1 = json.tokenLength (motorValueIndex1+1);
              strncpy (motorValue1, valueStart1, valueLength1 );

              const char * valueStart2  = json.tokenAddress (motorValueIndex1+1);
              int          valueLength2 = json.tokenLength (motorValueIndex1+1);
              strncpy (motorValue2, valueStart2, valueLength2 );
            
              const char * valueStart3  = json.tokenAddress (motorValueIndex1+1);
              int          valueLength3 = json.tokenLength (motorValueIndex1+1);
              strncpy (motorValue3, valueStart3, valueLength3 );
              int mv1 = stoi(motorValue1);
              int mv2 = stoi(motorValue2); 
              int mv3 = stoi(motorValue3);   

      

        
              
              serial_pc.printf( "%d", mv1);
              serial_pc.printf( "%d", mv2);
              serial_pc.printf( "%d \n", mv3);
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
            const char * valueStart4  = json.tokenAddress (ledArrayIndex+1);
            int          valueLength4 = json.tokenLength (ledArrayIndex+1);
            strncpy (ledValue, valueStart4, valueLength4 );
            serial_pc.printf( "%s \n", ledValue);

          }
        }
        
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
      //serial_pc.printf("Got arg %s\r\n", arg.c_str());
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
      serial_pc.printf("Setpoint %d, %f\r\n", i, speed_setpoint);
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
      float speed = lin_speed_mag * sin(lin_speed_dir - m[i].getWheelPosPhi()) +
                    m[i].getWheelPosR() * angular_speed_z;
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
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN ****\r\n");

  cmd_timeout_checker.attach(check_for_timeout, 0.1);
  cmd_timer.start();


  // MAIN LOOP
  while (true)
  {
    main_timer.reset();
    main_timer.start();




    
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      // MOTOR DEBUG
        //serial_pc.printf("\r\n");
	//serial_pc.printf("MOTOR %d: \r\n", i);
//      serial_pc.printf("Speed[%d]: %f (%f): \r\n", i, m[i].getMeasuredSpeed(),
//                       m[i].getSpeedSetPoint());
//      // serial_pc.printf("Effort: %f: \r\n", m[i].getEffort());
//      serial_pc.printf("Fault: %u: \r\n", m[i].getFaultPulseCount());
//      serial_pc.printf("Current[%d]: %f: \r\n", i, m[i].getCurrent());
    }

//    serial_pc.printf("Serial arrived: %d\r\n", serial_arrived);
    
    if (packet_received_b) // packet was completeted with \r \n
    { 
     
      std::string packet(serial_buf);
      
      serial_buf[0] = '\0';
      serial_arrived = 0;
      
      char *cstr = new char[packet.length() + 1];
      strcpy(cstr, packet.c_str());
      // do stuff
      processJsonPacket(cstr);
      delete [] cstr;
      
      packet_received_b = false;
    }
    
    // Update odometry
    //odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
    //serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(),
        //             odom_.getOriZ(), odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
    // Synchronize to given MAIN_DELTA_T
    wait_us(MAIN_DELTA_T*1000*1000 - main_timer.read_us());
  }
}
