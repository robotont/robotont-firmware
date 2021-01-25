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
// This method processes a received serial packet
void processPacket(const std::string& packet)
{

  // Trim all potential newline symbols from the beginning
  const auto strBegin = packet.find_first_not_of("\r\n");

  std::istringstream ss(packet.substr(strBegin));
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
	    summ += speed_setpoint;
	  
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
	    summ += value;
    }
    ws1.write(px.getBuf());
  }
  else if (cmd[0] == "END")
  {
    tulemuse_saatmine = true;
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
void odom_bool()
{
  odom_boolean = true;
  
}

int main()
{
  // Initialize serial connection
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN ****\r\n");

  cmd_timeout_checker.attach(check_for_timeout, 0.1);
  //odom_ticker.attach(odom_bool, 0.02);

  cmd_timer.start();
  

  // MAIN LOOP
  while (true)
  {
    main_timer.reset();


    
 
    


    if (packet_received_b)  // packet was completeted with \r \n
    {
      std::string packet(serial_buf);
      serial_buf[0] = '\0';
      serial_arrived = 0;
      processPacket(packet);
      
      if(summ == 200)
      {
        lugejams++;
      }
      else if (summ == 767057416)
      {
        lugejaled++;
      }
      else
      {
        checksum_loendur++;
      }
      //serial_pc.printf("%u\r\n",summ);
      summ = 0;
      
      

      packet_received_b = false;
      loendur++;
    }
    if(tulemuse_saatmine) {
            led = 1;
            for (size_t i = 0; i < 100; i++)
            {
              serial_pc.printf("MS %d, LED %d, LOENDUR %d,CSUM %d\r\n",lugejams,lugejaled,loendur-1,checksum_loendur);
              wait_us(10000);
            }    
            serial_pc.printf("Lugejad reset\r\n");
            lugejams = 0;
            lugejaled = 0;
            loendur = 0;
            checksum_loendur =0;
            tulemuse_saatmine = false;
        }
    else
    {
      led =0;
    }
    

    /*if (odom_boolean)
    {
          }
    
*/
          // Update odometry
    //odom_boolean=false;
    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
    serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(), odom_.getOriZ(),
                    odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
    // Synchronize to given MAIN_DELTA_T
    wait_us(MAIN_DELTA_T * 1000 * 1000 - main_timer.read_us());

  
  }

}
