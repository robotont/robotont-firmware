#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include "DS1820.h"
#include "VL53L0X.h"
#include "WS2812.h"
#include "PixelArray.h"
#include <sstream>
#include <vector>

// Common parameters for all motors
#define ENC_CPR 64
#define GEAR_RATIO 18.75
#define WHEEL_RADIUS 0.035
#define WHEEL_POS_R 0.127
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
#include "motor_config_v1_1.h"
// LED STRIP 
#define WS2812_BUF 18
#define NUM_COLORS 2
#define NUM_LEDS_PER_COLOR 3
PixelArray px(WS2812_BUF);
WS2812 ws1(D6, WS2812_BUF, 3, 12, 9, 12);

//CONFIGURE i2c pins!
I2C i2c(PB_9, PB_8);

// Initialize motors
Motor m[] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Initialize odometry
Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);

// Timeout
Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;
//VL53L0X sensor constructor
VL53L0X     sensor1(i2c, main_timer);
VL53L0X     sensor2(i2c, main_timer);
VL53L0X     sensor3(i2c, main_timer);
VL53L0X     sensor4(i2c, main_timer);
VL53L0X     sensor5(i2c, main_timer);
VL53L0X     sensor6(i2c, main_timer);
VL53L0X     sensor7(i2c, main_timer);
VL53L0X     sensor8(i2c, main_timer);
VL53L0X     sensor9(i2c, main_timer);
VL53L0X     sensor10(i2c, main_timer);
VL53L0X     sensor11(i2c, main_timer);
VL53L0X     sensor12(i2c, main_timer);
#define HIGH_SPEED

// Variables for serial connection
RawSerial serial_pc(USBTX, USBRX);  // tx, rx
char serial_buf[256];        // Buffer for incoming serial data
volatile uint8_t serial_arrived = 0;  // Number of bytes arrived
volatile bool packet_received_b = false;

// For parsing command with arguments received over serial
std::vector<std::string> cmd;

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
      //serial_pc.printf("Setpoint %d, %f\r\n", i, speed_setpoint);
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
  else if (cmd[0] == "LED")  // Update single LED state.
  { 
    ws1.useII(WS2812::GLOBAL);
    int index = std::atof(cmd[1].c_str()); //led index. Value 0 = First led.
    int rgb = std::atof(cmd[2].c_str()); //0xFF0000 - red, 0x00FF00 - green, 0x0000FF blue (color).
    px.Set(index, rgb);
    ws1.write(px.getBuf());
  }
  else if (cmd[0] == "LED_SEG")  // Update LED states for a segment.
  {
    ws1.useII(WS2812::GLOBAL);
    int start_index = std::atof(cmd[1].c_str());
    int argumentsSize = cmd.size();
    for(int i = 2; i < argumentsSize; i++) {
	px.Set(start_index, (int) std::atof(cmd[i].c_str()));
        start_index++;
    } 
    ws1.write(px.getBuf());
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
    if (serial_arrived > 254)
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

void i2cScanner() 
{
    serial_pc.printf("\nI2C Scanner\r\n");
    i2c.frequency(100000);
    while(1) {
        int error, address;
        int nDevices;
 
        serial_pc.printf("Scanning...\r\n");
 
         nDevices = 0;
 
          for(address = 1; address < 127; address++ )
          {
            //i2c.start();
            error = i2c.write(address << 1, NULL, 0); //We shift it left because mbed takes in 8 bit addreses
            //i2c.stop();
            if (error == 0)
            {
              serial_pc.printf("I2C device found at address 0x%X\r\n", address); //Returns 7-bit addres
              nDevices++;
	      //break;
            }
            if (error == 2) {
              serial_pc.printf("TIMEOUTED...\r\n");
            }
 
          }
          if (nDevices == 0)
            serial_pc.printf("No I2C devices found\r\n");
          else
            serial_pc.printf("\ndone\r\n");
        }
}

void i2cGPIOexpanderConfigure(int address, char pinState)  
{
  char conf_reg[2] = {0x03, pinState};  //0x00 output, 0xFF input
  int error = i2c.write(address << 1, conf_reg, sizeof(conf_reg));
  serial_pc.printf("Result: %s\n", (error == 0?"ACK \r\n":"NAK \r\n"));  
  wait_ms(10);
}

void i2cGPIOexpanderINIT(int address) 
{
  i2c.frequency(100000);
  char conf_reg[2] = {0x03, 0xF0};  //0x00 output, 0xFF input
  int error = i2c.write(address << 1, conf_reg, sizeof(conf_reg));
  serial_pc.printf("Result: %s\n", (error == 0?"ACK \r\n":"NAK \r\n"));
  char conf_reg2[2] = {0x01, 0x00}; //Last 4bits for PIN configure // ACTIVE-LOW STATE
  error = i2c.write(address << 1, conf_reg2, sizeof(conf_reg2));
  serial_pc.printf("Result: %s\n", (error == 0?"ACK \r\n":"NAK \r\n"));
}

void initSensors() 
{ 
  i2cGPIOexpanderINIT(0x20);
  i2cGPIOexpanderINIT(0x21);
  i2cGPIOexpanderINIT(0x22);

  i2cGPIOexpanderConfigure(0x20, 0b00001000); 
  sensor1.setAddress(0x30); 
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x20, 0b00001100); 
  sensor2.setAddress(0x31);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x20, 0b00001110); 
  sensor3.setAddress(0x32);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x20, 0b00001111); 
  sensor4.setAddress(0x33);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x21, 0b00001000); 
  sensor5.setAddress(0x34);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x21, 0b00001100); 
  sensor6.setAddress(0x35);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x21, 0b00001110); 
  sensor7.setAddress(0x36);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x21, 0b00001111);
  sensor8.setAddress(0x37);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x22, 0b00001000); 
  sensor9.setAddress(0x38);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x22, 0b00001100); 
  sensor10.setAddress(0x39);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x22, 0b00001110); 
  sensor11.setAddress(0x40);
  wait_ms(10);
  i2cGPIOexpanderConfigure(0x22, 0b00001111);
  sensor12.setAddress(0x41);
  wait_ms(10);

  sensor1.init();
  sensor2.init();
  sensor3.init();
  sensor4.init();
  sensor5.init();
  sensor6.init();
  sensor7.init();
  sensor8.init();
  sensor9.init();
  sensor10.init();
  sensor11.init();
  sensor12.init();

  sensor1.setTimeout(1);
  sensor2.setTimeout(1);
  sensor3.setTimeout(1);
  sensor4.setTimeout(1);
  sensor5.setTimeout(1);
  sensor6.setTimeout(1);
  sensor7.setTimeout(1);
  sensor8.setTimeout(1);
  sensor9.setTimeout(1);
  sensor10.setTimeout(1);
  sensor11.setTimeout(1);
  sensor12.setTimeout(1);

  sensor1.startContinuous(0);
  sensor2.startContinuous(0);
  sensor3.startContinuous(0);
  sensor4.startContinuous(0);
  sensor5.startContinuous(0);
  sensor6.startContinuous(0);
  sensor7.startContinuous(0);
  sensor8.startContinuous(0);
  sensor9.startContinuous(0);
  sensor10.startContinuous(0);
  sensor11.startContinuous(0);
  sensor12.startContinuous(0);

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  sensor2.setSignalRateLimit(0.1);
  sensor3.setSignalRateLimit(0.1);
  sensor4.setSignalRateLimit(0.1);
  sensor5.setSignalRateLimit(0.1);
  sensor6.setSignalRateLimit(0.1);
  sensor7.setSignalRateLimit(0.1);
  sensor8.setSignalRateLimit(0.1);
  sensor9.setSignalRateLimit(0.1);
  sensor10.setSignalRateLimit(0.1);
  sensor11.setSignalRateLimit(0.1);
  sensor12.setSignalRateLimit(0.1);
  
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor7.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor7.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor8.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor8.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor9.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor9.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor10.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor10.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor11.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor11.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor12.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor12.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor1.setMeasurementTimingBudget(10000);
  sensor2.setMeasurementTimingBudget(10000);
  sensor3.setMeasurementTimingBudget(10000);
  sensor4.setMeasurementTimingBudget(10000);
  sensor5.setMeasurementTimingBudget(10000);
  sensor6.setMeasurementTimingBudget(10000);
  sensor7.setMeasurementTimingBudget(10000);
  sensor8.setMeasurementTimingBudget(10000);
  sensor9.setMeasurementTimingBudget(10000);
  sensor10.setMeasurementTimingBudget(10000);
  sensor11.setMeasurementTimingBudget(10000);
  sensor12.setMeasurementTimingBudget(10000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
  sensor3.setMeasurementTimingBudget(200000);
  sensor4.setMeasurementTimingBudget(200000);
  sensor5.setMeasurementTimingBudget(200000);
  sensor6.setMeasurementTimingBudget(200000);
  sensor7.setMeasurementTimingBudget(200000);
  sensor8.setMeasurementTimingBudget(200000);
  sensor9.setMeasurementTimingBudget(200000);
  sensor10.setMeasurementTimingBudget(200000);
  sensor11.setMeasurementTimingBudget(200000);
  sensor12.setMeasurementTimingBudget(200000);
#endif

}

void singleSensorRead() //For debugging sensor with the first configured address.
{
  sensor1.init();
  sensor1.setTimeout(500);
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor1.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(200000);
#endif

  while (1)
    {
        serial_pc.printf("%u\r\n", sensor1.readRangeSingleMillimeters());
        if (sensor1.timeoutOccurred())
        {
            serial_pc.printf("TIMEOUT!\r\n");
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
  //SCAN i2c devices!
  //i2cScanner();  //DEBUG - scan i2c addresses.
  //configure gpio expander pins
  //i2cGPIOexpanderConfigure(0x22, 0b11110111);
  initSensors(); 
  //singleSensorRead();
  
  // MAIN LOOP
  while (true)
  {
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
//      // serial_pc.printf("Temp: %f: \r\n", m[i].getTemperature());
//      serial_pc.printf("Current[%d]: %f: \r\n", i, m[i].getCurrent());
    }

//    serial_pc.printf("Serial arrived: %d\r\n", serial_arrived);
    
    if (packet_received_b) // packet was completeted with \r \n
    {
      std::string packet(serial_buf);
      serial_buf[0] = '\0';
      serial_arrived = 0;
      processPacket(packet);
      packet_received_b = false;
    }
    // Update range sensors readings
    serial_pc.printf("RANGE:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u:%u\r\n", sensor1.readRangeContinuousMillimeters(), sensor2.readRangeContinuousMillimeters(), sensor3.readRangeContinuousMillimeters(), sensor4.readRangeContinuousMillimeters(), sensor5.readRangeContinuousMillimeters(), sensor6.readRangeContinuousMillimeters(), sensor7.readRangeContinuousMillimeters(), sensor8.readRangeContinuousMillimeters(), sensor9.readRangeContinuousMillimeters(), sensor10.readRangeContinuousMillimeters(), sensor11.readRangeContinuousMillimeters(), sensor12.readRangeContinuousMillimeters());
    // Update odometry
    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
    //serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(),
                     //odom_.getOriZ(), odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
    // Synchronize to given MAIN_DELTA_T
    wait_us(MAIN_DELTA_T*1000*1000 - main_timer.read_us());
  }
}
