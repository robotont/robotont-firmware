#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include <sstream>
#include <vector>

#include "PID.h"

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

// ! TODO aeg PID t66tlemiseks

// Initialize motors
Motor m[] = {{cfg0}, {cfg1}, {cfg2}};

// Initialize odometry
Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);
Odom odom_expected_(cfg0, cfg1, cfg2, MAIN_DELTA_T); // !

// !
float expected_speeds_m[3] = {0, 0, 0};

// float pid_angular_speed_z = 0;
float lin_speed_dir;
float lin_speed_mag; // TODO

// Timeout
Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;

// Variables for serial connection
RawSerial serial_pc(USBTX, USBRX);   // tx, rx
char serial_buf[256];                // Buffer for incoming serial data
volatile uint8_t serial_arrived = 0; // Number of bytes arrived
volatile bool packet_received_b = false;

// For parsing command with arguments received over serial
std::vector<std::string> cmd;

// This method processes a received serial packet
void processPacket(const std::string &packet)
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

    lin_speed_dir = atan2(lin_speed_y, lin_speed_x);
    lin_speed_mag = sqrt(lin_speed_x * lin_speed_x + lin_speed_y * lin_speed_y);

    cmd_timer.reset();
  }
  else if (cmd[0] == "DEBUG_IN")
  {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
    float e = 0.0f;
    sscanf(ss.str().c_str(), "%f:%f:%f:%f:%f", &a, &b, &c, &d, &e); // MAX_CMD_ARGS = 5
    // TODO process debug data
  }
  /*
  else if (cmd[0] == "PID") // Update PID parameters
  {
    float k_p = 0.0f;
    float tau_i = 0.0f;
    float tau_d = 0.0f;
    sscanf(ss.str().c_str(), "%f:%f:%f", &k_p, &tau_i, &tau_d);
    for (uint8_t i = 0; i < 3; i++)
    {
     m[i].setPIDTunings(k_p, tau_i, tau_d);
    }
  }
  */
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

    if (c == '\n' || c == '\r') // command terminated
    {
      if (serial_arrived > 3)
      {
        // signal that the packet is complete for processing
        packet_received_b = true;
      }
    }

    // if escape is received, clear the buffer and stop the motors for now
    if (c == 27) // esc
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
      // TODO stop odom expected
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

  // ! PID for angle
  PID pid_angle(PID_KP * 10, 0, 0, MAIN_DELTA_T);
  pid_angle.setInputLimits(-10.0f, 10.0f);
  pid_angle.setOutputLimits(-1.0f, 1.0f);
  pid_angle.setBias(0.0);
  pid_angle.setMode(1);

  // MAIN LOOP
  int counter = 0;
  while (true)
  {
    serial_pc.printf("DEBUG_OUT:%d:\r\n", ++counter);

    odom_expected_.update(expected_speeds_m[0], expected_speeds_m[1], expected_speeds_m[2]);
    serial_pc.printf("ODOM_EXPECTED:%f:%f:%f:%f:%f:%f\r\n",
                     odom_expected_.getPosX(), odom_expected_.getPosY(), odom_expected_.getOriZ(),
                     odom_expected_.getLinVelX(), odom_expected_.getLinVelY(), odom_expected_.getAngVelZ());

    // ! TODO pid for X and Y

    main_timer.reset();
    main_timer.start();
    pid_angle.setSetPoint(odom_expected_.getOriZ());
    // getAngVelZ() pid input todo
    pid_angle.setProcessValue(odom_.getOriZ());
    float pid_angular_speed_z = pid_angle.compute();

    // serial_pc.printf("ORIZ:%f %f %f\n", odom_.getOriZ(), odom_expected_.getOriZ(), pid_angular_speed_z);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      // TODO motor speed to the odom expected
      float speed = lin_speed_mag * sin(lin_speed_dir - m[i].getWheelPosPhi()) +
                    m[i].getWheelPosR() * pid_angular_speed_z;
      if (abs(speed) < 1e-3)
      {
        m[i].stop();
      }
      else
      {
        m[i].setSpeedSetPoint(speed);
        expected_speeds_m[i] = speed; // !
      }
      if (speed < 1e-3)
      {
        expected_speeds_m[i] = 0; // !
      }
    }

    // packet was completeted with \r \n
    if (packet_received_b)
    {
      std::string packet(serial_buf);
      serial_buf[0] = '\0';
      serial_arrived = 0;
      processPacket(packet);
      packet_received_b = false;
    }

    // Update odometry
    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());

    serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n",
                     odom_.getPosX(), odom_.getPosY(), odom_.getOriZ(),
                     odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
    // Synchronize to given MAIN_DELTA_T
    wait_us(MAIN_DELTA_T * 1000 * 1000 - main_timer.read_us());
  }
}
