#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include <sstream>
#include <vector>

#include "PID.h"

#define ENC_CPR 64
#define GEAR_RATIO 18.75
#define WHEEL_RADIUS 0.035
#define WHEEL_POS_R 0.145
#define PID_KP 0.8
#define PID_TI 0.05
#define PID_TD 0.0
#define PID_DELTA_T 0.01 / 2
#define MAIN_DELTA_T 0.02 / 2

#define MAX_CMD_ARGS 5
#define MOTOR_COUNT 3
#define CMD_TIMEOUT_MS 1000

// #define ENABLE_PID_Z
#define ENABLE_PID_X
// #define ENABLE_PID_Y

#include "motor_config_v2_1.h"

Motor m[] = {{cfg0}, {cfg1}, {cfg2}};

Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);
Odom odom_expected_(cfg0, cfg1, cfg2, MAIN_DELTA_T); // !

Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;

RawSerial serial_pc(USBTX, USBRX);
char serial_buf[256];
volatile uint8_t serial_arrived = 0;
volatile bool packet_received_b = false;

std::vector<std::string> cmd;

// ! Allocating expected values
volatile float expected_speeds_m[3] = {0, 0, 0};
float RS_lin_speed_x;
float RS_lin_speed_y;
float RS_lin_speed_dir;
float RS_lin_speed_mag;
float RS_angular_speed_z;
// !===========================

void pc_rx_callback()
{
  while (serial_pc.readable())
  {
    char c = serial_pc.getc();
    serial_buf[serial_arrived++] = c;
    serial_buf[serial_arrived] = '\0';
    if (serial_arrived > 254)
    {
      serial_arrived = 0;
    }

    if (c == '\n' || c == '\r')
    {
      if (serial_arrived > 3)
      {
        packet_received_b = true;
      }
    }

    if (c == 27)
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
      // TODO stop odom expected (it's done inside process packet, but need test here as well)
    }
  }
}

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

  if (cmd[0] == "MS")
  {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      float speed_setpoint = std::atof(cmd[i + 1].c_str());
      m[i].setSpeedSetPoint(speed_setpoint);
    }
    cmd_timer.reset();
  }

  else if (cmd[0] == "RS")
  {
    RS_lin_speed_x = std::atof(cmd[1].c_str());
    RS_lin_speed_y = std::atof(cmd[2].c_str());
    RS_angular_speed_z = std::atof(cmd[3].c_str());

    // ! Assigning expected values
    RS_lin_speed_dir = atan2(RS_lin_speed_y, RS_lin_speed_x);
    RS_lin_speed_mag = sqrt(RS_lin_speed_x * RS_lin_speed_x + RS_lin_speed_y * RS_lin_speed_y);
    // !==========================

    cmd_timer.reset();
  }
}

int main()
{
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  serial_pc.attach(&pc_rx_callback);
  serial_pc.printf("**** MAIN ****\r\n");

  cmd_timeout_checker.attach(check_for_timeout, 0.1);
  cmd_timer.start();

#ifdef ENABLE_PID_Z
  PID pid_angle(PID_KP * 300, 0, 0, MAIN_DELTA_T);
  pid_angle.setInputLimits(-10.0f, 10.0f); // Todo increase window size (10 rad ~ 2.5 square) (2nd prior)
  pid_angle.setOutputLimits(-1.0f, 1.0f);
  pid_angle.setBias(0.0);
  pid_angle.setMode(1);
#endif

#ifdef ENABLE_PID_X
  PID pid_speed_x(PID_KP * 5, 0, 0, MAIN_DELTA_T);
  pid_speed_x.setInputLimits(-1.0f, 1.0f); // ! TODO test appropriate range?
  pid_speed_x.setOutputLimits(-1.0f, 1.0f);
  pid_speed_x.setBias(0.0);
  pid_speed_x.setMode(1);
#endif

#ifdef ENABLE_PID_Y
  PID pid_speed_y(PID_KP * 10, 0, 0, MAIN_DELTA_T);
  pid_speed_y.setInputLimits(-1.0f, 1.0f); // ! TODO test appropriate range?
  pid_speed_y.setOutputLimits(-1.0f, 1.0f);
  pid_speed_y.setBias(0.0);
  pid_speed_y.setMode(1);
#endif

  float robot_angular_speed_z;
  float robot_lin_speed_x;
  float robot_lin_speed_y;
  float robot_lin_speed_mag;
  float robot_lin_speed_dir;

  float counter = 0;
  while (true)
  {
    odom_expected_.update(expected_speeds_m[0], expected_speeds_m[1], expected_speeds_m[2]);
    serial_pc.printf("ODOM_EXPECTED:%f:%f:%f:%f:%f:%f\r\n",
                     odom_expected_.getPosX(), odom_expected_.getPosY(), odom_expected_.getOriZ(),
                     odom_expected_.getLinVelX(), odom_expected_.getLinVelY(), odom_expected_.getAngVelZ());
    // TODO 3rd prior (motor encoder freq)

    main_timer.reset();
    main_timer.start();

#ifdef ENABLE_PID_Z
    pid_angle.setSetPoint(odom_expected_.getOriZ());
    pid_angle.setProcessValue(odom_.getOriZ());
    robot_angular_speed_z = pid_angle.compute();
#else
    robot_angular_speed_z = RS_angular_speed_z;
#endif

#ifdef ENABLE_PID_X
    pid_speed_x.setSetPoint(odom_expected_.getLinVelX());
    pid_speed_x.setProcessValue(odom_.getLinVelX());
    robot_lin_speed_x = pid_speed_x.compute();
#else
    robot_lin_speed_x = RS_lin_speed_x;
#endif

#ifdef ENABLE_PID_Y
    pid_speed_y.setSetPoint(odom_expected_.getLinVelY());
    pid_speed_y.setProcessValue(odom_.getLinVelY());
    robot_lin_speed_y = pid_speed_y.compute();
#else
    robot_lin_speed_y = RS_lin_speed_y;
#endif

    robot_lin_speed_dir = atan2(robot_lin_speed_y, robot_lin_speed_x);
    robot_lin_speed_mag = sqrt(robot_lin_speed_x * robot_lin_speed_x + robot_lin_speed_y * robot_lin_speed_y);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      // ! Updating expected values
      float expected_speed = RS_lin_speed_mag * sin(RS_lin_speed_dir - m[i].getWheelPosPhi()) +
                             m[i].getWheelPosR() * RS_angular_speed_z;
      float speed = robot_lin_speed_mag * sin(robot_lin_speed_dir - m[i].getWheelPosPhi()) +
                    m[i].getWheelPosR() * robot_angular_speed_z;
      // ! ========================
      if (abs(speed) < 1e-3)
      {
        m[i].stop();
      }
      else
      {
        m[i].setSpeedSetPoint(speed);
        expected_speeds_m[i] = expected_speed;
      }
      if (speed < 1e-3)
      {
        expected_speeds_m[i] = 0;
      }
    }

    if (packet_received_b)
    {
      std::string packet(serial_buf);
      serial_buf[0] = '\0';
      serial_arrived = 0;
      processPacket(packet);
      packet_received_b = false;
    }

    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
    serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n",
                     odom_.getPosX(), odom_.getPosY(), odom_.getOriZ(),
                     odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());

    // serial_pc.printf("DEBUG_OUT:%f:%f:%f\r\n", robot_lin_speed_x, RS_lin_speed_x, counter++);

    wait_us(MAIN_DELTA_T * 1000 * 1000 - main_timer.read_us());
  }
}
