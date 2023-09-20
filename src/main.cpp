/*
Ticker timer;
const int FREQUENCY = 50;
const float PERIOD = 1.0 / FREQUENCY;

void timer_interrupt()
{
}

int main()
{
  timer.attach(&timer_interrupt, PERIOD);
}
*/
#include "main.h"

#include <vector>
#include <algorithm>

#include "mbed.h"
#include "motor.h"
#include "motor_config_v2_1.h"
#include "odom.h"
#include "PID.h"
#include "moving_average.h"
#include "my_pid.h"

Motor m[] = {{cfg0}, {cfg1}, {cfg2}};

Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);
Odom odom_expected_(cfg0, cfg1, cfg2, MAIN_DELTA_T);

Timer cmd_timer, main_timer;
Ticker cmd_timeout_checker;

RawSerial serial_pc(USBTX, USBRX);
char serial_buf[256];
volatile uint8_t serial_arrived = 0;
volatile bool packet_received_b = false;

std::vector<std::string> cmd;

// ! Allocating expected values
volatile float expected_speeds_m[3] = {0, 0, 0};
float RS_lin_speed_x = 0;
float RS_lin_speed_y = 0;
float RS_lin_speed_dir = 0;
float RS_lin_speed_mag = 0;
float RS_angular_speed_z = 0;
// !===========================

MovingAverage<float> odom_avg_z(64, 0.0);

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
      RS_lin_speed_x = 0;
      RS_lin_speed_y = 0;
      RS_angular_speed_z = 0;
    }
    cmd_timer.reset();
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

    RS_lin_speed_dir = atan2(RS_lin_speed_y, RS_lin_speed_x);
    RS_lin_speed_mag = sqrt(RS_lin_speed_x * RS_lin_speed_x + RS_lin_speed_y * RS_lin_speed_y);

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

  MY_PID pid_z(1, 0.3, 0.0, MAIN_DELTA_T);
  MY_PID pid_x(0.1, 0.3, 0.0, MAIN_DELTA_T);
  MY_PID pid_y(0.1, 0.3, 0.0, MAIN_DELTA_T);

  float robot_angular_speed_z;
  float robot_lin_speed_x;
  float robot_lin_speed_y;
  float robot_lin_speed_mag;
  float robot_lin_speed_dir;

  while (true)
  {
    main_timer.reset();
    main_timer.start();

    // TODO
    // ! PID tuning seletus, filter(?)
    // ! BAAS LAHENDUS - KIIRUSE KONTROLL (X, Y, Z)
    // ! LISA LAHENDUS - POS KONTROLL ("kas yldse vaja on?")
    // pid_z.set_target_value(odom_expected_.getOriZ()); // TODO -pi (+-2?)pi, sii ei tee midagi, muidu MODULO ()
    // pid_z.set_real_value(odom_.getOriZ());
    // robot_angular_speed_z = pid_z.calculate_output();
    // serial_pc.printf("DEBUG_OUT:%f:%f:%f\r\n", odom_expected_.getOriZ(), odom_.getOriZ(), robot_angular_speed_z);
    robot_angular_speed_z = RS_angular_speed_z;

    pid_x.set_target_value(odom_expected_.getPosX());
    pid_x.set_real_value(odom_.getPosX());
    robot_lin_speed_x = pid_x.calculate_output();
    serial_pc.printf("DEBUG_OUT:%f:%f:%f\r\n", odom_expected_.getPosX(), odom_.getPosX(), robot_lin_speed_x);
    // robot_lin_speed_x = RS_lin_speed_x;

    // pid_y.set_target_value(odom_expected_.getPosY());
    // pid_y.set_real_value(odom_.getPosY());
    // robot_lin_speed_y = pid_y.calculate_output();
    // serial_pc.printf("DEBUG_OUT:%f:%f:%f\r\n", odom_expected_.getPosY(), odom_.getPosY(), robot_lin_speed_y);
    robot_lin_speed_y = RS_lin_speed_y;

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
      expected_speeds_m[i] = expected_speed;

      if (abs(speed) < 1e-3)
      {
        m[i].stop();
      }
      else
      {
        m[i].setSpeedSetPoint(speed);
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

    odom_expected_.update(expected_speeds_m[0], expected_speeds_m[1], expected_speeds_m[2]);
    serial_pc.printf("ODOM_EXPECTED:%f:%f:%f:%f:%f:%f\r\n",
                     odom_expected_.getPosX(), odom_expected_.getPosY(), odom_expected_.getOriZ(),
                     odom_expected_.getLinVelX(), odom_expected_.getLinVelY(), odom_expected_.getAngVelZ());

    wait_us(std::max(MAIN_DELTA_T * 1000 * 1000 - main_timer.read_us(), 0.0));
  }
}
