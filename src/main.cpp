#include "mbed.h"
#include "motor.h"
#include "odom.h"
#include "DS1820.h"
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
#define MAIN_DELTA_T 0.1

#define MAX_CMD_ARGS 5
#define MOTOR_COUNT 3
#define CMD_TIMEOUT_MS 0.5 // If velocity command is not received within this period all motors are stopped.

/*
 * Motor configurations
 */


// Motor 0 configuration
//#define M0_TEMP PC_0
struct MotorConfig cfg0 = { .pin_dir1 = PC_8,
                            .pin_dir2 = PB_8,
                            .pin_pwm = PC_6,
                            .pin_enca = PC_5,
                            .pin_encb = PB_9,
                            .pin_fault = NC,//PC_9,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = M_PI / 3.0f };

// Motor 1 configuration
#define M1_TEMP PC_1
struct MotorConfig cfg1 = { .pin_dir1 = PA_12,
                            .pin_dir2 = PA_6,
                            .pin_pwm = PA_11,
                            .pin_enca = PB_12,
                            .pin_encb = PA_7,
                            .pin_fault = NC,//PA_5,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = M_PI };

// Motor 2 configuration
#define M2_TEMP PB_0
struct MotorConfig cfg2 = { .pin_dir1 = PB_2,
                            .pin_dir2 = PA_8,
                            .pin_pwm = PB_1,
                            .pin_enca = PB_15,
                            .pin_encb = PB_10,
                            .pin_fault = NC,//PA_9,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = 5.0f /3.0f * M_PI };
/*
// Motor configurations for v0.6
struct MotorConfig cfg0_06 = { .pin_dir1 = PB_8,
                            .pin_dir2 = PC_8,
                            .pin_pwm = PC_6,
                            .pin_enca = PC_5,
                            .pin_encb = PB_9,
                            .pin_fault = PC_9,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = M_PI / 3.0f };

struct MotorConfig cfg1_06 = { .pin_dir1 = PA_6,
                            .pin_dir2 = PA_12,
                            .pin_pwm = PA_11,
                            .pin_enca = PB_12,
                            .pin_encb = PA_7,
                            .pin_fault = PA_5,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = M_PI };

struct MotorConfig cfg2_06 = { .pin_dir1 = PA_8,
                            .pin_dir2 = PB_2,
                            .pin_pwm = PB_1,
                            .pin_enca = PB_15,
                            .pin_encb = PB_10,
                            .pin_fault = PA_9,
                            .pin_feedback = NC,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = 5.0f /3.0f * M_PI };
*/


struct MotorConfig cfg0_v1_1 = {
  PC_8,          // pin_dir1
  PC_9,          // pin_dir2
  PC_6,          // pin_pwm
  PC_5,          // pin_enca
  PA_12,          // pin_encb
  PA_6,          // pin_fault
  PA_11,            // pin_feedback
  NC,            // pin_temp
  PID_KP,        // pid_k_p
  PID_TI,        // pid_tau_i
  PID_TD,        // pid_tau_d
  PID_DELTA_T,   // pid_dt
  ENC_CPR,       // enc_cpr
  GEAR_RATIO,    // gear_ratio
  WHEEL_RADIUS,  // wheel_radius
  WHEEL_POS_R,   // wheel_pos_r
  M_PI / 3.0f    // wheel_pos_phi
};

struct MotorConfig cfg1_v1_1 = {
  PB_12,         // pin_dir1
  PA_7,          // pin_dir2
  PB_6,          // pin_pwm
  PC_7,          // pin_enca
  PA_9,          // pin_encb
  PB_2,          // pin_fault
  PA_8,          // pin_feedback
  NC,            // pin_temp
  PID_KP,        // pid_k_p
  PID_TI,        // pid_tau_i
  PID_TD,        // pid_tau_d
  PID_DELTA_T,   // pid_dt
  ENC_CPR,       // enc_cpr
  GEAR_RATIO,    // gear_ratio
  WHEEL_RADIUS,  // wheel_radius
  WHEEL_POS_R,   // wheel_pos_r
  M_PI           // wheel_pos_phi
};

struct MotorConfig cfg2_v1_1 = {
  PB_15,              // pin_dir1
  PB_1,              // pin_dir2
  PB_14,               // pin_pwm
  PB_4,              // pin_enca
  PB_13,               // pin_encb
  PB_3,              // pin_fault
  PA_10,               // pin_feedback
  NC,                 // pin_temp
  PID_KP,             // pid_k_p
  PID_TI,             // pid_tau_i
  PID_TD,             // pid_tau_d
  PID_DELTA_T,        // pid_dt
  ENC_CPR,            // enc_cpr
  GEAR_RATIO,         // gear_ratio
  WHEEL_RADIUS,       // wheel_radius
  WHEEL_POS_R,        // wheel_pos_r
  5.0f / 3.0f * M_PI  // wheel_pos_phi
};



// Motor 0 configuration
//#define M0_TEMP PC_0
//const struct MotorConfig cfg0 = { .pin_dir1 = PC_8,
//                            .pin_dir2 = PC_9,
//                            .pin_pwm = PC_6,
//                            .pin_enca = PC_5,
//                            .pin_encb = PA_6,
//                            .pin_fault = PA_7,
//                            .pin_feedback = PB_12,
//                            .pin_temp = NC,
//                            .pid_k_p = PID_KP,
//                            .pid_tau_i = PID_TI,
//                            .pid_tau_d = PID_TD,
//                            .pid_dt = PID_DELTA_T,
//                            .enc_cpr = ENC_CPR,
//                            .gear_ratio = GEAR_RATIO,
//                            .wheel_radius = WHEEL_RADIUS,
//                            .wheel_pos_r = WHEEL_POS_R,
//                            .wheel_pos_phi = M_PI / 3.0f };
//
//// Motor 1 configuration
//#define M1_TEMP PC_1
//const struct MotorConfig cfg1 = { .pin_dir1 = PB_11,
//                            .pin_dir2 = PB_6,
//                            .pin_pwm = PC_7,
//                            .pin_enca = PA_9,
//                            .pin_encb = PB_2,
//                            .pin_fault = PA_8,
//                            .pin_feedback = NC,
//                            .pin_temp = NC,
//                            .pid_k_p = PID_KP,
//                            .pid_tau_i = PID_TI,
//                            .pid_tau_d = PID_TD,
//                            .pid_dt = PID_DELTA_T,
//                            .enc_cpr = ENC_CPR,
//                            .gear_ratio = GEAR_RATIO,
//                            .wheel_radius = WHEEL_RADIUS,
//                            .wheel_pos_r = WHEEL_POS_R,
//                            .wheel_pos_phi = M_PI };
//
//// Motor 2 configuration
//#define M2_TEMP PB_0
//const struct MotorConfig cfg2 = { .pin_dir1 = PB_15,
//                            .pin_dir2 = PB_10,
//                            .pin_pwm = PB_4,
//                            .pin_enca = PB_14,
//                            .pin_encb = PB_5,
//                            .pin_fault = PB_13,
//                            .pin_feedback = NC,
//                            .pin_temp = NC,
//                            .pid_k_p = PID_KP,
//                            .pid_tau_i = PID_TI,
//                            .pid_tau_d = PID_TD,
//                            .pid_dt = PID_DELTA_T,
//                            .enc_cpr = ENC_CPR,
//                            .gear_ratio = GEAR_RATIO,
//                            .wheel_radius = WHEEL_RADIUS,
//                            .wheel_pos_r = WHEEL_POS_R,
//                            .wheel_pos_phi = 5.0f /3.0f * M_PI };


// Initialize motors
//Motor m[] = { { cfg0_06 }, { cfg1_06 }, { cfg2_06 } };
Motor m[] = { { cfg0_v1_1 }, { cfg1_v1_1 }, { cfg2_v1_1 } };
//Motor m[] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Initialize odometry
Odom odom_(cfg0_v1_1, cfg1_v1_1, cfg2_v1_1, MAIN_DELTA_T);
//Odom odom_(cfg0, cfg1, cfg2, MAIN_DELTA_T);

// Timeout
Timer cmd_timer;
Ticker cmd_timeout_checker;

// Variables for serial connection
RawSerial serial_pc(USBTX, USBRX);  // tx, rx
char serial_buf[256];        // Buffer for incoming serial data
volatile uint8_t serial_arrived = 0;  // Number of bytes arrived

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
    if (serial_arrived > 254)
    {
      serial_arrived = 0;
    }

    if (c == '\n' || c == '\r')  // command terminated
    {
      if (serial_arrived > 3)
      {
        // the packet is complete, let's process it.
        std::string packet(serial_buf);
        processPacket(packet);
      }

      serial_buf[0] = '\0';
      serial_arrived = 0;
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

//  Timer t;

  // MAIN LOOP
  while (true)
  {
 //   t.reset();
 //   t.start();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
      // MOTOR DEBUG
      //      serial_pc.printf("\r\n");
            serial_pc.printf("MOTOR %d: \r\n", i);
       serial_pc.printf("Speed[%d]: %f (%f): \r\n", i, m[i].getMeasuredSpeed(),
                        m[i].getSpeedSetPoint());
//            serial_pc.printf("Effort: %f: \r\n", m[i].getEffort());
            serial_pc.printf("Fault: %u: \r\n", m[i].getFaultPulseCount());
      //  serial_pc.printf("Temp: %f: \r\n", m[i].getTemperature());
            serial_pc.printf("Current[%d]: %f: \r\n", i, m[i].getCurrent());
    }

    //serial_pc.printf("Serial arrived: %d\r\n", serial_arrived);

    odom_.update(m[0].getMeasuredSpeed(), m[1].getMeasuredSpeed(), m[2].getMeasuredSpeed());
//    serial_pc.printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", odom_.getPosX(), odom_.getPosY(),
//                     odom_.getOriZ(), odom_.getLinVelX(), odom_.getLinVelY(), odom_.getAngVelZ());
 //   t.stop();
  //  serial_pc.printf("The time taken was %f seconds\n", t.read());
    wait(MAIN_DELTA_T);
  }
}
