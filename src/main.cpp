#include "mbed.h"
#include "motor.h"
#include "DS1820.h"
#include <sstream>

// Common parameters for all motors
#define ENC_CPR 64
#define GEAR_RATIO 18.75
#define WHEEL_RADIUS 0.035
#define WHEEL_POS_R 0.127
#define PID_KP 0.8
#define PID_TI 0.05
#define PID_TD 0.0
#define PID_DELTA_T 0.01

// Motor 0 configuration
#define M0_TEMP PC_0
struct MotorConfig cfg0 = { .pin_dir1 = PA_12,
                            .pin_dir2 = PA_6,
                            .pin_pwm = PA_11,
                            .pin_enca = PB_12,
                            .pin_encb = PA_7,
                            .pin_fault = PA_5,
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
struct MotorConfig cfg1 = { .pin_dir1 = PB_2,
                            .pin_dir2 = PA_8,
                            .pin_pwm = PB_1,
                            .pin_enca = PB_15,
                            .pin_encb = PB_10,
                            .pin_fault = PA_9,
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
struct MotorConfig cfg2 = { .pin_dir1 = PC_8,
                            .pin_dir2 = PB_8,
                            .pin_pwm = PC_6,
                            .pin_enca = PC_5,
                            .pin_encb = PB_9,
                            .pin_fault = PC_9,
                            .pin_temp = NC,
                            .pid_k_p = PID_KP,
                            .pid_tau_i = PID_TI,
                            .pid_tau_d = PID_TD,
                            .pid_dt = PID_DELTA_T,
                            .enc_cpr = ENC_CPR,
                            .gear_ratio = GEAR_RATIO,
                            .wheel_radius = WHEEL_RADIUS,
                            .wheel_pos_r = WHEEL_POS_R,
                            .wheel_pos_phi = 5.0f /6.0f * M_PI };

// Initialize motors
Motor m[3] = { { cfg0 }, { cfg1 }, { cfg2 } };

// Variables for serial connection
Serial serial_pc(USBTX, USBRX);  // tx, rx
char serial_buf[256];        // Buffer for incoming serial data
uint8_t serial_arrived = 0;  // Number of bytes arrived

// For parsing floating point arguments received over serial
volatile float arg[3];

// For parsing the command string received over serial
std::string cmd;

// This method processes a complete serial packet
void processPacket(const std::string& packet)
{
  // Parse the command
  std::istringstream ss(packet);
  std::getline(ss, cmd, ':');
//  serial_pc.printf("Got Command %s\r\n", cmd.c_str());

  // MS - Set motor speeds manually (linear speed on wheel m/s)
  /* MS:motor1_speed:motor2_speed:motor3_speed */
  if (cmd == "MS")
  {
    sscanf(ss.str().c_str(), "%f:%f:%f", &arg[0], &arg[1], &arg[2]);
    for (uint8_t i = 0; i < 3; i++)
    {
      m[i].setSpeedSetPoint(arg[i]);
    }
  }

  // RS - Set motor speeds based on robot velocities. We use ROS coordinate convention: x-forward, y-left, theta-CCW rotation.
  /* RS:robot_speed_x(m/s):robot_speed_y(m/s):robot_speed_theta(rad/s) */
  else if (cmd == "RS")
  {
    float robot_speed_x = 0.0f;
    float robot_speed_y = 0.0f;
    float robot_speed_theta = 0.0f;
    sscanf(ss.str().c_str(), "%f:%f:%f", &robot_speed_x, &robot_speed_y, &robot_speed_theta);

    //TODO calculate motor speeds from x,y,theta
    arg[0]=0.0f;
    arg[1]=0.0f;
    arg[2]=0.0f;

    for (uint8_t i = 0; i < 3; i++)
    {
      m[i].setSpeedSetPoint(arg[i]);
    }
    
  }
  else if (cmd == "PID") // Update PID parameters
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
      for (uint8_t i = 0; i < 3; i++)
      {
        m[i].stop();
        arg[i] = 0.0f;
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

  // MAIN LOOP
  while (true)
  {

    for (int i = 0; i < 3; i++)
    {
      serial_pc.printf("\r\n");
      serial_pc.printf("MOTOR %d: \r\n", i);
      serial_pc.printf("Speed: %f (%f): \r\n", m[i].getMeasuredSpeed(), m[i].getSpeedSetPoint());
      serial_pc.printf("Effort: %f: \r\n", m[i].getEffort());
    //  serial_pc.printf("Temp: %f: \r\n", m[i].getTemperature());
    }
    wait(0.1);
  }
}
