#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "moving_average.h"


/**
 * @brief A configuration for the Motor
 */
struct MotorConfig
{
  PinName pin_dir1; 
  PinName pin_dir2;
  PinName pin_pwm;
  PinName pin_enca;
  PinName pin_encb;
  PinName pin_fault;
  PinName pin_feedback;

  //Default PID parameters
  float pid_k_p;
  float pid_tau_i;
  float pid_tau_d;
  float pid_dt;

  float enc_cpr; // encoder counts per revolution
  float gear_ratio; // gearbox reduction ratio
  float wheel_radius; // wheel outer radius

  // wheel position in polar coordinates
  float wheel_pos_r; // distance from center
  float wheel_pos_phi; // angle relative to x-axis (forward)
};

/**
 * \todo Needs attention
 */
enum MotorStatus {STATUS_UNINITIALIZED, STATUS_OK, STATUS_STOPPED};

/**
 * @brief A class representing the Motor of a robot
 */
class Motor
{
public:
  /**
   * @brief Constructor
   *
   * @param cfg Motor configuration
   */
  Motor(const MotorConfig& cfg);

  /**
   * @brief Destructor
   */
  ~Motor();

  /**
   * @brief Stops the motor
   */
  void stop();

  /**
   * @brief Set the desired linear speed for the wheel
   *
   * @param speed Linear speed of the wheel
   */
  void setSpeedSetPoint(float speed);

  /**
   * @brief Set limits for wheel speed
   *
   * @param speed_limit Angular speed in rad/s
   */
  void setSpeedLimit(float speed_limit);

  /**
   * @brief Limits motor power (duty cycle)
   *
   * @param effort_limit Effort [0..1]
   */
  void setEffortLimit(float effort_limit);

  /**
   * @brief Set PID parameters
   *
   * @param k_p Proportional term
   * @param tau_i Integral term
   * @param tau_d Derivative term
   */
  void setPIDTunings(float k_p, float tau_i, float tau_d);

  /**
   * @brief Callback function for updating the motor.
   * This function is used to periodically update the speed, consumed current, and the internal
   * state of the PID controller.
   */
  void update();

  /**
   * @brief Callback function for current feedback measurement
   */
  void onCurrentPulse(void);
  void onFaultPulse(void);

  float getMeasuredSpeed() const
  {
    return speed_measured_;
  };
  float getSpeedSetPoint() const
  {
    return speed_setpoint_;
  };
  float getEffort() const
  {
    return effort_;
  };
  float getWheelPosR() const
  {
    return config_.wheel_pos_r;
  };
  float getWheelPosPhi() const
  {
    return config_.wheel_pos_phi;
  };
  float getCurrent()
  {
    return current_measured_.GetAverage();
  };
  unsigned int getFaultPulseCount() const
  {
    return fault_pulse_count_;
  }

private:
  /* Set PWM duty cycle and polarity (direction). Effort is in range [0...1] */
  void setEffort(float effort);

  DigitalOut dir1_, dir2_;
  PwmOut pwm_;
  QEI enc_;
  InterruptIn fault_;
  PID pid_;
  Ticker pidTicker_;
  InterruptIn* current_feedback_;
  volatile uint32_t current_pulse_count_;
  MotorStatus status_;
  unsigned int fault_pulse_count_;

  float speed_setpoint_;                   // target wheel velocity in [rad/s]
  float speed_measured_;                   // actual wheel velocity [rad/s]
  MovingAverage<float> current_measured_;  // measured current going to motor [A]
  float speed_limit_;                      // wheel speed limit [rad/s]
  float effort_;                           // pwm duty cycle
  float effort_limit_;                     // pwm duty cycle limit [0...1]
  bool stopped_;
  float pulse_to_speed_ratio_;
  MotorConfig config_;
};

#endif
