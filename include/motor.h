#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "DS1820.h"


struct MotorConfig
{
  PinName pin_dir1;
  PinName pin_dir2;
  PinName pin_pwm;
  PinName pin_enca;
  PinName pin_encb;
  PinName pin_fault;

  PinName pin_temp;

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

enum MotorStatus {STATUS_UNINITIALIZED, STATUS_OK, STATUS_STOPPED};

class Motor
{
  public:
    Motor(const MotorConfig& cfg);
    ~Motor();
    //Motor(const MotorConfig& cfg);
    //void initialize(const MotorConfig& cfg);

    void stop();
    /* Linear speed of the wheel */
    void setSpeedSetPoint(float speed);
    void setSpeedLimit(float speed_limit);

    // Limit motor power (duty cycle) e.g. power=0.75 limits max power to 75%
    void setEffortLimit(float effort_limit);
    void setPIDTunings(float k_p, float tau_i, float tau_d);

    void processPID();

    float getMeasuredSpeed() const {return speed_measured_;};
    float getSpeedSetPoint() const {return speed_setpoint_;};
    float getEffort() const {return effort_;};
    float getWheelPosR() const {return config_.wheel_pos_r;};
    float getWheelPosPhi() const {return config_.wheel_pos_phi;};
    float getTemperature();

  private:
    /* Set PWM duty cycle and polarity (direction). Effort is in range [0...1] */
    void setEffort(float effort);

    DigitalOut dir1_, dir2_;
    PwmOut pwm_;
    QEI enc_;
    DigitalIn fault_;
    PID pid_;
    Ticker pidTicker_;
    DS1820* temp_sensor_;
    MotorStatus status_;

    float speed_setpoint_; // target wheel velocity in rad/s
    float speed_measured_; // actual wheel velocity in rad/s
    float speed_limit_; // wheel speed limit rad/s
    float effort_; // actual wheel velocoty in rad/s
    float effort_limit_; // pwm duty cycle limit [0...1]
    bool stopped_;
    float pulse_to_speed_ratio_;
    MotorConfig config_;
};

#endif
