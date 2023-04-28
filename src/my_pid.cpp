#include "my_pid.h"

MY_PID::MY_PID(float K, float I, float D, float dt)
{
    K_ = K;
    I_ = I;
    D_ = D;
    dt_ = dt;

    target_value_ = 0;
    real_value_ = 0;
    error_integral_ = 0;
}

MY_PID::~MY_PID() {}

void MY_PID::set_target_value(float target_value)
{
    target_value_ = target_value;
}

void MY_PID::set_real_value(float real_value)
{
    real_value_ = real_value;
}

float MY_PID::calculate_output()
{
    float error = target_value_ - real_value_;
    error_integral_ += error * dt_;
    float error_derivative = error - prev_error_ * dt_;

    float Pout = K_ * error;
    float Iout = I_ * error_integral_;
    float Dout = D_ * error_derivative;

    return Pout + Iout + Dout;
}