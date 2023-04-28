#ifndef MY_PID_H
#define MY_PID_H

class MY_PID
{
public:
    MY_PID(float K, float I, float D, float dt);
    ~MY_PID();

    void set_target_value(float target_value);
    void set_real_value(float target_value);
    float calculate_output();

private:
    float K_;
    float I_;
    float D_;
    float dt_;

    float target_value_;
    float real_value_;

    float prev_error_;
    float error_integral_;
};

#endif