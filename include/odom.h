#ifndef ODOM_H
#define ODOM_H

#include "mbed.h"
#include "Eigen/Dense.h"
#include "motor.h"


class Odom
{
  public:
    Odom(MotorConfig& cfg0, MotorConfig& cfg1, MotorConfig& cfg2);
    ~Odom();

    void reset();
    void update(float vel_1, float vel_2, float m2_vel);

    float getPosX() const {return pos_x_;};
    float getPosY() const {return pos_y_;};
    float getOriZ() const {return ori_z_;};
    float getLinVelX() const {return lin_vel_x_;};
    float getLinVelY() const {return lin_vel_y_;};
    float getAngVelZ() const {return ang_vel_z_;};

  private:
    float pos_x_;
    float pos_y_;
    float ori_z_;
    float lin_vel_x_;
    float lin_vel_y_;
    float ang_vel_z_;

    MotorConfig motor_configs_[3];

    Eigen::Matrix3f odom_matrix_;
    Eigen::Matrix3f odom_matrix_inv_;

};

#endif // ODOM_H