#include "odom.h"

Odom::Odom(MotorConfig& cfg0, MotorConfig& cfg1, MotorConfig& cfg2)
  : motor_configs_({ cfg0, cfg1, cfg2 })
{
  for (int i = 0; i < 3; i++)
  {
    odom_matrix_.row(i) << -sin(motor_configs_[i].wheel_pos_phi),
        cos(motor_configs_[i].wheel_pos_phi), motor_configs_[i].wheel_pos_r;
  }
  odom_matrix_inv_ = odom_matrix_.inverse();
}

Odom::~Odom()
{

}

void Odom::reset()
{
  pos_x_ = 0;
  pos_y_ = 0;
  ori_z_ = 0;
  lin_vel_x_ = 0;
  lin_vel_y_ = 0;
  ang_vel_z_ = 0;
}

void Odom::update(float vel_1, float vel_2, float vel_3)
{
  float dt = 0.01;

  Eigen::Vector3f wheel_vel(vel_1, vel_2, vel_3);
  Eigen::Vector3f robot_vel = odom_matrix_inv_ * wheel_vel;

  float delta_x = robot_vel(0) * dt;
  float delta_y = robot_vel(1) * dt;
  float delta_th = robot_vel(2) * dt;

  lin_vel_x_ = robot_vel(0) * cos(ori_z_) - robot_vel(1) * sin(ori_z_);
  lin_vel_y_ = robot_vel(0) * sin(ori_z_) + robot_vel(1) * cos(ori_z_);
  ang_vel_z_ = robot_vel(2);

  pos_x_ += lin_vel_x_ * dt;
  pos_y_ += lin_vel_y_ * dt;
  ori_z_ += ang_vel_z_ * dt;
}
