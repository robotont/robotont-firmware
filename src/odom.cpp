#include "odom.h"
#include "MatrixMath/MatrixMath.h"

Odom::Odom(const MotorConfig& cfg0, const MotorConfig& cfg1, const MotorConfig& cfg2)
  : motor_configs_({ cfg0, cfg1, cfg2 }), odom_matrix_(3,3), odom_matrix_inv_(3,3)
{
  // add elements to odom matrix row by row
  for (int i = 0; i < 3; i++)
  {
    odom_matrix_ << -sin(motor_configs_[i].wheel_pos_phi);
    odom_matrix_ << cos(motor_configs_[i].wheel_pos_phi);
    odom_matrix_ << motor_configs_[i].wheel_pos_r;
  }
  odom_matrix_inv_ = MatrixMath::Inv(odom_matrix_);
  reset();
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
  float dt = motor_configs_[0].pid_dt;

  Matrix wheel_vel(3, 1);
  wheel_vel << vel_1 << vel_2 << vel_3;
  Matrix robot_vel = odom_matrix_inv_ * wheel_vel;

//  float delta_x = robot_vel(0, 0) * dt;
//  float delta_y = robot_vel(1, 0) * dt;
//  float delta_th = robot_vel(2, 0) * dt;

  lin_vel_x_ = robot_vel(0, 0) * cos(ori_z_) - robot_vel(1, 0) * sin(ori_z_);
  lin_vel_y_ = robot_vel(0, 0) * sin(ori_z_) + robot_vel(1, 0) * cos(ori_z_);
  ang_vel_z_ = robot_vel(2, 0);

  pos_x_ += lin_vel_x_ * dt;
  pos_y_ += lin_vel_y_ * dt;
  ori_z_ += ang_vel_z_ * dt;
}
