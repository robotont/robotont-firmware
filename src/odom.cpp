#include "odom.h"
#include "MatrixMath.h"

Odom::Odom(const MotorConfig& cfg0, const MotorConfig& cfg1, const MotorConfig& cfg2,
           float delta_t)
  : motor_configs_({ cfg0, cfg1, cfg2 })
  , wheel_vel_(3, 1)
  , robot_vel_(3, 1)
  , odom_vel_(3, 1)
  , odom_pos_(3, 1)
  , odom_matrix_(3, 3)
  , odom_matrix_inv_(3, 3)
  , delta_t_(delta_t)
{
  // add elements to odom matrix row by row
  for (int i = 0; i < 3; i++)
  {
    odom_matrix_ << -sin(motor_configs_[i].wheel_pos_phi) << cos(motor_configs_[i].wheel_pos_phi)
                 << motor_configs_[i].wheel_pos_r;
  }
  
  ///TODO: Check if determinant is zero and report somehow
  //MBED_ASSERT(MatrixMath::det(odom_matrix_));
  
  // calculate the inverse of odom
  odom_matrix_inv_ = MatrixMath::Inv(odom_matrix_);

  // initialize vectors with zeros
  reset();
}

Odom::~Odom()
{

}

void Odom::reset()
{
  odom_pos_.Clear();
  odom_vel_.Clear();
}

void Odom::update(float vel_1, float vel_2, float vel_3)
{
  wheel_vel_(1,1) = vel_1;
  wheel_vel_(2,1) = vel_2;
  wheel_vel_(3,1) = vel_3;

  robot_vel_ = odom_matrix_inv_ * wheel_vel_;

  
  // transform velocities from robot frame to odom frame
  odom_vel_(1,1) = robot_vel_(1, 1) * cos(odom_pos_(3,1)) - robot_vel_(2, 1) * sin(odom_pos_(3,1));
  odom_vel_(2,1) = robot_vel_(1, 1) * sin(odom_pos_(3,1)) + robot_vel_(2, 1) * cos(odom_pos_(3,1));
  odom_vel_(3,1) = robot_vel_(3, 1);

  // position integration
  odom_pos_ += odom_vel_ * delta_t_;
}

void Odom::print()
{
  // DEBUG
    printf("\r\nodom_matrix:\r\n");
    odom_matrix_.print();
    printf("\r\n");
  
    printf("\r\nodom inverse:\r\n");
    odom_matrix_inv_.print();
    printf("\r\n");
  
    printf("\r\nwheel_vel:\r\n");
    wheel_vel_.print();
    printf("\r\n");
  
    printf("\r\nrobot_vel:\r\n");
    robot_vel_.print();
    printf("\r\n");

    printf("\r\nodom_vel:\r\n");
    odom_vel_.print();
    printf("\r\n");

    printf("\r\nodom_pos:\r\n");
    odom_pos_.print();
    printf("\r\n");
}
