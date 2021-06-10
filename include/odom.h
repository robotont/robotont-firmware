#ifndef ODOM_H
#define ODOM_H

#include "mbed.h"
#include "Matrix.h"

#include "motor.h"

/** 
 * \brief Odometry class
 * This class calculates robot's position according to the wheel speed and their confifguration
 */
class Odom
{
public:
  /**
   * \brief Constructor
   * \param cfg0 Configuration of motor 0
   * \param cfg1 Configuration of motor 1
   * \param cfg2 Configuration of motor 2
   * \param delta_t period of how often the odometry will be updated
   */
  Odom(const MotorConfig& cfg0, const MotorConfig& cfg1, const MotorConfig& cfg2, float delta_t);
  ~Odom();

  /**
   * \brief Resets the odometry
   * This function resets the odom frame velocity and position matrices to zero.
   */
  void reset();

  /**
   * \brief Updates the odometry
   * \param vel_1 Velocity of wheel 1
   * \param vel_2 Velocity of wheel 2
   * \param vel_3 Velocity of wheel 3
   *
   * This function calculates the new robot position based on its wheel speeds.
   */
  void update(float vel_1, float vel_2, float vel_3);

  /**
   * \todo FINISH DOCUMENTATION
   */
  float getPosX() const
  {
    return odom_pos_(1, 1);
  };

  float getPosY() const
  {
    return odom_pos_(2, 1);
  };

  float getOriZ() const
  {
    return odom_pos_(3, 1);
  };

  float getLinVelX() const
  {
    return odom_vel_(1, 1);
  };
  
  float getLinVelY() const
  {
    return odom_vel_(2, 1);
  };

  float getAngVelZ() const
  {
    return odom_vel_(3, 1);
  };

  /**
   * \brief Prints odom internals for debugging purposes
   */
  void print();


private:
  const MotorConfig motor_configs_[3]; /**< Motor configurations */

  Matrix wheel_vel_; /**< Vector with wheel speeds [rad/s] */
  Matrix robot_vel_; /**< Velocity vector (dX, dY, dtheta) in robot frame */
  Matrix odom_vel_;  /**< Velocity vector (dx, dy, dtheta) in odom frame */
  Matrix odom_pos_;  /**< Position Vector (x, y, theta) in odom frame */

  Matrix odom_matrix_;     /**< Odometry matrix */
  Matrix odom_matrix_inv_; /**< Inverse of the odometry matrix */

  const float delta_t_;
};

#endif // ODOM_H
