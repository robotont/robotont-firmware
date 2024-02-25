/**
 * @file odom.h
 * @brief Service. Calculates odometry paramters, based on fact, that robot is omnimotional with three motors
 *
 * @author Veiko Vunder
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef ODOM_H
#define ODOM_H

#define ARM_MATH_CM4
#include <arm_math.h>
#include <stdint.h>

#include "motor.h"

typedef struct
{
    float32_t wheel_vel_data[3];             // Vector of wheel velocities [rad/s]
    arm_matrix_instance_f32 wheel_vel;       // 3x1 matrix instance for wheel velocities
    float32_t robot_vel_data[3];             // Velocity vector (dX, dY, dtheta) in robot frame [m/s]
    arm_matrix_instance_f32 robot_vel;       // 3x1 matrix instance for robot velocities in robot frame
    float32_t odom_vel_data[3];              // Velocity vector (dx, dy, dtheta) in ptr_odom frame [m/s]
    arm_matrix_instance_f32 odom_vel;        // 3x1 matrix instance for robot velocities in ptr_odom frame
    float32_t odom_pos_data[3];              // Position vector (x, y, theta) in ptr_odom frame [m]
    arm_matrix_instance_f32 odom_pos;        // 3x1 matrix instance for robot position in ptr_odom frame
    arm_matrix_instance_f32 odom_matrix;     // Odom matrix
    float32_t odom_matrix_data[9];           // Odom matrix data
    arm_matrix_instance_f32 odom_matrix_inv; // Inverse of ptr_odom matrix
    float32_t odom_matrix_inv_data[9];       // Data of inverse of ptr_odom matrix
} OdomType;

void odom_init(OdomType *ptr_odom);
void odom_reset(OdomType *ptr_odom);
void odom_update(OdomType *ptr_odom, float vel_1, float vel_2, float vel_3, float dt);

#endif