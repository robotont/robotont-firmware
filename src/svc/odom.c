/**
 * @file odom.c
 * @brief Service. Calculates odometry paramters, based on fact, that robot is omnimotional with three motors
 *
 * @author Veiko Vunder
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "odom.h"

#include <stdio.h>

#include "motor_cfg.h"

/**
 * @brief Initializes odometry module
 */
void odom_init(OdomType *ptr_odom)
{
    float wheel_pos_phi[3u];
    wheel_pos_phi[0] = MOTOR_0_WHEEL_PHI;
    wheel_pos_phi[1] = MOTOR_1_WHEEL_PHI;
    wheel_pos_phi[2] = MOTOR_2_WHEEL_PHI;
    // add elements to odom matrix row by row (each row is a wheel)
    for (int i = 0; i < 3; i++)
    {
        ptr_odom->odom_matrix_data[i * 3 + 0] = -sin(wheel_pos_phi[i]);
        ptr_odom->odom_matrix_data[i * 3 + 1] = cos(wheel_pos_phi[i]);
        // TODO [implementation] Be careful, if motors are not the same size
        ptr_odom->odom_matrix_data[i * 3 + 2] = MOTOR_WHEEL_R;
    }

    // Initialize matrix instances for wheel velocities, robot velocities, odom velocities and odom position
    arm_mat_init_f32(&(ptr_odom->wheel_vel), 3, 1, ptr_odom->wheel_vel_data);
    arm_mat_init_f32(&(ptr_odom->robot_vel), 3, 1, ptr_odom->robot_vel_data);
    arm_mat_init_f32(&(ptr_odom->odom_vel), 3, 1, ptr_odom->odom_vel_data);
    arm_mat_init_f32(&(ptr_odom->odom_pos), 3, 1, ptr_odom->odom_pos_data);

    // Initialize data structures for odom matrix and it's inverse
    arm_mat_init_f32(&(ptr_odom->odom_matrix), 3, 3, ptr_odom->odom_matrix_data);
    arm_mat_init_f32(&(ptr_odom->odom_matrix_inv), 3, 3, ptr_odom->odom_matrix_inv_data);

    // Calculate inverse of odom matrix
    if (arm_mat_inverse_f32(&(ptr_odom->odom_matrix), &(ptr_odom->odom_matrix_inv)) == ARM_MATH_SINGULAR)
    {
        printf("Odom matrix is singular and finding it's inverse is not possible!!\n");
    }

    // initialize vectors with zeros
    odom_reset(ptr_odom);
}

/**
 * @brief Resets all odometry values to 0.0f
 */
void odom_reset(OdomType *ptr_odom)
{
    arm_scale_f32(ptr_odom->wheel_vel_data, 0.0f, ptr_odom->wheel_vel_data, 3);
    arm_scale_f32(ptr_odom->robot_vel_data, 0.0f, ptr_odom->robot_vel_data, 3);
    arm_scale_f32(ptr_odom->odom_vel_data, 0.0f, ptr_odom->odom_vel_data, 3);
    arm_scale_f32(ptr_odom->odom_pos_data, 0.0f, ptr_odom->odom_pos_data, 3);
}

/**
 * @brief Calculates robot's odometry (pos x, y, z and velocity x, y, z) based on each motors velocity
 */
void odom_update(OdomType *ptr_odom, float vel_1, float vel_2, float vel_3, float dt)
{
    ptr_odom->wheel_vel_data[0] = vel_1;
    ptr_odom->wheel_vel_data[1] = vel_2;
    ptr_odom->wheel_vel_data[2] = vel_3;

    // Calculate robot velocity
    arm_mat_mult_f32(&(ptr_odom->odom_matrix_inv), &(ptr_odom->wheel_vel), &(ptr_odom->robot_vel));

    // Transform velocities from robot frame to odom frame
    float32_t sin_ang_z = arm_sin_f32(ptr_odom->odom_pos_data[2]); // fast sine calculation
    float32_t cos_ang_z = arm_cos_f32(ptr_odom->odom_pos_data[2]); // fast cos calculation
    float32_t rotation_data[9] = { cos_ang_z, -sin_ang_z, 0, sin_ang_z, cos_ang_z, 0, 0, 0, 1 };
    arm_matrix_instance_f32 rotation_matrix;
    arm_mat_init_f32(&rotation_matrix, 3, 3, rotation_data);
    arm_mat_mult_f32(&rotation_matrix, &(ptr_odom->robot_vel), &(ptr_odom->odom_vel));

    // position integration (odom_pos = odom_pos + odom_vel * dt)
    float32_t odom_vel_times_dt[3];
    arm_scale_f32(ptr_odom->odom_vel_data, dt, odom_vel_times_dt, 3);
    arm_add_f32(ptr_odom->odom_pos_data, odom_vel_times_dt, ptr_odom->odom_pos_data, 3);
}
