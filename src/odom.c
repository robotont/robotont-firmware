#include <stdio.h>

#include "odom.h"


void OdomInit(odom_t* ho, motor_config_t* hmc0, motor_config_t* hmc1, motor_config_t* hmc2)
{
    motor_config_t* motor_configs[3];
    motor_configs[0] = hmc0;
    motor_configs[1] = hmc1;
    motor_configs[2] = hmc2;

    // add elements to odom matrix row by row (each row is a wheel)
    for (int i = 0; i < 3; i++)
    {
        ho->odom_matrix_data[i * 3 + 0] = -sin(motor_configs[i]->wheel_pos_phi);
        ho->odom_matrix_data[i * 3 + 1] = cos(motor_configs[i]->wheel_pos_phi);
        ho->odom_matrix_data[i * 3 + 2] = motor_configs[i]->wheel_pos_r;
    }

    // Initialize matrix instances for wheel velocities, robot velocities, odom velocities and odom position
    arm_mat_init_f32(&(ho->wheel_vel), 3, 1, ho->wheel_vel_data);
    arm_mat_init_f32(&(ho->robot_vel), 3, 1, ho->robot_vel_data);
    arm_mat_init_f32(&(ho->odom_vel), 3, 1, ho->odom_vel_data);
    arm_mat_init_f32(&(ho->odom_pos), 3, 1, ho->odom_pos_data);

    // Initialize data structures for odom matrix and it's inverse
    arm_mat_init_f32(&(ho->odom_matrix), 3, 3, ho->odom_matrix_data);
    arm_mat_init_f32(&(ho->odom_matrix_inv), 3, 3, ho->odom_matrix_inv_data);

    // Calculate inverse of odom matrix
    if (arm_mat_inverse_f32(&(ho->odom_matrix), &(ho->odom_matrix_inv)) == ARM_MATH_SINGULAR)
    {
        printf("Odom matrix is singular and finding it's inverse is not possible!!\n");
    }

    // initialize vectors with zeros
    OdomReset(ho);
}

void OdomReset(odom_t* ho)
{
    arm_scale_f32(ho->wheel_vel_data, 0, ho->wheel_vel_data, 3);
    arm_scale_f32(ho->robot_vel_data, 0, ho->robot_vel_data, 3);
    arm_scale_f32(ho->odom_vel_data, 0, ho->odom_vel_data, 3);
    arm_scale_f32(ho->odom_pos_data, 0, ho->odom_pos_data, 3);
}

void OdomUpdate(odom_t* ho, float vel_1, float vel_2, float vel_3, float dt)
{
    ho->wheel_vel_data[0] = vel_1;
    ho->wheel_vel_data[1] = vel_2;
    ho->wheel_vel_data[2] = vel_3;

    // Calculate robot velocity
    arm_mat_mult_f32(&(ho->odom_matrix_inv), &(ho->wheel_vel), &(ho->robot_vel));

    // Transform velocities from robot frame to odom frame
    float32_t sin_ang_z = arm_sin_f32(ho->odom_pos_data[2]);  // fast sine calculation
    float32_t cos_ang_z = arm_cos_f32(ho->odom_pos_data[2]);  // fast cos calculation
    float32_t rotation_data[9] = {cos_ang_z, -sin_ang_z, 0, sin_ang_z, cos_ang_z, 0, 0, 0, 1};
    arm_matrix_instance_f32 rotation_matrix;
    arm_mat_init_f32(&rotation_matrix, 3, 3, rotation_data);
    arm_mat_mult_f32(&rotation_matrix, &(ho->robot_vel), &(ho->odom_vel));

    // position integration (odom_pos = odom_pos + odom_vel * dt)
    float32_t odom_vel_times_dt[3];
    arm_scale_f32(ho->odom_vel_data, dt, odom_vel_times_dt, 3);
    arm_add_f32(ho->odom_pos_data, odom_vel_times_dt, ho->odom_pos_data, 3);
}
