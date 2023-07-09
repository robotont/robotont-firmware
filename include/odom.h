#ifndef __ODOM_H__
#define __ODOM_H__

#include <stdint.h>
#include "motor.h"


typedef struct
{
	double wheel_vel[3]; // Vector of wheel velocities [rad/s]
	double robot_vel[3]; // Velocity vector (dX, dY, dtheta) in robot frame [m/s]
	double odom_vel[3];  // Velocity vector (dx, dy, dtheta) in odom frame [m/s]
	double odom_pos[3];  // Position vector (x, y, theta) in odom frame [m]

	double odom_matrix[3][3]; 		// Odom matrix
	double odom_matrix_inv[3][3]; // Inverse of odom matrix
} odom_t;

void OdomInit(odom_t* odom, motor_config_t* hmc0, motor_config_t* hmc1, motor_config_t* hmc2);
void OdomReset(odom_t* odom);
void OdomUpdate(odom_t* odom, float vel_1, float vel_2, float vel_3, float dt);

#endif