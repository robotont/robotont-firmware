#include <stdio.h>
#include "odom.h"

#define ARM_MATH_CM4
#include <arm_math.h>

void OdomInit(odom_t* ho, motor_config_t* hmc0, motor_config_t* hmc1, motor_config_t* hmc2)
{
  motor_config_t* motor_configs[3];
	motor_configs[0] = hmc0;
	motor_configs[1] = hmc1;
  motor_configs[2] = hmc2;
  
  // add elements to odom matrix row by row (each row is a wheel)
  for (int i = 0; i < 3; i++)
  {
    ho->odom_matrix[i][0] = -sin(motor_configs[i]->wheel_pos_phi);
    ho->odom_matrix[i][1] = cos(motor_configs[i]->wheel_pos_phi);
    ho->odom_matrix[i][2] = motor_configs[i]->wheel_pos_r;
  }

	// Calculate inverse of odom matrix
	double det = 
		ho->odom_matrix[0][0] * ho->odom_matrix[1][1] * ho->odom_matrix[2][2] +  ho->odom_matrix[0][1] * ho->odom_matrix[1][2] * ho->odom_matrix[2][0] +
		ho->odom_matrix[0][2] * ho->odom_matrix[1][0] * ho->odom_matrix[2][1] - ho->odom_matrix[0][2] * ho->odom_matrix[1][1] * ho->odom_matrix[2][0] -
		ho->odom_matrix[0][1] * ho->odom_matrix[1][0] * ho->odom_matrix[2][2] - ho->odom_matrix[0][0] * ho->odom_matrix[1][2] * ho->odom_matrix[2][1];
  
	if (det == 0)
	{
		printf("Odom matrix is singular!!!\n");
		ho->odom_matrix_inv[0][0] = 1;
		ho->odom_matrix_inv[0][1] = 0;
		ho->odom_matrix_inv[0][2] = 0;
		ho->odom_matrix_inv[1][0] = 0;
		ho->odom_matrix_inv[1][1] = 1;
		ho->odom_matrix_inv[1][2] = 0;
		ho->odom_matrix_inv[2][0] = 0;
		ho->odom_matrix_inv[2][1] = 0;
		ho->odom_matrix_inv[2][2] = 1;
	}
	else
	{
		ho->odom_matrix_inv[0][0] = (ho->odom_matrix[1][1] * ho->odom_matrix[2][2] - ho->odom_matrix[1][2] * ho->odom_matrix[2][1]) / det;
		ho->odom_matrix_inv[0][1] = (ho->odom_matrix[0][2] * ho->odom_matrix[2][1] - ho->odom_matrix[0][1] * ho->odom_matrix[2][2]) / det;
		ho->odom_matrix_inv[0][2] = (ho->odom_matrix[0][1] * ho->odom_matrix[1][2] - ho->odom_matrix[0][2] * ho->odom_matrix[1][1]) / det;
		ho->odom_matrix_inv[1][0] = (ho->odom_matrix[1][2] * ho->odom_matrix[2][0] - ho->odom_matrix[1][0] * ho->odom_matrix[2][2]) / det;
		ho->odom_matrix_inv[1][1] = (ho->odom_matrix[0][0] * ho->odom_matrix[2][2] - ho->odom_matrix[0][2] * ho->odom_matrix[2][0]) / det;
		ho->odom_matrix_inv[1][2] = (ho->odom_matrix[0][2] * ho->odom_matrix[1][0] - ho->odom_matrix[0][0] * ho->odom_matrix[1][2]) / det;
		ho->odom_matrix_inv[2][0] = (ho->odom_matrix[1][0] * ho->odom_matrix[2][1] - ho->odom_matrix[1][1] * ho->odom_matrix[2][0]) / det;
		ho->odom_matrix_inv[2][1] = (ho->odom_matrix[0][1] * ho->odom_matrix[2][0] - ho->odom_matrix[0][0] * ho->odom_matrix[2][1]) / det;
		ho->odom_matrix_inv[2][2] = (ho->odom_matrix[0][0] * ho->odom_matrix[1][1] - ho->odom_matrix[0][1] * ho->odom_matrix[1][0]) / det;
	}

	// initialize vectors with zeros
  OdomReset(ho);
}

void OdomReset(odom_t* ho)
{
  ho->odom_pos[0] = 0;
  ho->odom_pos[1] = 0;
  ho->odom_pos[2] = 0;

  ho->odom_vel[0] = 0;
  ho->odom_vel[1] = 0;
  ho->odom_vel[2] = 0;

  ho->wheel_vel[0] = 0;
  ho->wheel_vel[1] = 0;
  ho->wheel_vel[2] = 0;
}

void OdomUpdate(odom_t* ho, float vel_1, float vel_2, float vel_3, float dt)
{
  ho->wheel_vel[0] = vel_1;
  ho->wheel_vel[1] = vel_2;
  ho->wheel_vel[2] = vel_3;

	// Calculate robot velocity
	for (int i = 0; i < 3; i++)
	{
		ho->robot_vel[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			ho->robot_vel[i] += ho->odom_matrix_inv[i][j] * ho->wheel_vel[j];
		}
	}

  // transform velocities from robot frame to odom frame
/*	for (int i = 0; i < 3; i++)
	{
		ho->odom_vel[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			ho->odom_vel[i] += ho->odom_matrix[i][j] * ho->robot_vel[j];
		}
	}
	*/
	
	
	ho->odom_vel[0] = ho->robot_vel[0] * cos(ho->odom_pos[2]) - ho->robot_vel[1] * sin(ho->odom_pos[2]);
  ho->odom_vel[1] = ho->robot_vel[0] * sin(ho->odom_pos[2]) + ho->robot_vel[1] * cos(ho->odom_pos[2]);
  ho->odom_vel[2] = ho->robot_vel[2];
	

  // position integration
    ho->odom_pos[0] += ho->odom_vel[0] * dt;
    ho->odom_pos[1] += ho->odom_vel[1] * dt;
    ho->odom_pos[2] += ho->odom_vel[2] * dt;

		// Transform velocities from robot coordinates to odom coordinates
		
		
}
