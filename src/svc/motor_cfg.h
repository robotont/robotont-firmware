#ifndef MOTOR_CFG_H
#define MOTOR_CFG_H

#include "motor.h"

#define MOTOR_0_WHEEL_PHI  (M_PI / 3.0f)
#define MOTOR_1_WHEEL_PHI  M_PI
#define MOTOR_2_WHEEL_PHI  (5.0f / 3.0f * M_PI)
#define MOTOR_NUMBER       3u

#define MOTOR_WHEEL_R      0.145f

#define MOTOR_GEAR_RATIO   18.75f // gearbox reduction ratio
#define MOTOR_WHEEL_RADIUS 0.035f // wheel outer radius

#define MOTOR_MAX_LIN_VEL  0.4f // m/s
#define MOTOR_MAX_ANG_VEL  1.0f // rad/s

#define ENCODER_CPR        64 // TODO move to enc

void motor_cfg_setConfig(MotorCfgType *ptr_motor1_config, MotorCfgType *ptr_motor2_config, MotorCfgType *ptr_motor3_config);

#endif