#ifndef MOTOR_CFG_H
#define MOTOR_CFG_H

#include "motor.h"

#define MOTOR_0_WHEEL_PHI    (M_PI / 3.0f)
#define MOTOR_1_WHEEL_PHI    M_PI
#define MOTOR_2_WHEEL_PHI    (5.0f / 3.0f * M_PI)

#define MOTOR_WHEEL_R        0.145f

#define MOTOR_GEAR_RATIO     18.75f // gearbox reduction ratio
#define MOTOR_WHEEL__OUTER_R 0.035f // wheel outer radius

#define MOTOR_MAX_LIN_VEL    0.4f // m/s
#define MOTOR_MAX_ANG_VEL    1.0f // rad/s

#define MOTOR_ENC_CPR        64

void motor_configurePinout(MotorPinoutType *pinout_m0, MotorPinoutType *pinout_m1, MotorPinoutType *pinout_m2);

#endif