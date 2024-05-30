/**
 * @file motor_cfg.h
 * @brief PWM motor configuration header file, contains commonly used contants
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#ifndef MOTOR_CFG_H
#define MOTOR_CFG_H

#include "motor.h"

#define MOTOR_0_WHEEL_PHI   (M_PI / 3.0f)        /* Wheen position angle, rel. to the center */
#define MOTOR_1_WHEEL_PHI   (M_PI)               /* Wheen position angle, rel. to the center */
#define MOTOR_2_WHEEL_PHI   (5.0f / 3.0f * M_PI) /* Wheen position angle, rel. to the center */
#define MOTOR_WHEEL_R       0.145f               /* Wheen inner radius (mm) */
#define MOTOR_GEAR_RATIO    18.75f               /* Gearbox reduction ratio */
#define MOTOR_WHEEL_OUTER_R 0.035f               /* Wheel outer radius */
#define MOTOR_MAX_LIN_VEL   0.4f                 /* Maximum allowed linear velocity (m/s) */
#define MOTOR_MAX_ANG_VEL   1.0f                 /* Maximum allowed angular velocity (rad/s) */
#define MOTOR_ENC_CPR       64                   /* Motor encoder Counts per Revolution */

void motor_configurePinout(MotorPinoutType *pinout_m0, MotorPinoutType *pinout_m1, MotorPinoutType *pinout_m2);

#endif