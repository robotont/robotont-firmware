/**
 * @file movement.c
 * @brief
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "movement.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "macros.h"

#define MAX_LINEAR_VEL  0.4f // m/s
#define MAX_ANGULAR_VEL 1.0f // rad/s

/* Linear velocity in Cartesian (normal) coordinates */
typedef struct
{
    float x; // linear
    float y; // linear
    float z; // angular
} LinearVelocityCType;

/** Linear velocity in polar coordinates */
typedef struct
{
    float direction;
    float magnitude;
} LinearVelocityPType;

typedef struct
{
    float motor1;
    float motor2;
    float motor3;
} MotorSpeedType;

static volatile LinearVelocityCType velocity_cartesian = { 0.0f, 0.0f, 0.0f };
static volatile LinearVelocityPType velocity_polar = { 0.0f, 0.0f };
static volatile MotorSpeedType motor_speed = { 0.0f, 0.0f, 0.0f }; // Todo remove volatile, debug

static MotorType *ptr_motor0;
static MotorType *ptr_motor1;
static MotorType *ptr_motor2;

void movement_init(MotorType *ptr_m0, MotorType *ptr_m1, MotorType *ptr_m2)
{
    // TODO move motor init here
    ptr_motor0 = ptr_m1;
    ptr_motor1 = ptr_m1;
    ptr_motor2 = ptr_m2;
}

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght)
{
    // update received tick

    // TODO error handler, if input is wrong (e.g. "MS:35\r\n")
    char *token = strtok((char *)ptr_data, ":");
    velocity_cartesian.x = atof(token);
    token = strtok(NULL, ":");
    velocity_cartesian.y = atof(token);
    token = strtok(NULL, "\r\n");
    velocity_cartesian.z = atof(token);

    velocity_polar.direction = atan2(velocity_cartesian.y, velocity_cartesian.x);
    velocity_polar.magnitude = sqrt(SQUARE_OF(velocity_cartesian.x) + SQUARE_OF(velocity_cartesian.y));
    velocity_polar.magnitude = MIN(velocity_polar.magnitude, MAX_LINEAR_VEL);
    velocity_cartesian.z = MAX(MIN(velocity_cartesian.z, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);
    // Apply velocities to motors
    // TODO
}

void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght)
{
    // update received tick

    // TODO error handler, if input is wrong (e.g. "MS:35\r\n")
    char *token = strtok((char *)ptr_data, ":");
    motor_speed.motor1 = atof(token);
    token = strtok(NULL, ":");
    motor_speed.motor2 = atof(token);
    token = strtok(NULL, "\r\n");
    motor_speed.motor3 = atof(token);

    // Apply velocities to motors
    // TODO
}

void movement_handleCommandsEF(uint8_t *ptr_data, uint16_t lenght)
{
#pragma "Not implemented"
    // // Command: EF (Effort control)
    // else if (last_packet[0] == 'E' && last_packet[1] == 'F')
    // {
    //     last_vel_received_tick = HAL_GetTick();
    //     char *pch;
    //     pch = strtok((char *)last_packet, ":");
    //     int arg = 0;
    //     while (pch != NULL)
    //     {
    //         if (arg == 1)
    //         {
    //             hm0.effort = atof(pch);
    //         }
    //         else if (arg == 2)
    //         {
    //             hm1.effort = atof(pch);
    //         }
    //         else if (arg == 3)
    //         {
    //             hm2.effort = atof(pch);
    //         }
    //         pch = strtok(NULL, ":");
    //         arg++;
    //     }
    // }
}

void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght)
{
#pragma "Not implemented"
    // // Command: OR (Odom Reset)
    // else if (last_packet[0] == 'O' && last_packet[1] == 'R')
    // {
    //     odom_reset(&hodom);
    // }
    // last_packet[0] = '\0'; // indicate that packet has been processed
    // last_packet_length = 0;
}

void movement_update(uint32_t update_time_ms)
{
}
