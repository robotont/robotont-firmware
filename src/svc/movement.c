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
#include "main.h"
#include "motor_cfg.h"
#include "odom.h"
#include "peripheral.h"
#include "pid.h"
#include "sw_enc.h"
#include "timerif.h"

// TODO [code quality] Move this define?
#define PACKET_TIMEOUT_MS 1000 // If velocity command is not received within this period all motors are stopped

/* Speed that goes as an input to the PID controller of the each motor */
typedef struct
{
    float motor0;
    float motor1;
    float motor2;
} MotorSpeedType;

typedef struct
{
    float x;   // linear (Cartesian)
    float y;   // linear (Cartesian)
    float z;   // angular (Cartesian)
    float dir; // direction (polar)
    float mag; // magnitude (polar)
} RobotVelocityType;

// TODO [code quality] consider use setters and getters instead of globals?
static RobotVelocityType velocity = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static MotorSpeedType motor_speed = { 0.0f, 0.0f, 0.0f };

// TODO [code quality] for timeout logic, use state states? Updates states separately?
static uint32_t priv_receive_time_ms;

// TODO [code quality] get rid of pointers, pass data directly as an argument?
static MotorHandleType *ptr_motor0;
static MotorHandleType *ptr_motor1;
static MotorHandleType *ptr_motor2;
static PID_TypeDef hPID0, hPID1, hPID2;

OdomType hodom;

static void initPID(void);

void movement_init(MotorHandleType *m0_handler, MotorHandleType *m1_handler, MotorHandleType *m2_handler)
{
    timerif_init();
    
    ptr_motor0 = m0_handler;
    ptr_motor1 = m1_handler;
    ptr_motor2 = m2_handler;

    MotorPinoutType m0_pinout, m1_pinout, m2_pinout;

    motor_configurePinout(&m0_pinout, &m1_pinout, &m2_pinout);

    motor_init(ptr_motor0, &m0_pinout, &htim11);
    motor_init(ptr_motor1, &m1_pinout, &htim13);
    motor_init(ptr_motor2, &m2_pinout, &htim14);
    odom_init(&hodom);

    initPID();

    // TODO [code quality] move to the motor, as separate function (e.g. initPins)

    ioif_writePin(&ptr_motor0->pinout->en2_pin, false);
    ioif_writePin(&ptr_motor1->pinout->en2_pin, false);
    ioif_writePin(&ptr_motor2->pinout->en2_pin, false);
    // HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->en2_port, ptr_motor0->ptr_motor_config->en2_pin, RESET);
    // HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->en2_port, ptr_motor1->ptr_motor_config->en2_pin, RESET);
    // HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->en2_port, ptr_motor2->ptr_motor_config->en2_pin, RESET);
    // Enable drv of motorX
    ioif_writePin(&ptr_motor0->pinout->nsleep_pin, true);
    ioif_writePin(&ptr_motor1->pinout->nsleep_pin, true);
    ioif_writePin(&ptr_motor2->pinout->nsleep_pin, true);
    // HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->nsleep_port, ptr_motor0->ptr_motor_config->nsleep_pin, SET);
    // HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->nsleep_port, ptr_motor1->ptr_motor_config->nsleep_pin, SET);
    // HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->nsleep_port, ptr_motor2->ptr_motor_config->nsleep_pin, SET);
}

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation] error handler, if input is wrong (e.g. "MS:35,abcd\r\n")

    char *token = strtok((char *)ptr_data, ":");
    velocity.x = atof(token);
    token = strtok(NULL, ":");
    velocity.y = atof(token);
    token = strtok(NULL, "\r\n");
    velocity.z = atof(token);

    velocity.dir = atan2(velocity.y, velocity.x);
    velocity.mag = sqrt(SQUARE_OF(velocity.x) + SQUARE_OF(velocity.y));

    velocity.mag = MIN(velocity.mag, MOTOR_MAX_LIN_VEL);
    velocity.z = MAX(MIN(velocity.z, MOTOR_MAX_ANG_VEL), -MOTOR_MAX_ANG_VEL);

    motor_speed.motor0 = velocity.mag * sin(velocity.dir - MOTOR_0_WHEEL_PHI) + MOTOR_WHEEL_R * velocity.z;
    motor_speed.motor1 = velocity.mag * sin(velocity.dir - MOTOR_1_WHEEL_PHI) + MOTOR_WHEEL_R * velocity.z;
    motor_speed.motor2 = velocity.mag * sin(velocity.dir - MOTOR_2_WHEEL_PHI) + MOTOR_WHEEL_R * velocity.z;

    priv_receive_time_ms = HAL_GetTick(); // TODO [code quality] get rid of direct HAL usage
}

void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation] error handler, if input is wrong (e.g. "MS:35\r\n")

    char *token = strtok((char *)ptr_data, ":");
    motor_speed.motor0 = atof(token);
    token = strtok(NULL, ":");
    motor_speed.motor1 = atof(token);
    token = strtok(NULL, "\r\n");
    motor_speed.motor2 = atof(token);

    priv_receive_time_ms = HAL_GetTick(); // TODO [code quality] get rid of direct HAL usage
}

void movement_handleCommandsEF(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation]
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
    //             ptr_motor0->effort = atof(pch);
    //         }
    //         else if (arg == 2)
    //         {
    //             ptr_motor1->effort = atof(pch);
    //         }
    //         else if (arg == 3)
    //         {
    //             ptr_motor2->effort = atof(pch);
    //         }
    //         pch = strtok(NULL, ":");
    //         arg++;
    //     }
    // }
}

void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation]
    // // Command: OR (Odom Reset)
    // else if (last_packet[0] == 'O' && last_packet[1] == 'R')
    // {
    //     odom_reset(&hodom);
    // }
    // last_packet[0] = '\0'; // indicate that packet has been processed
    // last_packet_length = 0;
}

void movement_update()
{
    uint32_t current_time_ms = HAL_GetTick(); // TODO [code quality] get rid of direct HAL usage
    if (current_time_ms > priv_receive_time_ms + PACKET_TIMEOUT_MS)
    {
        motor_speed.motor0 = 0.0f;
        motor_speed.motor1 = 0.0f;
        motor_speed.motor2 = 0.0f;
    }

    ptr_motor0->data->linear_velocity_setpoint = motor_speed.motor0;
    ptr_motor1->data->linear_velocity_setpoint = motor_speed.motor1;
    ptr_motor2->data->linear_velocity_setpoint = motor_speed.motor2;

    PID_Compute(&hPID0);
    PID_Compute(&hPID1);
    PID_Compute(&hPID2);

    motor_update(ptr_motor0);
    motor_update(ptr_motor1);
    motor_update(ptr_motor2);

    odom_update(&hodom, ptr_motor0->data->linear_velocity, ptr_motor1->data->linear_velocity, ptr_motor2->data->linear_velocity,
               ( MAIN_LOOP_DT_MS / 1000.0f));

    printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", hodom.odom_pos_data[0], hodom.odom_pos_data[1], hodom.odom_pos_data[2],
           hodom.robot_vel_data[0], hodom.robot_vel_data[1], hodom.robot_vel_data[2]);
}

static void initPID(void)
{
    uint32_t pid_k = 600;
    uint32_t pid_i = 15000;
    uint32_t pid_d = 0;
    PID(&hPID0, &(ptr_motor0->data->linear_velocity), &(ptr_motor0->data->effort), &(ptr_motor0->data->linear_velocity_setpoint), pid_k,
        pid_i, pid_d, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID1, &(ptr_motor1->data->linear_velocity), &(ptr_motor1->data->effort), &(ptr_motor1->data->linear_velocity_setpoint), pid_k,
        pid_i, pid_d, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID2, &(ptr_motor2->data->linear_velocity), &(ptr_motor2->data->effort), &(ptr_motor2->data->linear_velocity_setpoint), pid_k,
        pid_i, pid_d, _PID_P_ON_E, _PID_CD_DIRECT);

    PID_SetMode(&hPID0, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID0, -1000, 1000);
    PID_SetSampleTime(&hPID0, MAIN_LOOP_DT_MS);
    PID_SetMode(&hPID1, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID1, -1000, 1000);
    PID_SetSampleTime(&hPID1, MAIN_LOOP_DT_MS);
    PID_SetMode(&hPID2, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID2, -1000, 1000);
    PID_SetSampleTime(&hPID2, MAIN_LOOP_DT_MS);
}
