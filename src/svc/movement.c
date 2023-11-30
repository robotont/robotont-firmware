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
#include "odom.h"
#include "peripheral.h"
#include "pid.h"
#include "sw_enc.h"

// TODO [code quality, critical ] multiple deifnitions
#define MAIN_LOOP_DT_MS 10

// TODO [code quality] Move this define?
#define PACKET_TIMEOUT_MS 1000 // If velocity command is not received within this period all motors are stopped

#define MAX_LINEAR_VEL    0.4f // m/s
#define MAX_ANGULAR_VEL   1.0f // rad/s

// TODO [code quality] move those defines to the motor module?^^^
#define MOTOR_WHEEL_R     0.145f
#define MOTOR_0_WHEEL_PHI (M_PI / 3.0f)
#define MOTOR_1_WHEEL_PHI M_PI
#define MOTOR_2_WHEEL_PHI (5.0f / 3.0f * M_PI)

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
static volatile RobotVelocityType priv_velocity = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static volatile MotorSpeedType priv_motor_speed = { 0.0f, 0.0f, 0.0f }; // TODO [debug] remove volatile, debug

// TODO [code quality] for timeout logic, use state states? Updates states separately?
static uint32_t priv_receive_time_ms;

// TODO [code quality] get rid of pointers, pass data directly as an argument?
static MotorType *ptr_motor0;
static MotorType *ptr_motor1;
static MotorType *ptr_motor2;
static PID_TypeDef hPID0, hPID1, hPID2;

OdomType hodom;
MotorCfgType mcfg0, mcfg1, mcfg2; // TODO move to the local scope?

static void initPID(void);

void movement_init(MotorType *ptr_m0, MotorType *ptr_m1, MotorType *ptr_m2, EncoderType *ptr_enc0,
                   EncoderType *ptr_enc1, EncoderType *ptr_enc2)
{
    ptr_motor0 = ptr_m0;
    ptr_motor1 = ptr_m1;
    ptr_motor2 = ptr_m2;

    motor_setConfig(&mcfg0, &mcfg1, &mcfg2);
    motor_init(ptr_motor0, &mcfg0, ptr_enc0, &(TIM3->CCR1), &htim3);
    motor_init(ptr_motor1, &mcfg1, ptr_enc1, &(TIM11->CCR1), &htim11);
    motor_init(ptr_motor2, &mcfg2, ptr_enc2, &(TIM13->CCR1), &htim13);
    odom_init(&hodom, &mcfg0, &mcfg1, &mcfg2);

    initPID();

    // TODO [code quality] move to the motor, as separate function (e.g. initPins)
    HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->en2_port, ptr_motor0->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->en2_port, ptr_motor1->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->en2_port, ptr_motor2->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->nsleep_port, ptr_motor0->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor0
    HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->nsleep_port, ptr_motor1->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor1
    HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->nsleep_port, ptr_motor2->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor2
    HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->en2_port, ptr_motor0->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->en2_port, ptr_motor1->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->en2_port, ptr_motor2->ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(ptr_motor0->ptr_motor_config->nsleep_port, ptr_motor0->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor0
    HAL_GPIO_WritePin(ptr_motor1->ptr_motor_config->nsleep_port, ptr_motor1->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor1
    HAL_GPIO_WritePin(ptr_motor2->ptr_motor_config->nsleep_port, ptr_motor2->ptr_motor_config->nsleep_pin, SET); // Enable drv of motor2
}

void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation] error handler, if input is wrong (e.g. "MS:35,abcd\r\n")

    char *token = strtok((char *)ptr_data, ":");
    priv_velocity.x = atof(token);
    token = strtok(NULL, ":");
    priv_velocity.y = atof(token);
    token = strtok(NULL, "\r\n");
    priv_velocity.z = atof(token);

    priv_velocity.dir = atan2(priv_velocity.y, priv_velocity.x);
    priv_velocity.mag = sqrt(SQUARE_OF(priv_velocity.x) + SQUARE_OF(priv_velocity.y));

    priv_velocity.mag = MIN(priv_velocity.mag, MAX_LINEAR_VEL);
    priv_velocity.z = MAX(MIN(priv_velocity.z, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL);

    priv_motor_speed.motor0 =
        priv_velocity.mag * sin(priv_velocity.dir - MOTOR_0_WHEEL_PHI) + MOTOR_WHEEL_R * priv_velocity.z;
    priv_motor_speed.motor1 =
        priv_velocity.mag * sin(priv_velocity.dir - MOTOR_1_WHEEL_PHI) + MOTOR_WHEEL_R * priv_velocity.z;
    priv_motor_speed.motor2 =
        priv_velocity.mag * sin(priv_velocity.dir - MOTOR_2_WHEEL_PHI) + MOTOR_WHEEL_R * priv_velocity.z;

    priv_receive_time_ms = HAL_GetTick(); // TODO [code quality] get rid of direct HAL usage
}

void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation] error handler, if input is wrong (e.g. "MS:35\r\n")

    char *token = strtok((char *)ptr_data, ":");
    priv_motor_speed.motor0 = atof(token);
    token = strtok(NULL, ":");
    priv_motor_speed.motor1 = atof(token);
    token = strtok(NULL, "\r\n");
    priv_motor_speed.motor2 = atof(token);

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
        priv_motor_speed.motor0 = 0.0f;
        priv_motor_speed.motor1 = 0.0f;
        priv_motor_speed.motor2 = 0.0f;
    }

    ptr_motor0->linear_velocity_setpoint = priv_motor_speed.motor0;
    ptr_motor1->linear_velocity_setpoint = priv_motor_speed.motor1;
    ptr_motor2->linear_velocity_setpoint = priv_motor_speed.motor2;

    PID_Compute(&hPID0);
    PID_Compute(&hPID1);
    PID_Compute(&hPID2);

    motor_update(ptr_motor0);
    motor_update(ptr_motor1);
    motor_update(ptr_motor2);

    odom_update(&hodom, ptr_motor0->linear_velocity, ptr_motor1->linear_velocity, ptr_motor2->linear_velocity,
                MAIN_LOOP_DT_MS / 1000.0f);

    printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", hodom.odom_pos_data[0], hodom.odom_pos_data[1], hodom.odom_pos_data[2],
           hodom.robot_vel_data[0], hodom.robot_vel_data[1], hodom.robot_vel_data[2]);
}

static void initPID(void)
{
    uint32_t pid_k = 600;
    uint32_t pid_i = 15000;
    uint32_t pid_d = 0;
    PID(&hPID0, &(ptr_motor0->linear_velocity), &(ptr_motor0->effort), &(ptr_motor0->linear_velocity_setpoint), pid_k,
        pid_i, pid_d, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID1, &(ptr_motor1->linear_velocity), &(ptr_motor1->effort), &(ptr_motor1->linear_velocity_setpoint), pid_k,
        pid_i, pid_d, _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID2, &(ptr_motor2->linear_velocity), &(ptr_motor2->effort), &(ptr_motor2->linear_velocity_setpoint), pid_k,
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
