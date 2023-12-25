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

/* If velocity command is not received within this period all motors are stopped */
#define PACKET_TIMEOUT_MS 1000

#define PID_KP            600u
#define PID_KI            15000u
#define PID_KD            0u

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

/* Runtime variables */

static RobotVelocityType velocity = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static MotorSpeedType motor_speed = { 0.0f, 0.0f, 0.0f }; /* Target motor speed, that received from CMD handler */

static uint32_t priv_receive_time_ms; /* Last time, when command received. Based on that calculated timeout */

static MotorHandleType *motor0_handler;
static MotorHandleType *motor1_handler;
static MotorHandleType *motor2_handler;
static PID_TypeDef pid0_handler;
static PID_TypeDef pid1_handler;
static PID_TypeDef pid2_handler;
OdomType *odom_handler;

static void initPID(void);
static void printOdom(void);
static void pwmSetHigh(TIM_HandleTypeDef *timer_handler);
static void pwmSetLow(TIM_HandleTypeDef *timer_handler);

void movement_init(MotorHandleType *m0_handler, MotorHandleType *m1_handler, MotorHandleType *m2_handler)
{
    motor0_handler = m0_handler;
    motor1_handler = m1_handler;
    motor2_handler = m2_handler;
    MotorPinoutType motor0_pinout;
    MotorPinoutType motor1_pinout;
    MotorPinoutType motor2_pinout;

    ioif_init();
    timerif_init();

    motor_configurePinout(&motor0_pinout, &motor1_pinout, &motor2_pinout);
    motor_init(motor0_handler, &motor0_pinout, TIMER_PWM_M0);
    motor_init(motor1_handler, &motor1_pinout, TIMER_PWM_M1);
    motor_init(motor2_handler, &motor2_pinout, TIMER_PWM_M2);
    odom_init(odom_handler);
    initPID();

    timerif_setPeriodElapsedCallback((TimerCallbackType)movement_pwmHighCallback);
    timerif_setPulseFinishedCallback((TimerCallbackType)movement_pwmLowCallback);
    timerif_initInterrups();
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

    motor0_handler->data->linear_velocity_setpoint = motor_speed.motor0;
    motor1_handler->data->linear_velocity_setpoint = motor_speed.motor1;
    motor2_handler->data->linear_velocity_setpoint = motor_speed.motor2;

    PID_Compute(&pid0_handler);
    PID_Compute(&pid1_handler);
    PID_Compute(&pid2_handler);

    motor_update(motor0_handler);
    motor_update(motor1_handler);
    motor_update(motor2_handler);

    odom_update(odom_handler, motor0_handler->data->linear_velocity, motor1_handler->data->linear_velocity,
                motor2_handler->data->linear_velocity, (MAIN_LOOP_DT_MS / 1000.0f));

    printOdom();
}

void movement_pwmHighCallback(TIM_HandleTypeDef *timer_handler)
{
    if (timer_handler->Instance == TIMER_PWM_M0->Instance)
    {
        ioif_writePin(&motor0_handler->pwm_pin, true);
    }
    else if (timer_handler->Instance == TIMER_PWM_M1->Instance)
    {
        ioif_writePin(&motor1_handler->pwm_pin, true);
    }
    else if (timer_handler->Instance == TIMER_PWM_M2->Instance)
    {
        ioif_writePin(&motor2_handler->pwm_pin, true);
    }
}

void movement_pwmLowCallback(TIM_HandleTypeDef *timer_handler)
{
    if (timer_handler->Instance == TIMER_PWM_M0->Instance)
    {
        ioif_writePin(&motor0_handler->pwm_pin, false);
    }
    else if (timer_handler->Instance == TIMER_PWM_M1->Instance)
    {
        ioif_writePin(&motor1_handler->pwm_pin, false);
    }
    else if (timer_handler->Instance == TIMER_PWM_M2->Instance)
    {
        ioif_writePin(&motor2_handler->pwm_pin, false);
    }
}

static void initPID(void)
{
    double *ptr_input;
    double *ptr_output;
    double *ptr_setpoint;

    ptr_input = &(motor0_handler->data->linear_velocity);
    ptr_output = &(motor0_handler->data->effort);
    ptr_setpoint = &(motor0_handler->data->linear_velocity_setpoint);
    PID(&pid0_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    ptr_input = &(motor1_handler->data->linear_velocity);
    ptr_output = &(motor1_handler->data->effort);
    ptr_setpoint = &(motor1_handler->data->linear_velocity_setpoint);
    PID(&pid1_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    ptr_input = &(motor2_handler->data->linear_velocity);
    ptr_output = &(motor2_handler->data->effort);
    ptr_setpoint = &(motor2_handler->data->linear_velocity_setpoint);
    PID(&pid2_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    PID_SetMode(&pid0_handler, _PID_MODE_AUTOMATIC);
    PID_SetMode(&pid1_handler, _PID_MODE_AUTOMATIC);
    PID_SetMode(&pid2_handler, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&pid0_handler, -1000.0f, 1000.0f);
    PID_SetOutputLimits(&pid1_handler, -1000.0f, 1000.0f);
    PID_SetOutputLimits(&pid2_handler, -1000.0f, 1000.0f);
    PID_SetSampleTime(&pid0_handler, MAIN_LOOP_DT_MS);
    PID_SetSampleTime(&pid1_handler, MAIN_LOOP_DT_MS);
    PID_SetSampleTime(&pid2_handler, MAIN_LOOP_DT_MS);
}

static void printOdom(void)
{
    float pos_x = odom_handler->odom_pos_data[0];
    float pos_y = odom_handler->odom_pos_data[1];
    float pos_z = odom_handler->odom_pos_data[2];
    float vel_x = odom_handler->robot_vel_data[0];
    float vel_y = odom_handler->robot_vel_data[1];
    float vel_z = odom_handler->robot_vel_data[2];
    printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);
}
