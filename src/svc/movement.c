/**
 * @file movement.c
 * @brief Service. Makes robot move using omnimotional movement
 *
 * Robot is able to move by controlling speeds of three PWM motors.
 * Speed is set by the user via serial (USB). If no data received within ~1s, then robot will stopped automaticly.
 * Robot calculates and sends back odometry data via serial as well.
 * All data is transfered as a string (char array) in the format "ARG:VALUE_0:...:VALUE_N\r\n".
 *
 * @note "\r\n" is used as a packet separator, so data will be collected until those symbols are received.
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
#include "system_hal.h"
#include "timerif.h"

#define PACKET_TIMEOUT_MS 1000 /* Timeout, if no new packets received, then all motors will be stopped */

#define PID_KP            60u   /* Proportional coef*/
#define PID_KI            1000u /* Integral coef*/
#define PID_KD            1u    /* Derivative coef*/

/* Speed that goes as an input to the PID controller of the each motor */
typedef struct
{
    float motor0;
    float motor1;
    float motor2;

    bool is_manually_controlled; /* In case of manual control, duty cycle directly sent to the motors */
    int16_t m0_duty_cycle;
    int16_t m1_duty_cycle;
    int16_t m2_duty_cycle;
} MotorSpeedType;

static MotorSpeedType motor_speed;   /* Target motor speed, that received from CMD handler */
static uint32_t last_packet_time_ms; /* Last time, when command received. Based on that calculated timeout */

MotorHandleType motor0_handler;
MotorHandleType motor1_handler;
MotorHandleType motor2_handler;
PID_TypeDef pid0_handler;
PID_TypeDef pid1_handler;
PID_TypeDef pid2_handler;
OdomType odom_handler;

static void initPID(void);
static void printOdom(void);

/**
 * @brief   Initialized movement module.
 * @details Initialized three motors with proper pin config, odometry, PID module and starts PWM interrups
 */
void movement_init()
{
    static MotorPinoutType motor0_pinout;
    static MotorPinoutType motor1_pinout;
    static MotorPinoutType motor2_pinout;

    motor_speed.motor0 = 0.0f;
    motor_speed.motor1 = 0.0f;
    motor_speed.motor2 = 0.0f;
    motor_speed.is_manually_controlled = false;
    motor_speed.m0_duty_cycle = 0;
    motor_speed.m1_duty_cycle = 0;
    motor_speed.m2_duty_cycle = 0;

    ioif_init();
    timerif_init();

    motor_configurePinout(&motor0_pinout, &motor1_pinout, &motor2_pinout);
    motor_init(&motor0_handler, &motor0_pinout, TIMER_PWM_M0, TIMER_ENC_M0);
    motor_init(&motor1_handler, &motor1_pinout, TIMER_PWM_M1, TIMER_ENC_M1);
    motor_init(&motor2_handler, &motor2_pinout, TIMER_PWM_M2, TIMER_ENC_M2);
    odom_init(&odom_handler);
    initPID();

    timerif_setPeriodElapsedCallback((TimerCallbackType)movement_pwmHighCallback);
    timerif_setPulseFinishedCallback((TimerCallbackType)movement_pwmLowCallback);
    timerif_initInterrupts();
}

/**
 * @brief   Handles "RS:speed0:speed1:speed2" command
 * @details Calculates motor speed based on required robot velocities (x, y, z); Omnimotional movement
 * @note    Called within ISR context from `cmd` module
 * @note    Resets "manual" mode allowing robot control speed by himself
 */
void movement_handleCommandsRS(uint8_t *ptr_data, uint16_t lenght)
{
    // TODO [implementation] error handler, if input is wrong (e.g. "MS:35,abcd\r\n")
    float velocity_x;   // linear (Cartesian)
    float velocity_y;   // linear (Cartesian)
    float velocity_z;   // angular (Cartesian)
    float velocity_dir; // direction (polar)
    float velocity_mag; // magnitude (polar)
    char *ptr_token;

    motor_speed.is_manually_controlled = false;
    last_packet_time_ms = system_hal_timestamp();

    ptr_token = strtok((char *)ptr_data, ":");
    velocity_x = atof(ptr_token);
    ptr_token = strtok(NULL, ":");
    velocity_y = atof(ptr_token);
    ptr_token = strtok(NULL, "\r\n");
    velocity_z = atof(ptr_token);
    velocity_dir = atan2(velocity_y, velocity_x);
    velocity_mag = sqrt(SQUARE_OF(velocity_x) + SQUARE_OF(velocity_y));
    velocity_mag = MIN(velocity_mag, MOTOR_MAX_LIN_VEL);
    velocity_z = MAX(MIN(velocity_z, MOTOR_MAX_ANG_VEL), -MOTOR_MAX_ANG_VEL);

    motor_speed.motor0 = velocity_mag * sin(velocity_dir - MOTOR_0_WHEEL_PHI) + MOTOR_WHEEL_R * velocity_z;
    motor_speed.motor1 = velocity_mag * sin(velocity_dir - MOTOR_1_WHEEL_PHI) + MOTOR_WHEEL_R * velocity_z;
    motor_speed.motor2 = velocity_mag * sin(velocity_dir - MOTOR_2_WHEEL_PHI) + MOTOR_WHEEL_R * velocity_z;
}

/**
 * @brief   Handles "MS:speed0:speed1:speed2" command
 * @details Sets each motor speed based on arguments
 * @note    Called within ISR context from `cmd` module
 * @note    Resets "manual" mode allowing robot control speed by himself
 */
void movement_handleCommandsMS(uint8_t *ptr_data, uint16_t lenght)
{
    float m0_speed;
    float m1_speed;
    float m2_speed;

    motor_speed.is_manually_controlled = false;
    last_packet_time_ms = system_hal_timestamp();

    char *token = strtok((char *)ptr_data, ":");
    m0_speed = atof(token);
    token = strtok(NULL, ":");
    m1_speed = atof(token);
    token = strtok(NULL, "\r\n");
    m2_speed = atof(token);

    motor_speed.motor0 = m0_speed;
    motor_speed.motor1 = m1_speed;
    motor_speed.motor2 = m2_speed;
}

/**
 * @brief   Handles "DC:duty_cycle1:duty_cycle2:duty_cycle3" command
 * @details Sets PWM duty cycle, that will be sent on each motor
 * @note    Called within ISR context from `cmd` module
 * @note    Robot will kept in "manual" mode until new command is received
 */
void movement_handleCommandsDC(uint8_t *ptr_data, uint16_t lenght)
{
    int16_t m0_duty_cycle;
    int16_t m1_duty_cycle;
    int16_t m2_duty_cycle;

    motor_speed.is_manually_controlled = true;

    char *token = strtok((char *)ptr_data, ":");
    m0_duty_cycle = (int16_t)atof(token);
    token = strtok(NULL, ":");
    m1_duty_cycle = (int16_t)atof(token);
    token = strtok(NULL, "\r\n");
    m2_duty_cycle = (int16_t)atof(token);

    motor_speed.m0_duty_cycle = m0_duty_cycle;
    motor_speed.m1_duty_cycle = m1_duty_cycle;
    motor_speed.m2_duty_cycle = m2_duty_cycle;
}

/**
 * @brief   Handles "OR" command
 * @details Resets odometry data
 * @note    Called within ISR context from `cmd` module
 */
void movement_handleCommandsOR(uint8_t *ptr_data, uint16_t lenght)
{
    odom_reset(&odom_handler);
}

/**
 * @brief   Updates three motors based on desired motor speed.
 * @details If "DC" command received, robot will kept in manual mode, and motors will always work.
 *          If "MS" or "RS" command received, robot will update motor speeds using PID controlled and calculate odometry
 * @note    Should be called continuously in the application layer (i.e. inside the mainloop)
 */
void movement_update()
{
    if (motor_speed.is_manually_controlled)
    {
        motor0_handler.duty_cycle = motor_speed.m0_duty_cycle;
        motor1_handler.duty_cycle = motor_speed.m1_duty_cycle;
        motor2_handler.duty_cycle = motor_speed.m2_duty_cycle;

        motor_update(&motor0_handler);
        motor_update(&motor1_handler);
        motor_update(&motor2_handler);

        printf("EF:%d:%d:%d\r\n", motor_speed.m0_duty_cycle, motor_speed.m1_duty_cycle, motor_speed.m2_duty_cycle);
    }
    else
    {
        uint32_t current_time_ms = system_hal_timestamp();
        if (current_time_ms > last_packet_time_ms + PACKET_TIMEOUT_MS)
        {
            motor_speed.motor0 = 0.0f;
            motor_speed.motor1 = 0.0f;
            motor_speed.motor2 = 0.0f;
        }

        motor0_handler.linear_velocity_setpoint = motor_speed.motor0;
        motor1_handler.linear_velocity_setpoint = motor_speed.motor1;
        motor2_handler.linear_velocity_setpoint = motor_speed.motor2;

        PID_Compute(&pid0_handler);
        PID_Compute(&pid1_handler);
        PID_Compute(&pid2_handler);

        motor_update(&motor0_handler);
        motor_update(&motor1_handler);
        motor_update(&motor2_handler);

        odom_update(&odom_handler, motor0_handler.linear_velocity, motor1_handler.linear_velocity,
                    motor2_handler.linear_velocity, (MAIN_LOOP_DT_MS / 1000.0f));
        // printOdom();
    }
}

/**
 * @brief   PWM IT handler, that is called, when PWM signal changes state from LOW to HIGH
 * @note    Called from `timerif` module (HAL_TIM_PeriodElapsedCallback)
 */
void movement_pwmHighCallback(TIM_HandleTypeDef *timer_handler)
{
    /*
    Ideally, this interrupt handler should be defined in motor module.
    But since there's no way to pass motor handler as an argument to the ISR,
    look up (timer_n <-> motorN_handler) performed here.
    */
    if (timer_handler->Instance == TIMER_PWM_M0->Instance)
    {
        ioif_writePin(&motor0_handler.pwm_pin, true);
    }
    else if (timer_handler->Instance == TIMER_PWM_M1->Instance)
    {
        ioif_writePin(&motor1_handler.pwm_pin, true);
    }
    else if (timer_handler->Instance == TIMER_PWM_M2->Instance)
    {
        ioif_writePin(&motor2_handler.pwm_pin, true);
    }
}

/**
 * @brief   PWM IT handler, that is called, when PWM signal changes state from HIGH to LOW
 * @note    Called from `timerif` module (HAL_TIM_PWM_PulseFinishedCallback)
 */
void movement_pwmLowCallback(TIM_HandleTypeDef *timer_handler)
{
    if (timer_handler->Instance == TIMER_PWM_M0->Instance)
    {
        ioif_writePin(&motor0_handler.pwm_pin, false);
    }
    else if (timer_handler->Instance == TIMER_PWM_M1->Instance)
    {
        ioif_writePin(&motor1_handler.pwm_pin, false);
    }
    else if (timer_handler->Instance == TIMER_PWM_M2->Instance)
    {
        ioif_writePin(&motor2_handler.pwm_pin, false);
    }
}

/** @brief Initializes PID controller for each motor */
static void initPID(void)
{
    double *ptr_input;
    double *ptr_output;
    double *ptr_setpoint;

    ptr_input = &(motor0_handler.linear_velocity);
    ptr_output = &(motor0_handler.duty_cycle);
    ptr_setpoint = &(motor0_handler.linear_velocity_setpoint);
    PID(&pid0_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    ptr_input = &(motor1_handler.linear_velocity);
    ptr_output = &(motor1_handler.duty_cycle);
    ptr_setpoint = &(motor1_handler.linear_velocity_setpoint);
    PID(&pid1_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    ptr_input = &(motor2_handler.linear_velocity);
    ptr_output = &(motor2_handler.duty_cycle);
    ptr_setpoint = &(motor2_handler.linear_velocity_setpoint);
    PID(&pid2_handler, ptr_input, ptr_output, ptr_setpoint, PID_KP, PID_KI, PID_KD, _PID_P_ON_E, _PID_CD_DIRECT);

    PID_SetMode(&pid0_handler, _PID_MODE_AUTOMATIC);
    PID_SetMode(&pid1_handler, _PID_MODE_AUTOMATIC);
    PID_SetMode(&pid2_handler, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&pid0_handler, -100.0f, 100.0f);
    PID_SetOutputLimits(&pid1_handler, -100.0f, 100.0f);
    PID_SetOutputLimits(&pid2_handler, -100.0f, 100.0f);
    PID_SetSampleTime(&pid0_handler, MAIN_LOOP_DT_MS);
    PID_SetSampleTime(&pid1_handler, MAIN_LOOP_DT_MS);
    PID_SetSampleTime(&pid2_handler, MAIN_LOOP_DT_MS);
}

/** @brief Prints over serial odometry data in the format "ODOM:{pos_x}:{pos_y}:{pos_z}:{vel_x}:{vel_y}:{vel_z}\r\n" */
static void printOdom(void)
{
    float pos_x = odom_handler.odom_pos_data[0];
    float pos_y = odom_handler.odom_pos_data[1];
    float pos_z = odom_handler.odom_pos_data[2];
    float vel_x = odom_handler.robot_vel_data[0];
    float vel_y = odom_handler.robot_vel_data[1];
    float vel_z = odom_handler.robot_vel_data[2];
    printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", pos_x, pos_y, pos_z, vel_x, vel_y, vel_z);
}
