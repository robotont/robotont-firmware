#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmd.h"
#include "motor.h"
#include "odom.h"
#include "peripheral.h"
#include "pid.h"
#include "sw_enc.h"
#include "system_hal.h"
#include "usbif.h"

#define MAIN_LOOP_DT_MS 10
#define CMD_TIMEOUT_MS  1000 // If velocity command is not received within this period all motors are stopped.
#define MAX_LIN_VEL     0.4  // m/s
#define MAX_ANG_VEL     1.0  // rad/s

EncoderType henc0, henc1, henc2;
MotorType hm0, hm1, hm2;
MotorCfgType mcfg0, mcfg1, mcfg2;
PID_TypeDef hPID0, hPID1, hPID2;
OdomType hodom;

int main(void)
{
    system_hal_init();
    peripheral_init();

    // Initialize software encoders
    sw_enc_init(&henc0, PIN_M0_ENCA_GPIO_Port, PIN_M0_ENCA_Pin, PIN_M0_ENCB_GPIO_Port, PIN_M0_ENCB_Pin);
    sw_enc_init(&henc1, PIN_M1_ENCA_GPIO_Port, PIN_M1_ENCA_Pin, PIN_M1_ENCB_GPIO_Port, PIN_M1_ENCB_Pin);
    sw_enc_init(&henc2, PIN_M2_ENCA_GPIO_Port, PIN_M2_ENCA_Pin, PIN_M2_ENCB_GPIO_Port, PIN_M2_ENCB_Pin);

    motor_setConfig(&mcfg0, &mcfg1, &mcfg2);
    motor_init(&hm0, &mcfg0, &henc0, &(TIM3->CCR1), &htim3);
    motor_init(&hm1, &mcfg1, &henc1, &(TIM11->CCR1), &htim11);
    motor_init(&hm2, &mcfg2, &henc2, &(TIM13->CCR1), &htim13);

    // Initialize odometry
    odom_init(&hodom, &mcfg0, &mcfg1, &mcfg2);
    cmd_init();

    // Start timers for motor PWM generation (gpios are SET in periodelapsedCallback and RESET in pulseFinishedCallback)
    HAL_TIM_Base_Start_IT(&htim3); // motor 0
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim11); // motor 1
    HAL_TIM_PWM_Start_IT(&htim11, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim13); // motor 2
    HAL_TIM_PWM_Start_IT(&htim13, TIM_CHANNEL_1);

    // Start timer for reading encoders
    HAL_TIM_Base_Start_IT(&htim14);
    // HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // uint8_t Text[] = "Hello from Robotont\r\n";
    HAL_Delay(1000);
    uint32_t pid_k = 600;
    uint32_t pid_i = 15000;
    uint32_t pid_d = 0;
    PID(&hPID0, &(hm0.linear_velocity), &(hm0.effort), &(hm0.linear_velocity_setpoint), pid_k, pid_i, pid_d,
        _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID1, &(hm1.linear_velocity), &(hm1.effort), &(hm1.linear_velocity_setpoint), pid_k, pid_i, pid_d,
        _PID_P_ON_E, _PID_CD_DIRECT);
    PID(&hPID2, &(hm2.linear_velocity), &(hm2.effort), &(hm2.linear_velocity_setpoint), pid_k, pid_i, pid_d,
        _PID_P_ON_E, _PID_CD_DIRECT);

    PID_SetMode(&hPID0, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID0, -1000, 1000);
    PID_SetSampleTime(&hPID0, MAIN_LOOP_DT_MS);
    PID_SetMode(&hPID1, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID1, -1000, 1000);
    PID_SetSampleTime(&hPID1, MAIN_LOOP_DT_MS);
    PID_SetMode(&hPID2, _PID_MODE_AUTOMATIC);
    PID_SetOutputLimits(&hPID2, -1000, 1000);
    PID_SetSampleTime(&hPID2, MAIN_LOOP_DT_MS);

    uint32_t counter = 0; // for debugging purposes
    uint32_t duty = 0;    // for debugging purposes

    uint32_t last_tick = HAL_GetTick();
    uint32_t last_vel_received_tick = HAL_GetTick();
    uint32_t delay_tick = 0;

    HAL_GPIO_WritePin(hm0.ptr_motor_config->en2_port, hm0.ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(hm1.ptr_motor_config->en2_port, hm1.ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(hm2.ptr_motor_config->en2_port, hm2.ptr_motor_config->en2_pin, RESET);
    HAL_GPIO_WritePin(hm0.ptr_motor_config->nsleep_port, hm0.ptr_motor_config->nsleep_pin, SET); // Enable drv of motor0
    HAL_GPIO_WritePin(hm1.ptr_motor_config->nsleep_port, hm1.ptr_motor_config->nsleep_pin, SET); // Enable drv of motor1
    HAL_GPIO_WritePin(hm2.ptr_motor_config->nsleep_port, hm2.ptr_motor_config->nsleep_pin, SET); // Enable drv of motor2

    counter = 1;
    duty = 50;
    while (1)
    {
        // Process data that was received over the USB virtual COM port.
        if (last_packet_length)
        {
            // printf("processing packet (%d): %s\r\n", last_packet_length, (char *) last_packet);
            // Command: RS (Robot Speed)
            if (last_packet[0] == 'R' && last_packet[1] == 'S')
            {
                last_vel_received_tick = HAL_GetTick();
                float lin_vel_x = 0;
                float lin_vel_y = 0;
                float ang_vel_z = 0;

                char *pch;
                pch = strtok((char *)last_packet, ":");
                int arg = 0;
                while (pch != NULL)
                {
                    if (arg == 1)
                    {
                        lin_vel_x = atof(pch);
                    }
                    else if (arg == 2)
                    {
                        lin_vel_y = atof(pch);
                    }
                    else if (arg == 3)
                    {
                        ang_vel_z = atof(pch);
                    }
                    pch = strtok(NULL, ":");
                    arg++;
                }

                float lin_vel_dir = atan2(lin_vel_y, lin_vel_x);
                float lin_vel_mag = sqrt(lin_vel_x * lin_vel_x + lin_vel_y * lin_vel_y);

                // Hard limit linear and angular velocities //TODO: implement min max macros to make this a 2-liner
                if (lin_vel_mag > MAX_LIN_VEL)
                {
                    lin_vel_mag = MAX_LIN_VEL;
                }
                if (ang_vel_z > MAX_ANG_VEL)
                {
                    ang_vel_z = MAX_ANG_VEL;
                }
                else if (ang_vel_z < -MAX_ANG_VEL)
                {
                    ang_vel_z = -MAX_ANG_VEL;
                }

                // Apply velocities to motors
                hm0.linear_velocity_setpoint =
                    lin_vel_mag * sin(lin_vel_dir - hm0.ptr_motor_config->wheel_pos_phi) + hm0.ptr_motor_config->wheel_pos_r * ang_vel_z;
                hm1.linear_velocity_setpoint =
                    lin_vel_mag * sin(lin_vel_dir - hm1.ptr_motor_config->wheel_pos_phi) + hm1.ptr_motor_config->wheel_pos_r * ang_vel_z;
                hm2.linear_velocity_setpoint =
                    lin_vel_mag * sin(lin_vel_dir - hm2.ptr_motor_config->wheel_pos_phi) + hm2.ptr_motor_config->wheel_pos_r * ang_vel_z;
            }
            // Command: MS (Motor Speed)
            else if (last_packet[0] == 'M' && last_packet[1] == 'S')
            {
                last_vel_received_tick = HAL_GetTick();
                char *pch;
                pch = strtok((char *)last_packet, ":");
                int arg = 0;
                while (pch != NULL)
                {
                    if (arg == 1)
                    {
                        hm0.linear_velocity_setpoint = atof(pch);
                    }
                    else if (arg == 2)
                    {
                        hm1.linear_velocity_setpoint = atof(pch);
                    }
                    else if (arg == 3)
                    {
                        hm2.linear_velocity_setpoint = atof(pch);
                    }
                    pch = strtok(NULL, ":");
                    arg++;
                }
            }
            // Command: EF (Effort control)
            else if (last_packet[0] == 'E' && last_packet[1] == 'F')
            {
                last_vel_received_tick = HAL_GetTick();
                char *pch;
                pch = strtok((char *)last_packet, ":");
                int arg = 0;
                while (pch != NULL)
                {
                    if (arg == 1)
                    {
                        hm0.effort = atof(pch);
                    }
                    else if (arg == 2)
                    {
                        hm1.effort = atof(pch);
                    }
                    else if (arg == 3)
                    {
                        hm2.effort = atof(pch);
                    }
                    pch = strtok(NULL, ":");
                    arg++;
                }
            }
            // Command: OR (Odom Reset)
            else if (last_packet[0] == 'O' && last_packet[1] == 'R')
            {
                odom_reset(&hodom);
            }
            last_packet[0] = '\0'; // indicate that packet has been processed
            last_packet_length = 0;
        }

        // Print out some debugging information at 10 lower rate.
        if (counter % 100 == 0)
        {
            HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
            // printf("M0: sp: %f; vel: %f, effort: %f\r\n", hm0.linear_velocity_setpoint, hm0.linear_velocity,
            // hm0.effort); printf("M1: sp: %f; vel: %f, effort: %f\r\n", hm1.linear_velocity_setpoint,
            // hm1.linear_velocity, hm1.effort); printf("M2: sp: %f; vel: %f, effort: %f\r\n",
            // hm2.linear_velocity_setpoint, hm2.linear_velocity, hm2.effort);
            printf("Main_delay:%ld %ld\r\n", delay_tick, last_tick);
        }

        // If no velocity command has been received within the timeout period, stop all motors
        if (HAL_GetTick() - last_vel_received_tick > CMD_TIMEOUT_MS)
        {
            hm0.linear_velocity_setpoint = 0;
            hm1.linear_velocity_setpoint = 0;
            hm2.linear_velocity_setpoint = 0;
        }

        // Update motors
        PID_Compute(&hPID0);
        PID_Compute(&hPID1);
        PID_Compute(&hPID2);
        motor_update(&hm0);
        motor_update(&hm1);
        motor_update(&hm2);
        odom_update(&hodom, hm0.linear_velocity, hm1.linear_velocity, hm2.linear_velocity, MAIN_LOOP_DT_MS / 1000.0f);

        // // Send odometry command to the on-board computer
        // printf("ODOM:%f:%f:%f:%f:%f:%f\r\n", hodom.odom_pos_data[0], hodom.odom_pos_data[1], hodom.odom_pos_data[2],
        //        hodom.robot_vel_data[0], hodom.robot_vel_data[1], hodom.robot_vel_data[2]);

        // Wait until the desired loop time has elapsed
        delay_tick = MAIN_LOOP_DT_MS - (HAL_GetTick() - last_tick);
        if (delay_tick > MAIN_LOOP_DT_MS) // check for overflow
        {
            delay_tick = 0;
        }
        HAL_Delay(delay_tick);
        last_tick = HAL_GetTick();
        counter++;
    } // end of main while loop
} // end of main

/**
 * @brief  Retargets the C library printf function to the VIRTUAL COM PORT.
 * @param  None
 * @retval None
 */
int _write(int file, char *ptr, int len)
{
    static uint8_t rc = USBD_OK;

    // loop commented out for a non-blocking behavior
    // do {
    rc = CDC_Transmit_FS((unsigned char *)ptr, len);
    //} while (USBD_BUSY == rc);
    if (USBD_FAIL == rc)
    {
        /// NOTE: Should never reach here.
        /// TODO: Handle this error.
        return 0;
    }
    return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Set PWM pin to high depending on which pwm timer triggered the interrupt
    if (htim->Instance == htim3.Instance)
    {
        HAL_GPIO_WritePin(hm0.pwm_port, hm0.pwm_pin, SET);
    }
    else if (htim->Instance == htim11.Instance)
    {
        HAL_GPIO_WritePin(hm1.pwm_port, hm1.pwm_pin, SET);
    }
    else if (htim->Instance == htim13.Instance)
    {
        HAL_GPIO_WritePin(hm2.pwm_port, hm2.pwm_pin, SET);
    }
    // Check if the timer for the encoder interrupt triggered
    else if (htim->Instance == htim14.Instance)
    {
        sw_enc_interrupt(&henc0);
        sw_enc_interrupt(&henc1);
        sw_enc_interrupt(&henc2);
    }
}

/**
 * @brief  This function is executed when the PWM pulses finish. Depending on which timer triggered the interrupt, the
 * corresponding PWM pin is set to low.
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim3.Instance) // motor 0
    {
        HAL_GPIO_WritePin(hm0.pwm_port, hm0.pwm_pin, RESET);
    }
    else if (htim->Instance == htim11.Instance) // motor 1
    {
        HAL_GPIO_WritePin(hm1.pwm_port, hm1.pwm_pin, RESET);
    }
    else if (htim->Instance == htim13.Instance) // motor 2
    {
        HAL_GPIO_WritePin(hm2.pwm_port, hm2.pwm_pin, RESET);
    }
}
