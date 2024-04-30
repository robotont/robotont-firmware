/**
 * @file led.c
 * @brief
 *
 * @author Raimo Köidam (raimokoidam@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

#include "led.h"
#include "timerif.h"
#include "ARGB.h"
#include "measurements.h"
#include "ioif.h"
#include "movement.h"

#define MAX_BAT_VOLTAGE 18
#define LOW_BAT_VOLTAGE 13
#define BATTERY_V_DISPLAY_TIME 100
#define max(x, y) (x > y) ? x : y
#define min(x, y) (x < y) ? x : y

int led_val = 0;
uint8_t led_val_increasing = 1;
uint8_t led_i = 0;
LEDMode led_mode = MOTOR_DUTY;
uint8_t led_mode_color[3] = { 0, 255, 0 };
uint32_t counter = 0;
static IoPinType estop;

void led_init()
{
    estop.ptr_port = PIN_ESTOP_GPIO_Port;
    estop.pin_number = PIN_ESTOP_Pin;

    ARGB_Init();  // Initialization
    ARGB_Clear(); // Clear stirp

    // Clamp battery level between low batery voltage and max battery voltage
    float current_bat_v = max(min(BatVoltage, MAX_BAT_VOLTAGE), LOW_BAT_VOLTAGE);
    // Show battery level
    ARGB_FillRGB(255 - ((MAX_BAT_VOLTAGE - current_bat_v) / (MAX_BAT_VOLTAGE - LOW_BAT_VOLTAGE)) * 255, ((MAX_BAT_VOLTAGE - current_bat_v) / (MAX_BAT_VOLTAGE - LOW_BAT_VOLTAGE)) * 255, 0);

    while (ARGB_Show() != ARGB_OK);
}

void led_handleCommandsLD(uint8_t *ptr_data, uint16_t lenght)
{
    char *token = strtok((char *)ptr_data, ":");
    int index = atof(token);
    token = strtok(NULL, ":");
    int r = atof(token);
    token = strtok(NULL, ":");
    int g = atof(token);
    token = strtok(NULL, "\r\n");
    int b = atof(token);
    ARGB_SetBrightness(255);
    ARGB_SetRGB(index, r, g, b);
}

void led_handleCommandsLM(uint8_t *ptr_data, uint16_t lenght)
{
    ARGB_SetBrightness(255);
    ARGB_Clear();
    char *token = strtok((char *)ptr_data, ":");
    led_mode = atof(token);
    token = strtok(NULL, ":");
    led_mode_color[0] = atof(token);
    token = strtok(NULL, ":");
    led_mode_color[1] = atof(token);
    token = strtok(NULL, "\r\n");
    led_mode_color[2] = atof(token);
}

void led_handleCommandsLS(uint8_t *ptr_data, uint16_t lenght)
{
    char *token = strtok(ptr_data, ":");
    int idx_start = atof(token);

    token = strtok(NULL, ":");
    int idx_end = atof(token);
    uint32_t color = 0;
    int i = idx_start;

    ARGB_SetBrightness(255);
    while (i <= idx_end)
    {
        token = strtok(NULL, ":");
        if (token != NULL)
        {
            color = atoi(token);
        }
        ARGB_SetRGB(i, color >> 16, (color & 0x00FF00) >> 8, color & 0x0000FF);
        i++;
    }
    ARGB_Show();
}

void led_update()
{
    if (counter >= BATTERY_V_DISPLAY_TIME)
    {
        if (BatVoltage <= LOW_BAT_VOLTAGE) // Blink red if battery empty
        {
            led_mode = PULSE;
            led_mode_color[0] = 255;
            led_mode_color[1] = 0;
            led_mode_color[2] = 0;
        }
        // LED modes
        if (counter == BATTERY_V_DISPLAY_TIME)
        { 
            ARGB_Clear();
            ARGB_Show();
        }
        else if (ioif_isActive(&estop)) {
            led_mode = PULSE;
            led_mode_color[0] = 255;
            led_mode_color[1] = 255;
            led_mode_color[2] = 0;
        }
        switch (led_mode)
        {
            case SPIN:
                if (counter % 2 == 0) // LED spin green
                {
                    ARGB_SetBrightness(255);
                    ARGB_Clear();
                    ARGB_SetRGB(led_i, led_mode_color[0], led_mode_color[1], led_mode_color[2]);
                    ARGB_Show();
                    led_i++;
                    if (led_i >= 60)
                        led_i = 0;
                }
                break;
            case PULSE:
                if (counter % 2 == 0) // Leds pulse on/off
                {
                    ARGB_SetBrightness(led_val);
                    ARGB_FillRGB(led_mode_color[0], led_mode_color[1], led_mode_color[2]);
                    ARGB_Show();
                    if (led_val_increasing)
                    {
                        led_val += 5;
                        if (led_val > 255)
                        {
                            led_val = 255;
                            led_val_increasing = 0;
                        }
                    }
                    else
                    {
                        led_val -= 5;
                        if (led_val <= 0)
                        {
                            led_val = 0;
                            led_val_increasing = 1;
                        }
                    }
                }
                break;
            case COLORS_SMOOTH:
                if (counter % 10 == 0) // Change colours
                {
                    ARGB_SetBrightness(255);
                    ARGB_FillHSV(led_val, 255, 255);
                    ARGB_Show();
                    if (led_val < 255)
                        led_val += 1;
                    else
                        led_val = 0;
                }
                break;
            case WHEEL_COLORS:
                if (counter % 10 == 0) // Behind the wheel change colours
                {
                    ARGB_SetBrightness(255);
                    ARGB_Clear();
                    for (uint32_t i = 6; i < 14; i++)
                    {
                        ARGB_SetHSV(i, led_val, 255, 255);
                    }
                    for (uint32_t i = 26; i < 34; i++)
                    {
                        ARGB_SetHSV(i, led_val, 255, 255);
                    }
                    for (uint32_t i = 46; i < 54; i++)
                    {
                        ARGB_SetHSV(i, led_val, 255, 255);
                    }
                    ARGB_Show();
                    if (led_val < 255)
                        led_val += 5;
                    else
                        led_val = 0;
                }
                break;
            case COLORS_RGB:
                ARGB_SetBrightness(255);
                if (counter % 600 == 0) // Switch all leds between red, green, blue
                {
                    ARGB_FillRGB(255, 0, 0);
                }
                else if (counter % 400 == 0)
                {
                    ARGB_FillRGB(0, 255, 0);
                }
                else if (counter % 200 == 0)
                {
                    ARGB_FillRGB(0, 0, 255);
                }
                ARGB_Show();
                break;
            case COLORS_SPIN:
                if (counter % 2 == 0) // Change colours spin
                {
                    ARGB_SetBrightness(255);
                    ARGB_SetHSV(led_i, led_val, 255, 255);
                    ARGB_Show();
                    if (led_val < 255)
                        led_val += 1;
                    else
                        led_val = 0;
                    led_i++;
                    if (led_i >= 60)
                        led_i = 0;
                }
                break;
            case MOTOR_DUTY:
                if (counter % 10 == 0) // Behind the wheel change colours with motor speeds
                {
                    ARGB_SetBrightness(255);
                    ARGB_Clear();
                    int left = max(min(motor0_handler.duty_cycle, 100), -100) / 100.0f * 255;
                    int middle = max(min(motor1_handler.duty_cycle, 100), -100) / 100.0f * 255;
                    int right = max(min(motor2_handler.duty_cycle, 100), -100) / 100.0f * 255;

                    for (uint32_t i = 6; i < 14; i++)
                    {
                        if (left > 0) {
                            ARGB_SetRGB(i, abs(left), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(left));
                        }
                    }
                    for (uint32_t i = 26; i < 34; i++)
                    {
                        if (middle > 0) {
                            ARGB_SetRGB(i, abs(middle), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(middle));
                        }
                    }
                    for (uint32_t i = 46; i < 54; i++)
                    {
                        if (right > 0) {
                            ARGB_SetRGB(i, abs(right), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(right));
                        }
                    }
                    ARGB_Show();
                }
                break;
            case MOTOR_SPEEDS:
                if (counter % 10 == 0) // Behind the wheel change colours with motor speeds
                {
                    ARGB_SetBrightness(255);
                    ARGB_Clear();
                    int left = max(min(motor0_handler.linear_velocity * 255, 255), -255);
                    int middle = max(min(motor1_handler.linear_velocity * 255, 255), -255);
                    int right = max(min(motor2_handler.linear_velocity * 255, 255), -255);

                    for (uint32_t i = 6; i < 14; i++)
                    {
                        if (left > 0) {
                            ARGB_SetRGB(i, abs(left), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(left));
                        }
                    }
                    for (uint32_t i = 26; i < 34; i++)
                    {
                        if (middle > 0) {
                            ARGB_SetRGB(i, abs(middle), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(middle));
                        }
                    }
                    for (uint32_t i = 46; i < 54; i++)
                    {
                        if (right > 0) {
                            ARGB_SetRGB(i, abs(right), 0, 0);
                        } else {
                            ARGB_SetRGB(i, 0, 0, abs(right));
                        }
                    }
                    ARGB_Show();
                }
                break;
            case SCAN_RANGES:
                if (counter % 10 == 0)
                {
                    ARGB_SetBrightness(255);
                    ARGB_Clear();
                    // Left
                    for (uint32_t i = 3; i < 6; i++)
                    {
                        ARGB_SetRGB(i, 255 - led_mode_color[0], led_mode_color[0], 0);
                    }
                    // Right
                    for (uint32_t i = 54; i < 57; i++)
                    {
                        ARGB_SetRGB(i, 255 - led_mode_color[1], led_mode_color[1], 0);
                    }
                    // Front
                    for (uint32_t i = 0; i < 3; i++)
                    {
                        ARGB_SetRGB(i, 255 - led_mode_color[2], led_mode_color[2], 0);
                    }
                    for (uint32_t i = 57; i < 60; i++)
                    {
                        ARGB_SetRGB(i, 255 - led_mode_color[2], led_mode_color[2], 0);
                    }
                    ARGB_Show();
                }
                break;
            case NONE:
                break;
            default:
                break;
        }
    }
    counter++;
}