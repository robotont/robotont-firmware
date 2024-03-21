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


uint16_t led_val = 0;
uint8_t led_val_increasing = 1;
uint8_t led_i = 0;
LEDMode led_mode = LED_MODE_SPIN;
uint8_t led_mode_color[3] = { 0, 255, 0 };
uint32_t counter = 0;

void led_init()
{
    ARGB_Init();  // Initialization
    ARGB_Clear(); // Clear stirp
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
    ARGB_SetBrightness(255);
    ARGB_Clear();

    char *token = strtok(ptr_data, ":");
    int idx_start = atof(token);

    token = strtok(NULL, ":");
    int idx_end = atof(token);
    uint32_t color = 0;
    int i = idx_start;

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
    // LED modes
    switch (led_mode)
    {
        case LED_MODE_SPIN:
            if (counter % 2 == 0) // LED spin green
            {
                ARGB_Clear();
                ARGB_SetRGB(led_i, led_mode_color[0], led_mode_color[1], led_mode_color[2]);
                ARGB_Show();
                led_i++;
                if (led_i >= 60)
                    led_i = 0;
            }
            break;
        case LED_MODE_PULSE:
            if (counter % 2 == 0) // Leds pulse on/off
            {
                ARGB_SetBrightness(led_val);
                ARGB_FillRGB(led_mode_color[0], led_mode_color[1], led_mode_color[2]);
                ARGB_Show();
                if (led_val_increasing)
                {
                    led_val += 5;
                    if (led_val >= 255)
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
        case LED_MODE_COLORS_SMOOTH:
            if (counter % 10 == 0) // Change colours
            {
                ARGB_FillHSV(led_val, 255, 255);
                ARGB_Show();
                if (led_val < 255)
                    led_val += 1;
                else
                    led_val = 0;
            }
            break;
        case LED_MODE_WHEEL_COLORS:
            if (counter % 10 == 0) // Behind the wheel change colours
            {
                for (uint32_t i = 8; i < 12; i++)
                {
                    ARGB_SetHSV(i, led_val, 255, 255);
                }
                for (uint32_t i = 28; i < 32; i++)
                {
                    ARGB_SetHSV(i, led_val, 255, 255);
                }
                for (uint32_t i = 48; i < 52; i++)
                {
                    ARGB_SetHSV(i, led_val, 255, 255);
                }
                ARGB_Show();
                if (led_val < 255)
                    led_val += 1;
                else
                    led_val = 0;
            }
            break;
        case LED_MODE_COLORS_RGB:
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
        case LED_MODE_COLORS_SPIN:
            if (counter % 2 == 0) // Change colours spin
            {
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
        case LED_MODE_MOTOR_SPEEDS:
            if (counter % 10 == 0) // Behind the wheel change colours with motor speeds
            {
                uint8_t left = abs(timerif_getCounter(TIMER_ENC_M0));
                uint8_t middle = abs(timerif_getCounter(TIMER_ENC_M1));
                uint8_t right = abs(timerif_getCounter(TIMER_ENC_M2));

                for (uint32_t i = 8; i < 12; i++)
                {
                    if (timerif_getCounter(TIMER_ENC_M0) > 0) {
                        ARGB_SetRGB(i, left, 0, 0);
                    } else {
                        ARGB_SetRGB(i, 0, 0, left);
                    }
                }
                for (uint32_t i = 28; i < 32; i++)
                {
                    if (timerif_getCounter(TIMER_ENC_M1) > 0) {
                        ARGB_SetRGB(i, middle, 0, 0);
                    } else {
                        ARGB_SetRGB(i, 0, 0, middle);
                    }
                }
                for (uint32_t i = 48; i < 52; i++)
                {
                    if (timerif_getCounter(TIMER_ENC_M2) > 0) {
                        ARGB_SetRGB(i, right, 0, 0);
                    } else {
                        ARGB_SetRGB(i, 0, 0, right);
                    }
                }
                ARGB_Show();
            }
            break;
        case LED_MODE_SCAN_RANGES:
            if (counter % 10 == 0)
            {
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
        default:
            break;
    }

    counter++;
}