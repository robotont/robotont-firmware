/**
 * @file led.c
 * @brief
 *
 * @author Raimo Köidam (raimokoidam@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */
#include "led.h"
#include "ARGB.h"

#define LED_MODE_SPIN 0 //LED spin green
#define LED_MODE_PULSE 1 //Leds pulse on/off
#define LED_MODE_COLORS_SMOOTH 2 //Change colours
#define LED_MODE_WHEEL_COLORS 3 //Behind the wheel change colours
#define LED_MODE_COLORS_RGB 4 //Switch all leds between red, green, blue
#define LED_MODE_COLORS_SPIN 5 //Change colours spin

uint16_t led_val = 0;
uint8_t led_val_increasing = 1;
uint8_t led_i = 0;
uint8_t led_mode = LED_MODE_COLORS_SPIN;
uint8_t led_mode_color[3] = {0, 255, 0};
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

void led_update()
{
    //LED modes
    switch (led_mode)
    {
    case LED_MODE_SPIN:
      if (counter % 2 == 0) // LED spin green
      {
        ARGB_Clear();
        ARGB_SetRGB(led_i, led_mode_color[0], led_mode_color[1], led_mode_color[2]);
        ARGB_Show();
        led_i++;
        if (led_i >= 60) led_i = 0;
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
          led_val+= 5; 
          if (led_val >= 255) {
            led_val = 255;
            led_val_increasing = 0;
          }
        }
        else 
        {
          led_val-= 5; 
          if (led_val <= 0) {
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
        if (led_val < 255) led_val += 1;
        else led_val = 0;
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
        if (led_val < 255) led_val += 1;
        else led_val = 0;
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
        if (led_val < 255) led_val += 1;
        else led_val = 0;
        led_i++;
        if (led_i >= 60) led_i = 0;
      }
      break;
    default:
      break;
    
    }
    
    counter++;
}