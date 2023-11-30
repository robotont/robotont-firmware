/**
 * @file led.c
 * @brief
 *
 * @author Raimo Köidam (raimokoidam@gmail.com)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */
#include "led.h"

#define LED_MODE_SPIN 0 //LED spin green
#define LED_MODE_PULSE 1 //Leds pulse on/off
#define LED_MODE_COLORS 2 //Change colours
#define LED_MODE_WHEEL_COLORS 3 //Behind the wheel change colours

uint8_t led_val = 0;
uint8_t led_val_increasing = 1;
uint8_t led_i = 0;
uint8_t led_mode = LED_MODE_WHEEL_COLORS;
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
    ARGB_SetRGB(index, r, g, b);
}

void led_handleCommandsLM(uint8_t *ptr_data, uint16_t lenght)
{
    ARGB_Clear();
    char *token = strtok((char *)ptr_data, "\r\n");
    led_mode = atof(token);
}

void led_update()
{
    //LED modes
    if (HAL_GPIO_ReadPin(PIN_POWEROFF_REQ_GPIO_Port, PIN_POWEROFF_REQ_Pin)) //STOP button pressed
    {
      ARGB_FillRGB(255, 0, 0); //All leds red
    }
    else
    {
      switch (led_mode)
      {
      case LED_MODE_SPIN:
        if (counter % 2 == 0) //LED spin green
        {
          ARGB_Clear();
          ARGB_SetRGB(led_i, 255, 0, 0);
          ARGB_Show();
          led_i++;
          if (led_i >= 60) led_i = 0;
        }
        break;
      case LED_MODE_PULSE:
        if (counter % 2 == 0) //Leds pulse on/off
        {
          ARGB_FillRGB(0, led_val, 0);
          ARGB_Show();
          if(led_val_increasing) {
            led_val+= 5; 
            if (led_val >= 255) led_val_increasing = 0;
          } else {
            led_val-= 5; 
            if (led_val <= 0) led_val_increasing = 1;
          }
        }
        break;
      case LED_MODE_COLORS:
        if (counter % 10 == 0)//Change colours
        {
          ARGB_FillHSV(led_val, 255, 255);
          ARGB_Show();
          if (led_val < 255) led_val += 1;
          else led_val = 0;
        }
      case LED_MODE_WHEEL_COLORS:
        if (counter % 10 == 0)//Behind the wheel change colours
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
      default:
        break;
      }
    }
    counter++;
}