#include "menu.h"
#include "ioif.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

static bool menu_right = false;
static bool menu_select = false;
static bool menu_left = false;

void menu_init(void)
{
    EXTICallbackType my_callback = (EXTICallbackType)menu_processInput;
    ioif_setRotaryEncoderCallback(my_callback);

    MX_I2C3_Init();
    ssd1306_Init();

}

void menu_processInput(uint16_t pin_number)
{
    static IoPinType enc_a;
    enc_a.ptr_port = PIN_ROT_ENC_A_GPIO_Port;
    enc_a.pin_number = PIN_ROT_ENC_A_Pin;

    if (pin_number == PIN_ENC_SW)
    {
        menu_select = true;
    }
    
    else if (pin_number == PIN_ENC_B) 
    {
        if (ioif_isActive(&enc_a))
        {
            menu_right = true;
        }

        else 
        {
            menu_left = true;
        }
    }    
}

void menu_update()
{
    if (menu_right || menu_left || menu_select)
    {
        char buff[64];
        ssd1306_SetCursor(2, 2);
        
        if (menu_select)
        {
            ssd1306_Fill(Black);
            snprintf(buff, sizeof(buff), "pressed");
            ssd1306_WriteString(buff, Font_11x18, White);
            menu_select = false;
        }

        else if (menu_left)
        {
            ssd1306_Fill(Black);
            snprintf(buff, sizeof(buff), "left");
            ssd1306_WriteString(buff, Font_11x18, White);
            menu_left = false;
        }

        else if (menu_right)
        {
            ssd1306_Fill(Black);
            snprintf(buff, sizeof(buff), "right");
            ssd1306_WriteString(buff, Font_11x18, White);
            menu_right = false;
        }
        ssd1306_UpdateScreen();
    }
}
