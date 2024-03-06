/**
 * Includes
 * Static defines
 * Static type definitions
 * Static constants
 * Static variables
 * Static function prototypes
 * 
 * Public function definitions
 * Static function definitions
 * */

// TODO implement led mode switching
// TODO implement changing max speed
// TODO make dashboard nicer
// TODO implement debug screen
// TODO implement sending commands to NUC
// TODO implement firmware info screen
// TODO show battery voltage
// TODO show current draw info
// TODO show ESTOP state
// TODO show power path info
// TODO demo program submenu
// TODO PID parameters tuning screen
// TODO network info screen
// TODO measure robot performance with unoptimised menu

#include "menu.h"
#include "ioif.h"
// #include "led.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define BORDER_BEGIN_X 0
#define BORDER_BEGIN_Y 0
#define BORDER_END_X 120
#define FIELD_HEIGHT 21 // in pixels
#define MAX_MENUITEMS 16

ItemPosition border_position = ITEM_TOP;
MenuType current_menu = MENU_ROOT;
MenuState menu_state = STATE_DASHBOARD;

static bool is_input_clockwise = false;
static bool is_input_select = false;
static bool is_input_counterclockwise = false;

int item_index = 0;

int dummy = 10;

static MenuItem menu[][MAX_MENUITEMS] = 
{
    // ROOT
    {
        {"^-- Dashboard", &showDashboard},
        {"Submenu 1", &enterSubmenu, NULL, MENU_SUBMENU1},
        {"Set max speed", &setValue, &dummy},
        {"LED settings", &enterSubmenu, NULL, MENU_LED_SETTINGS},
        {"Dummy 1.3", &doNothing},
        {"Dummy 1.4", &doNothing},
        {"12345678901234567", &doNothing},
        {"12345678901234567", &doNothing}
    },
    // SUBMENU 1
    {
        {"^-- Main menu", &enterMainmenu},
        {"Submenu 2", &enterSubmenu, NULL, MENU_SUBMENU2},
        {"Item 2.2", &doNothing},
        {"Item 2.3", &doNothing}
    },
    // SUBMENU 2
    {
        {"^-- Main menu", &enterMainmenu},
        {"Item 3.1", &doNothing},
        {"Item 3.2", &doNothing},
        {"Item 3.3", &doNothing}
    },
    // LED SETTINGS
    {
        {"^-- Main menu", &enterMainmenu},
        {"MODE_SPIN", &setLEDMode},
        {"MODE_PULSE", &setLEDMode},
        {"MODE_COLORS_SMOOTH", &setLEDMode},
        {"MODE_WHEEL_COLORS", &setLEDMode},
        {"MODE_COLORS_RGB", &setLEDMode},
        {"MODE_COLORS_SPIN", &setLEDMode},
        {"MODE_MOTOR_SPEEDS", &setLEDMode}, 
        {"MODE_SCAN_RANGES", &setLEDMode}
    },

};

static void setLEDMode()
{
    // led_mode = item_index - 1;
}


static void enterMainmenu()
{
    menu_state = STATE_MENU;
    current_menu = MENU_ROOT;
    item_index = 0;
    border_position = ITEM_TOP;
}

static void setValue()
{
    menu_state = STATE_INPUT;  
}

static void showDashboard()
{
    menu_state = STATE_DASHBOARD;
}

static void doNothing()
{
    return;
}

static void enterSubmenu() {
    current_menu = menu[current_menu][item_index].submenu_index;
    item_index = 0;
    border_position = ITEM_TOP;
}

static void drawBorder(int border_position)
{
    ssd1306_DrawRectangle(BORDER_BEGIN_X, BORDER_BEGIN_Y + FIELD_HEIGHT * border_position,
                          BORDER_END_X, (border_position+1) * FIELD_HEIGHT, White);
}

static int getCurrentMenuSize()
{
    for (int size_counter = 0; size_counter < MAX_MENUITEMS; size_counter++)
    {
        if (menu[current_menu][size_counter].label == NULL) 
        {
           return size_counter;
        }
    }
    return MAX_MENUITEMS;
}

static void drawScrollbar()
{
    // divide vertical space between items
    int pixelsPerItem = SSD1306_HEIGHT / getCurrentMenuSize();

    ssd1306_FillRectangle(124, (item_index-border_position)*pixelsPerItem, 128, (item_index - border_position + 3) * pixelsPerItem, White);
}

static void drawDashboard() 
{
    // useful items while operating the robot
    // * battery voltage
    // * current ROS program
    // *  
    ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 3);
    ssd1306_WriteString("Bat volt: 14.6 V", Font_6x8, White);

    ssd1306_SetCursor(2, 15);
    char buff[64];
    snprintf(buff, sizeof(buff), "Max speed: %d", dummy);
    ssd1306_WriteString(buff, Font_6x8, White);

    ssd1306_SetCursor(2, 27);
    ssd1306_WriteString("IP:123.123.123.123", Font_6x8, White);

    ssd1306_SetCursor(2, 39);
    ssd1306_WriteString("press knob 4 menu", Font_6x8, White);

    ssd1306_SetCursor(2, 51);
    ssd1306_WriteString("LED mode: blink", Font_6x8, White);
}

static void drawMenuItems() 
{
    ssd1306_Fill(Black);
    
    // max label length 17 chars
    // or 19 with smallest font

    // top item
    ssd1306_SetCursor(2, 6);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_TOP - border_position].label, Font_7x10, White);

    // center item
    ssd1306_SetCursor(2, 6+21);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_CENTER - border_position].label, Font_7x10, White);

    // bottom item
    ssd1306_SetCursor(2, 6+21+21);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_BOTTOM - border_position].label, Font_7x10, White);

    drawBorder(border_position);
    drawScrollbar();
}

static void drawInputScreen() {
    ssd1306_Fill(Black);

    // draw variable name
    ssd1306_SetCursor(2, 3);
    ssd1306_WriteString(menu[current_menu][item_index].label, Font_7x10, White);

    // draw variable value
    char buff[64];
    snprintf(buff, sizeof(buff), "%d", *(menu[current_menu][item_index].ptr_valuetochange));
    ssd1306_SetCursor(48, 30);
    ssd1306_WriteString(buff, Font_16x24, White);
}

static void inputHandler(uint16_t pin_number)
{
    static IoPinType enc_a;
    enc_a.ptr_port = PIN_ROT_ENC_A_GPIO_Port;
    enc_a.pin_number = PIN_ROT_ENC_A_Pin;

    if (pin_number == PIN_ENC_SW)
    {
        is_input_select = true;
    }
    
    else if (pin_number == PIN_ENC_B) 
    {
        if (ioif_isActive(&enc_a))
        {
            is_input_clockwise = true;
        }

        else 
        {
            is_input_counterclockwise = true;
        }
    }    
}

static void inputHandlerDashboard()
{
    // enter main menu
    if (is_input_select)
    {
        enterMainmenu();
        is_input_select = false;
    }
}

static void inputHandlerMenu()
{
    // run current menu item callback 
    if (is_input_select)
    {
        menu[current_menu][item_index].select_callback();
        is_input_select = false;
    }

    // select upper item
    else if (is_input_counterclockwise)
    {
        if (item_index > 0)
        {
            item_index--;

            if (border_position != ITEM_TOP)
            {
                border_position--;
            }
        }

        is_input_counterclockwise = false;
    }

    // select lower item
    else if (is_input_clockwise)
    {   
        if (item_index < getCurrentMenuSize() - 1)
        {
            item_index++;

            if (border_position != ITEM_BOTTOM)
            {
                border_position++;
            }
        }

        is_input_clockwise = false;
    }
}

static void inputHandlerInput()
{
    // return to previous menu
    // leaves other menu specific variables unchanged
    if (is_input_select)
    {
        menu_state = STATE_MENU;
        is_input_select = false;
    }

    // decrement variable
    else if (is_input_counterclockwise)
    {
        (*(menu[current_menu][item_index].ptr_valuetochange))--;
        is_input_counterclockwise = false;
    }

    // increment variable
    else if (is_input_clockwise)
    {   
        (*(menu[current_menu][item_index].ptr_valuetochange))++;
        is_input_clockwise = false;
    }
}

void menu_init()
{
    EXTICallbackType my_callback = (EXTICallbackType)inputHandler;
    ioif_setRotaryEncoderCallback(my_callback);

    menu_state = STATE_DASHBOARD;

    MX_I2C3_Init();
    ssd1306_Init();
}

// frequency of calling this depends on main(). possibly optimize this, if updating menu is too resource intensive
// dashboard updating should happen regularily whatever the robot is doing (for example moving around)
void menu_update()
{
    if (menu_state == STATE_DASHBOARD)
    {
        inputHandlerDashboard();
        drawDashboard();   
    }

    else if (menu_state == STATE_MENU) 
    {
        inputHandlerMenu();
        drawMenuItems();
    }

    else if (menu_state == STATE_INPUT)
    {
        inputHandlerInput();
        drawInputScreen();
    }

    ssd1306_UpdateScreen();
}
