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

// TODO remake menu so it shows submenu names and depth on top or bottom NO SPACE...
// TODO make dashboard nicer
// TODO make led mode selection modular
// TODO implement debug screen
// TODO implement sending commands to NUC
// TODO implement firmware info screen

// TODO show power path info incl battery voltage, current draw info
// TODO show ESTOP state
// TODO demo program submenu
// TODO PID parameters tuning screen
// TODO network info screen
// TODO make input screen nicer

#include "menu.h"
#include "ioif.h"
#include "measurements.h"
#include "led.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define BORDER_BEGIN_X 0
#define BORDER_BEGIN_Y 0
#define BORDER_WIDTH 120

#define SCROLLBAR_BEGIN_X 124
#define SCROLLBAR_WIDTH 4

#define FIELD_HEIGHT 21

#define MAX_MENUITEMS 16
#define MAX_MENUITEM_LABEL_LENGTH 16

#define SCROLLING_WAIT_IN_MAIN_LOOP_DT_INCREMENTS 25
#define SCROLLING_CONTINUE_IN_MAIN_LOOP_DT_INCREMENTS 5

ItemPosition border_position = ITEM_TOP;
MenuType current_menu = MENU_ROOT;
MenuState menu_state = STATE_DASHBOARD;

static bool is_input_clockwise = false;
static bool is_input_select = false;
static bool is_input_counterclockwise = false;

int scrolling_main_loop_counter = 0;
int scrolling_label_index = 0;
bool scrolling_activated = false;

int menu_item_index = 0;

int dummy = 10;

// All submenus have to have the same index in menu[] array as their counterparts in MenuType enum
static MenuItem menu[][MAX_MENUITEMS] = 
{
    // ROOT
    {
        {"^-- Dashboard", &showDashboard},
        {"LED modes", &enterSubmenu, NULL, MENU_LED_SETTINGS},
        {"Demo submenu 1", &enterSubmenu, NULL, MENU_SUBMENU1},
        {"Set max speed", &setValue, &dummy},
        {"scrolling demo 1 scrolling demo 2 scrolling demo 3 scrolling demo 4", &doNothing}
    },
    // SUBMENU 1
    {
        {"^-- Main menu", &enterMainmenu},
        {"Demo submenu 2", &enterSubmenu, NULL, MENU_SUBMENU2},
        {"Demo item 2.2", &doNothing},
        {"Demo item 2.3", &doNothing}
    },
    // SUBMENU 2
    {
        {"^-- Main menu", &enterMainmenu},
        {"Demo item 3.1", &doNothing},
        {"Demo item 3.2", &doNothing},
        {"Demo item 3.3", &doNothing}
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

// This implies that LedModes enum has same ordering as LED settings submenu
static void setLEDMode()
{
    led_mode = menu_item_index - 1;
}

static void enterMainmenu()
{
    menu_state = STATE_MENU;
    current_menu = MENU_ROOT;
    menu_item_index = 0;
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
    current_menu = menu[current_menu][menu_item_index].submenu_index;
    menu_item_index = 0;
    border_position = ITEM_TOP;
}

static void drawBorder(int border_position)
{
    ssd1306_DrawRect(BORDER_BEGIN_X, BORDER_BEGIN_Y + FIELD_HEIGHT * border_position, BORDER_WIDTH, FIELD_HEIGHT);
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

    ssd1306_FillRect(SCROLLBAR_BEGIN_X, (menu_item_index - border_position) * pixelsPerItem, 4, pixelsPerItem * 3 + 2);
}

static void drawDashboard() 
{
    ssd1306_Clear();
    char buff[64];

    ssd1306_SetCursor(2, 3);
    snprintf(buff, sizeof(buff), "Bat volt: %.1f V", BatVoltage);
    ssd1306_WriteString(buff, Font_7x10);

    ssd1306_SetCursor(2, 15);
    snprintf(buff, sizeof(buff), "Max speed: %d", dummy);
    ssd1306_WriteString(buff, Font_7x10);

    ssd1306_SetCursor(2, 27);
    ssd1306_WriteString("IP:123.123.123.123", Font_7x10);

    ssd1306_SetCursor(2, 51);
    ssd1306_WriteString("LED mode: blink", Font_7x10);
}

static void drawMenuItems() 
{
    ssd1306_Clear();

    for (uint8_t item_pos = 0; item_pos < 3; item_pos++)
    {
        ssd1306_SetCursor(4, 6 + item_pos * 21);
        // if label too long
        if (strlen(menu[current_menu][menu_item_index + item_pos - border_position].label) > MAX_MENUITEM_LABEL_LENGTH)
        {
            char buffer_label[MAX_MENUITEM_LABEL_LENGTH + 1];
            // if item selected
            if (item_pos == border_position)
            {
                // do scrolling

                // if scrolled to end
                if (strlen(menu[current_menu][menu_item_index + item_pos - border_position].label) < MAX_MENUITEM_LABEL_LENGTH + scrolling_label_index)
                {
                    // only show end
                    strncpy(buffer_label, menu[current_menu][menu_item_index + item_pos - border_position].label +
                    strlen(menu[current_menu][menu_item_index + item_pos - border_position].label) - MAX_MENUITEM_LABEL_LENGTH, MAX_MENUITEM_LABEL_LENGTH);
                }

                // else keep scrolling 
                else
                {
                    strncpy(buffer_label, menu[current_menu][menu_item_index + item_pos - border_position].label + scrolling_label_index, MAX_MENUITEM_LABEL_LENGTH);
                }
                
            }

            else
            {
                // just show beginning
                strncpy(buffer_label, menu[current_menu][menu_item_index + item_pos - border_position].label, MAX_MENUITEM_LABEL_LENGTH);
            }
            buffer_label[MAX_MENUITEM_LABEL_LENGTH] = '\0';

            ssd1306_WriteString(buffer_label, Font_7x10);
        }
        // label fits
        else
        {
            ssd1306_WriteString(menu[current_menu][menu_item_index + item_pos - border_position].label, Font_7x10);
        }
    }

    drawBorder(border_position);
    drawScrollbar();
}

static void drawInputScreen() {
    ssd1306_Clear();

    // draw variable name
    ssd1306_SetCursor(5, 15);
    ssd1306_WriteString(menu[current_menu][menu_item_index].label, Font_7x10);

    // draw variable value
    char buff[64];
    snprintf(buff, sizeof(buff), "%d", *(menu[current_menu][menu_item_index].ptr_input_value));
    ssd1306_SetCursor(48, 30);
    ssd1306_WriteString(buff, Font_16x26);
}

static void hardwareInputHandler(uint16_t pin_number)
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
    if (is_input_select)
    {
        enterMainmenu();
        is_input_select = false;
    }
}

static void inputHandlerMenu()
{
    if (scrolling_activated)
    {
        if (scrolling_main_loop_counter == SCROLLING_CONTINUE_IN_MAIN_LOOP_DT_INCREMENTS)
        {
            scrolling_label_index++;
            scrolling_main_loop_counter = 0;
        }
    }

    else
    {
        if (scrolling_main_loop_counter == SCROLLING_WAIT_IN_MAIN_LOOP_DT_INCREMENTS)
        {
            scrolling_activated = true;
            scrolling_main_loop_counter = 0;
        }
    }

    if (is_input_select)
    {
        menu[current_menu][menu_item_index].item_callback();
        is_input_select = false;
    }

    else if (is_input_counterclockwise)
    {
        if (menu_item_index > 0)
        {
            menu_item_index--;

            if (border_position != ITEM_TOP)
            {
                border_position--;
            }
        }

        scrolling_main_loop_counter = 0;
        scrolling_label_index = 0;
        scrolling_activated = false;

        is_input_counterclockwise = false;
    }

    else if (is_input_clockwise)
    {   
        if (menu_item_index < getCurrentMenuSize() - 1)
        {
            menu_item_index++;

            if (border_position != ITEM_BOTTOM)
            {
                border_position++;
            }
        }

        scrolling_main_loop_counter = 0;
        scrolling_label_index = 0;
        scrolling_activated = false;

        is_input_clockwise = false;
    }
}

static void inputHandlerInput()
{
    if (is_input_select)
    {
        menu_state = STATE_MENU;
        is_input_select = false;
    }

    else if (is_input_counterclockwise)
    {
        (*(menu[current_menu][menu_item_index].ptr_input_value))--;
        is_input_counterclockwise = false;
    }

    else if (is_input_clockwise)
    {   
        (*(menu[current_menu][menu_item_index].ptr_input_value))++;
        is_input_clockwise = false;
    }
}

void menu_init()
{
    ioif_setRotaryEncoderCallback((EXTICallbackType) hardwareInputHandler);

    menu_state = STATE_DASHBOARD;

    ssd1306_Init();
    ssd1306_FlipScreenVertically();
    ssd1306_SetColor(White);
}

void menu_update()
{
    if (menu_state == STATE_DASHBOARD)
    {
        inputHandlerDashboard();
        drawDashboard();   
    }

    else if (menu_state == STATE_MENU) 
    {
        scrolling_main_loop_counter++;
        inputHandlerMenu();
        drawMenuItems();
    }

    else if (menu_state == STATE_INPUT)
    {
        inputHandlerInput();
        drawInputScreen();
    }

    if (ssd1306_UpdateScreenCompleted())
    {
        ssd1306_UpdateScreen();
    }
}
