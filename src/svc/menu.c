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

// TODO make dashboard nicer
// TODO implement sending commands to NUC
// TODO make helper functions for displaying info
// TODO implement drawScrollingText function
// TODO show ESTOP state
// TODO demo program submenu
// TODO racing and normal mode callbacks
// TODO network info screen ???

#include "menu.h"
#include "peripheral.h"
#include "movement.h"
#include "ioif.h"
#include "timerif.h"
#include "measurements.h"
#include "led.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define BORDER_BEGIN_X 0
#define BORDER_BEGIN_Y 0
#define BORDER_WIDTH 120
#define FIELD_HEIGHT 21

#define MENU_ITEM_LABEL_BEGIN_X 4
#define MENU_ITEM_LABEL_OFFSET_Y 6

#define SCROLLBAR_BEGIN_X 124
#define SCROLLBAR_WIDTH 4

#define MAX_MENUITEMS 16
#define MAX_MENUITEM_LABEL_LENGTH 16

#define SCROLLING_WAIT_IN_MAIN_LOOP_DT_INCREMENTS 25
#define SCROLLING_CONTINUE_IN_MAIN_LOOP_DT_INCREMENTS 5

typedef enum 
{
    MENU_NONE = -1,
    MENU_ROOT,
    MENU_LED_SETTINGS,
    MENU_MOTOR_SETTINGS,
    MENU_DEMO_SUBMENU1,
    MENU_DEMO_SUBMENU2,
} MenuType;

typedef enum 
{
    ITEM_TOP,
    ITEM_CENTER,
    ITEM_BOTTOM
} ItemPosition;

typedef enum 
{
    STATE_DASHBOARD,
    STATE_MENU,
    STATE_INPUT,
    STATE_DEBUG
} MenuState;

typedef struct
{
    char *label;
    void (*item_callback)(void);
    MenuType menu_to_enter;
    int *ptr_dynamic_value;
} MenuItem;

static MenuType current_menu = MENU_ROOT;
static MenuState menu_state = STATE_DASHBOARD;
static ItemPosition border_position = ITEM_TOP;

static bool is_input_select = false;
static bool is_input_clockwise = false;
static bool is_input_counterclockwise = false;

static int scrolling_main_loop_counter = 0;
static int scrolling_label_index = 0;
static bool is_scrolling_activated = false;

static int menu_item_index = 0;

static int dummy = 10;
static int *ptr_user_input_value;

// ================ BEGIN DECLARATIONS ================

// BEGIN MENU CALLBACKS
static void enterMainMenu();
static void showDashboard();
static void enterSubmenu();
static void setValue();
static void setLEDMode();
static void showMotorSpeeds();
static void doNothing();
static void showFirmwareInfo();
static void showPowerInfo();
// END MENU CALLBACKS

// BEGIN DRAWING FUNCTIONS
static void drawDashboard();
static void drawScrollbar();
static void drawBorder(int borderIndex);
static void drawMenuItems();
static void drawInputScreen();
static void drawDebugScreen();
// END DRAWING FUNCTIONS

// BEGIN INPUT HANDLERS
static void hardwareInputHandler(uint16_t pin_number);
static void dashboardInputHandler();
static void menuInputHandler();
static void userInputInputHandler();
static void inputHandlerDebug();
static void clearInputs();
// END INPUT HANDLERS

static int getCurrentMenuSize();

// ================ END DECLARATIONS ================

// NOTE: All submenus have to have the same index in menu[] array as their counterparts in MenuType enum
static MenuItem menu[][MAX_MENUITEMS] = 
{
    // ROOT
    {
        {"^-- Dashboard", &showDashboard},
        {"LED modes", &enterSubmenu, MENU_LED_SETTINGS},
        {"Motor control settings", &enterSubmenu, MENU_MOTOR_SETTINGS},
        {"Demo submenu 1", &enterSubmenu, MENU_DEMO_SUBMENU1},
        {"Set max speed", &setValue, MENU_NONE, &dummy},
        {"scrolling demo 1 scrolling demo 2 scrolling demo 3 scrolling demo 4", &doNothing},
        {"Motor speeds", &showMotorSpeeds},
        {"Power information", &showPowerInfo},
        {"Firmware information", &showFirmwareInfo},
    },
    // LED SETTINGS
    {
        {"^-- Main menu", &enterMainMenu},
        {"MODE_SPIN", &setLEDMode},
        {"MODE_PULSE", &setLEDMode},
        {"MODE_COLORS_SMOOTH", &setLEDMode},
        {"MODE_WHEEL_COLORS", &setLEDMode},
        {"MODE_COLORS_RGB", &setLEDMode},
        {"MODE_COLORS_SPIN", &setLEDMode},
        {"MODE_MOTOR_SPEEDS", &setLEDMode}, 
        {"MODE_SCAN_RANGES", &setLEDMode},
    },
    // MOTOR SETTINGS
    {   
        {"^-- Main menu", &enterMainMenu},
        {"Activate racing mode", &doNothing},
        {"Activate normal mode", &doNothing},
        {"Set motor linear velocity", &setValue, MENU_NONE, &dummy},
        {"Set motor angular velocity",  &setValue, MENU_NONE, &dummy},
        {"Set motor effort", &setValue, MENU_NONE, &dummy},
    },
    // DEMO SUBMENU 1
    {
        {"^-- Main menu", &enterMainMenu},
        {"Demo submenu 2", &enterSubmenu, MENU_DEMO_SUBMENU2},
        {"Demo item 2.1", &doNothing},
        {"Demo item 2.2", &doNothing},
        {"Demo item 2.3", &doNothing},
        {"Demo item 2.4", &doNothing},
        {"Demo item 2.5", &doNothing},
        {"Demo item 2.6", &doNothing},
        {"Demo item 2.7", &doNothing},
        {"Demo item 2.8", &doNothing},
        {"Demo item 2.9", &doNothing},
    },
    // DEMO SUBMENU 2
    {
        {"^-- Demo submenu 1", &enterSubmenu, MENU_DEMO_SUBMENU1},
        {"Demo item 3.1", &doNothing},
        {"Demo item 3.2", &doNothing},
        {"Demo item 3.3", &doNothing},
    },
};

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
    switch (menu_state)
    {
        case STATE_DASHBOARD:
            drawDashboard();
            dashboardInputHandler();
            break;

        case STATE_MENU:
            drawMenuItems();
            menuInputHandler();
            scrolling_main_loop_counter++;
            break;
        
        case STATE_INPUT:
            drawInputScreen();
            userInputInputHandler();
            break;

        case STATE_DEBUG:
            drawDebugScreen();
            inputHandlerDebug();
            break;
    }

    clearInputs();

    if (ssd1306_UpdateScreenCompleted())
    {
        ssd1306_UpdateScreen();
    }
}

// ================ BEGIN MENU ITEM CALLBACKS ================

static void showDashboard()
{
    menu_state = STATE_DASHBOARD;
}

static void enterMainMenu()
{
    menu_state = STATE_MENU;
    current_menu = MENU_ROOT;
    menu_item_index = 0;
    border_position = ITEM_TOP;
}

static void enterSubmenu()
{
    current_menu = menu[current_menu][menu_item_index].menu_to_enter;
    menu_item_index = 0;
    border_position = ITEM_TOP;
}

// This implies that LedModes enum has same ordering as LED settings submenu
static void setLEDMode()
{
    led_mode = menu_item_index - 1;
}

static void setValue()
{
    menu_state = STATE_INPUT;
    ptr_user_input_value = menu[current_menu][menu_item_index].ptr_dynamic_value;
}

static void doNothing()
{
    return;
}

static void showMotorSpeeds()
{
    menu_state = STATE_DEBUG;
    ssd1306_Clear();
    char buff[64];
    snprintf(buff, sizeof(buff), "Vel0:%05d", timerif_getCounter(TIMER_ENC_M0));
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString(buff, Font_11x18);
    snprintf(buff, sizeof(buff), "Vel1:%05d", timerif_getCounter(TIMER_ENC_M1));
    ssd1306_SetCursor(2, 20);
    ssd1306_WriteString(buff, Font_11x18);
    snprintf(buff, sizeof(buff), "Vel2:%05d", timerif_getCounter(TIMER_ENC_M2));
    ssd1306_SetCursor(2, 38);
    ssd1306_WriteString(buff, Font_11x18);
}

static void showPowerInfo()
{
    menu_state = STATE_DEBUG;
    ssd1306_Clear();
    char buff[32];

    snprintf(buff, sizeof(buff), "Battery: %.2f V", BatVoltage);
    ssd1306_SetCursor(4, 4);
    ssd1306_WriteString(buff, Font_7x10);

    snprintf(buff, sizeof(buff), "Wall: %.2f V", WallVoltage);
    ssd1306_SetCursor(4, 16);
    ssd1306_WriteString(buff, Font_7x10);

    snprintf(buff, sizeof(buff), "Motors: %.2f A", MtrCurrent);
    ssd1306_SetCursor(4, 28);
    ssd1306_WriteString(buff, Font_7x10);

    snprintf(buff, sizeof(buff), "NUC: %.2f A", NucCurrent);
    ssd1306_SetCursor(4, 40);
    ssd1306_WriteString(buff, Font_7x10);
}

static void showFirmwareInfo()
{
    menu_state = STATE_DEBUG;
    ssd1306_Clear();
    char buff[64];
    snprintf(buff, sizeof(buff), "Firmware ver 3.0.0");
    ssd1306_SetCursor(2, 2);
    ssd1306_WriteString(buff, Font_7x10);
}
// ================ END MENU ITEM CALLBACKS ================

// ================ BEGIN DRAWING FUNCTIONS ================
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

    static IoPinType estop;
    estop.ptr_port = PIN_ESTOP_GPIO_Port;
    estop.pin_number = PIN_ESTOP_Pin;

    ssd1306_SetCursor(2, 15);
    if (ioif_isActive(&estop))
    {
        snprintf(buff, sizeof(buff), "ESTOP: ON");
    }

    else
    {
        snprintf(buff, sizeof(buff), "ESTOP: OFF");
    }
    ssd1306_WriteString(buff, Font_7x10);


    ssd1306_SetCursor(2, 27);
    ssd1306_WriteString("IP:123.123.123.123", Font_7x10);

    ssd1306_SetCursor(2, 51);
    ssd1306_WriteString("LED mode: blink", Font_7x10);
}

static void drawBorder(int border_position)
{
    ssd1306_DrawRect(BORDER_BEGIN_X, BORDER_BEGIN_Y + FIELD_HEIGHT * border_position, BORDER_WIDTH, FIELD_HEIGHT);
}

static void drawScrollbar()
{
    // divide vertical space between items
    float pixelsPerItem = SSD1306_HEIGHT / (float) getCurrentMenuSize();
    ssd1306_FillRect(SCROLLBAR_BEGIN_X, (menu_item_index - border_position) * pixelsPerItem, SCROLLBAR_WIDTH, (int) pixelsPerItem * 3);
}

static void drawMenuItems() 
{
    if (is_scrolling_activated)
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
            is_scrolling_activated = true;
            scrolling_main_loop_counter = 0;
        }
    }

    ssd1306_Clear();
    // Draw 3 items
    for (uint8_t item_pos = 0; item_pos < 3; item_pos++)
    {
        ssd1306_SetCursor(MENU_ITEM_LABEL_BEGIN_X, MENU_ITEM_LABEL_OFFSET_Y + item_pos * FIELD_HEIGHT);

        size_t label_length = strlen(menu[current_menu][menu_item_index + item_pos - border_position].label);

        // If label too long
        if (label_length > MAX_MENUITEM_LABEL_LENGTH)
        {
            char buffer_label[MAX_MENUITEM_LABEL_LENGTH + 1];

            // If item is selected, do scrolling
            if (item_pos == border_position)
            {
                // If scrolled to end
                if (label_length < MAX_MENUITEM_LABEL_LENGTH + scrolling_label_index)
                {
                    // Only show end
                    strncpy(buffer_label, menu[current_menu][menu_item_index + item_pos - border_position].label +
                    label_length - MAX_MENUITEM_LABEL_LENGTH, MAX_MENUITEM_LABEL_LENGTH);
                }

                // Else keep scrolling 
                else
                {
                    strncpy(buffer_label, menu[current_menu][menu_item_index + item_pos - border_position].label + scrolling_label_index, MAX_MENUITEM_LABEL_LENGTH);
                }
            }

            // Just show beginning
            else
            {
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
    // Draw menu item label
    ssd1306_SetCursor(5, 15);
    ssd1306_WriteString(menu[current_menu][menu_item_index].label, Font_7x10);

    // Draw variable value
    char buff[64];
    snprintf(buff, sizeof(buff), "%d", *(ptr_user_input_value));
    ssd1306_SetCursor(48, 30);
    ssd1306_WriteString(buff, Font_16x26);
}

static void drawDebugScreen()
{
    menu[current_menu][menu_item_index].item_callback();
}

// ================ END DRAWING FUNCTIONS ================

// ================ BEGIN INPUT HANDLERS ================

// This runs as an interrupt callback function
static void hardwareInputHandler(uint16_t pin_number)
{
    static IoPinType enc_a;
    enc_a.ptr_port = PIN_ROT_ENC_A_GPIO_Port;
    enc_a.pin_number = PIN_ROT_ENC_A_Pin;

    if (pin_number == PIN_ROT_ENC_SW_Pin)
    {
        is_input_select = true;
    }
    
    else if (pin_number == PIN_ROT_ENC_B_Pin) 
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

static void dashboardInputHandler()
{
    if (is_input_select)
    {
        enterMainMenu();
    }

    else if (is_input_clockwise)
    {
        // No functionality
        ;
    }

    else if (is_input_counterclockwise)
    {
        // No functionality
        ;
    }
}

static void menuInputHandler()
{
    if (is_input_select)
    {
        menu[current_menu][menu_item_index].item_callback();
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
    }

    if (is_input_select || is_input_clockwise || is_input_counterclockwise)
    {
        scrolling_main_loop_counter = 0;
        scrolling_label_index = 0;
        is_scrolling_activated = false;
    }
}

static void inputHandlerDebug()
{
    if (is_input_select)
    {
        menu_state = STATE_MENU;
    }

    else if (is_input_clockwise)
    {
        // No functionality
        ;
    }

    else if (is_input_counterclockwise)
    {
        // No functionality
        ;
    }
}

static void userInputInputHandler()
{
    if (is_input_select)
    {
        menu_state = STATE_MENU;
    }

    else if (is_input_clockwise)
    {   
        (*ptr_user_input_value)++;
    }

    else if (is_input_counterclockwise)
    {
        (*ptr_user_input_value)--;
    }
}

static void clearInputs()
{
    is_input_select = false;
    is_input_clockwise = false;
    is_input_counterclockwise = false;
}

// ================ END INPUT HANDLERS ================

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