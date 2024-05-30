/**
 * @file menu.h
 * @brief Service. Displays information and allows for changing various parameters on the display using rotary encoder as input
 * 
 * @author Andres Sakk (andres.sakk@ut.ee)
 * @copyright Copyright (c) 2023 Tartu Ülikool
 */

// TODO make dashboard nicer
// TODO demo program submenu
// TODO racing and normal mode callbacks

// #define DEBUG

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "menu.h"
#include "peripheral.h" // Pin defines
#include "ioif.h" // IoPinType
#include "measurements.h" // Power info
#include "led.h" // LED modes
#include "timerif.h" // showMotorSpeeds

#define DASHBOARD {"^ Dashboard", &showDashboard}
#define MAINMENU {"^ Main menu", &enterMainMenu}
#define SUBMENU(submenu_label, MENU_TYPE) {submenu_label, &enterSubmenu, MENU_TYPE}
#define USERINPUT(setvalue_label, ptr_value) {setvalue_label, &setValue, MENU_NONE, ptr_value}
#define INFOSCREEN(infoscreen_label, callback) {infoscreen_label, callback}
#define MENUITEM(menuitem_label, callback) {menuitem_label, callback}

#define BORDER_BEGIN_X 0
#define BORDER_BEGIN_Y 0
#define BORDER_WIDTH 120
#define FIELD_HEIGHT 21

#define MENU_ITEM_LABEL_BEGIN_X 4
#define MENU_ITEM_LABEL_OFFSET_Y 6

#define SCROLLBAR_BEGIN_X 124
#define SCROLLBAR_WIDTH 4

#define MAX_MENUITEMS 16
#define MAX_TEXT_LENGTH 16

#define SCROLLING_WAIT_IN_MAIN_LOOP_DT_INCREMENTS 25
#define SCROLLING_CONTINUE_IN_MAIN_LOOP_DT_INCREMENTS 5

typedef enum 
{
    MENU_NONE = -1,
    MENU_ROOT,
    MENU_LED_SETTINGS,
    MENU_MOTOR_SETTINGS,
    MENU_SEND_CMD,
    MENU_DEMO_SUBMENU1,
    MENU_DEMO_SUBMENU2,
    MENU_NETWORK_SETTINGS,
    MENU_ROS_NODES,
    MENU_ROS_CONTAINERS,
} MenuType;

typedef enum 
{
    ITEM_TOP,
    ITEM_CENTER,
    ITEM_BOTTOM,
} ItemPosition;

typedef enum 
{
    COMPACTVIEW_TOP,
    COMPACTVIEW_ABOVECENTER,
    COMPACTVIEW_CENTER,
    COMPACTVIEW_BELOWCENTER,
    COMPACTVIEW_BOTTOM,
} CompactViewPosition;

typedef enum 
{
    LARGEVIEW_TOP,
    LARGEVIEW_CENTER,
    LARGEVIEW_BOTTOM,
} LargeViewPosition;

typedef enum 
{
    STATE_DASHBOARD,
    STATE_MENU,
    STATE_USERINPUT,
    STATE_INFOSCREEN,
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
static FontDef *ptr_current_font;

static bool is_input_select = false;
static bool is_input_clockwise = false;
static bool is_input_counterclockwise = false;
static uint8_t input_clockwise_counter = 0;
static uint8_t input_counterclockwise_counter = 0;

static int scrolling_main_loop_counter = 0;
static int scrolling_text_index = 0;
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
static void sendCommand();
// END MENU CALLBACKS

// BEGIN DRAWING FUNCTIONS
static void drawDashboard();
static void drawScrollbar();
static void drawBorder(int borderIndex);
static void drawText(char *text, bool is_scrolling);
static void drawMenuItems();
static void drawInputScreen();
static void drawInfoScreen();
// END DRAWING FUNCTIONS

// BEGIN INPUT HANDLERS
static void hardwareInputHandler(uint16_t pin_number);
static void dashboardInputHandler();
static void menuInputHandler();
static void userInputInputHandler();
static void infoScreenInputHandler();
static void clearInputs();
// END INPUT HANDLERS

static int getCurrentMenuSize();
static void setCursorCompactView(CompactViewPosition position);
static void setCursorLargeView(LargeViewPosition position);

// ================ END DECLARATIONS ================

// NOTE: All submenus have to have the same index in menu[] array as their counterparts in MenuType enum
static MenuItem menu[][MAX_MENUITEMS] = 
{
    // ROOT
    {
        DASHBOARD,
        SUBMENU("> LED modes", MENU_LED_SETTINGS),
        SUBMENU("> Motor control settings", MENU_MOTOR_SETTINGS),
        SUBMENU("> Send commands", MENU_SEND_CMD),
        SUBMENU("> Demo submenu 1", MENU_DEMO_SUBMENU1),
        SUBMENU("> Network settings", MENU_NETWORK_SETTINGS),
        SUBMENU("> Reset ROS nodes", MENU_ROS_NODES),
        SUBMENU("> Choose ROS container", MENU_ROS_CONTAINERS),
        INFOSCREEN("Firmware information", &showFirmwareInfo),
        INFOSCREEN("Motor speeds", &showMotorSpeeds),
        USERINPUT("Set max speed", &dummy),
        INFOSCREEN("Power information", &showPowerInfo),
    },
    // LED SETTINGS
    {
        MAINMENU,
        MENUITEM("MODE_SPIN", &setLEDMode),
        MENUITEM("MODE_PULSE", &setLEDMode),
        MENUITEM("MODE_COLORS_SMOOTH", &setLEDMode),
        MENUITEM("MODE_WHEEL_COLORS", &setLEDMode),
        MENUITEM("MODE_COLORS_RGB", &setLEDMode),
        MENUITEM("MODE_COLORS_SPIN", &setLEDMode),
        MENUITEM("MODE_MOTOR_SPEEDS", &setLEDMode),
        MENUITEM("MODE_SCAN_RANGES", &setLEDMode),
    },
    // MOTOR SETTINGS
    {   
        MAINMENU,
        MENUITEM("Activate racing mode", &doNothing),
        MENUITEM("Activate normal mode", &doNothing),
        USERINPUT("Set motor linear velocity", &dummy),
        USERINPUT("Set motor angular velocity", &dummy),
        USERINPUT("Set motor effort", &dummy),
    },
    // MENU_SEND_CMD
    {
        MAINMENU,
        MENUITEM("Send shutdown", &sendCommand),
        MENUITEM("Send reboot", &sendCommand),
        MENUITEM("Send debug msg", &sendCommand),
    },
    // DEMO SUBMENU 1
    {
        MAINMENU,
        SUBMENU("> Demo submenu 2", MENU_DEMO_SUBMENU2),
        MENUITEM("Demo item 2.1", &doNothing),
        MENUITEM("Demo item 2.2", &doNothing),
        MENUITEM("Demo item 2.3", &doNothing),
        MENUITEM("Demo item 2.4", &doNothing),
        MENUITEM("Demo item 2.5", &doNothing),
        MENUITEM("Demo item 2.6", &doNothing),
        MENUITEM("Demo item 2.7", &doNothing),
        MENUITEM("Demo item 2.8", &doNothing),
        MENUITEM("Demo item 2.9", &doNothing),
    },
    // DEMO SUBMENU 2
    {
        SUBMENU("^ Demo submenu 1", MENU_DEMO_SUBMENU1),
        MENUITEM("Demo item 3.1", &doNothing),
        MENUITEM("Demo item 3.2", &doNothing),
        MENUITEM("Demo item 3.3", &doNothing),
    },
    // NETWORK SETTINGS
    {
        MAINMENU,
        MENUITEM("Show IP address", &doNothing),
        MENUITEM("Choose network", &doNothing),
        MENUITEM("Enter network password", &doNothing),
        MENUITEM("Connect", &doNothing),
        MENUITEM("AP mode selection", &doNothing),
    },
    // ROS NODES
    {
        MAINMENU,
        MENUITEM("Reset camera node", &doNothing),
        MENUITEM("Reset driver", &doNothing),
    },
    // ROS CONTAINERS
    {
        MAINMENU,
        MENUITEM("Container 1 (mapping?)", &doNothing),
        MENUITEM("Container 2 (AR steering?)", &doNothing),
        MENUITEM("Container 3 (teleop?)", &doNothing),
        MENUITEM("Container 4 (demo x?)", &doNothing),
    }
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
    if (ssd1306_UpdateScreenCompleted())
    {
        #ifdef DEBUG
        volatile uint32_t before = system_hal_timestamp();
        #endif

        switch (menu_state)
        {
            case STATE_DASHBOARD:
                drawDashboard();
                dashboardInputHandler();
                break;

            case STATE_MENU:
                drawMenuItems();
                menuInputHandler();
                break;
            
            case STATE_USERINPUT:
                drawInputScreen();
                userInputInputHandler();
                break;

            case STATE_INFOSCREEN:
                drawInfoScreen();
                infoScreenInputHandler();
                break;
        }

        clearInputs();

        #ifdef DEBUG
        ssd1306_SetCursor(90,5);
        char buff[10];
        volatile uint32_t after = system_hal_timestamp();
        int delay = after-before;
        snprintf(buff, sizeof(buff), "%d", delay);
        ssd1306_WriteString(buff, *ptr_current_font);
        #endif

        ssd1306_UpdateScreen();
    }

    if (is_scrolling_activated)
    {
        if (scrolling_main_loop_counter == SCROLLING_CONTINUE_IN_MAIN_LOOP_DT_INCREMENTS)
        {
            scrolling_text_index++;
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

    scrolling_main_loop_counter++;
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
    menu_state = STATE_USERINPUT;
    ptr_user_input_value = menu[current_menu][menu_item_index].ptr_dynamic_value;
}

static void doNothing()
{
    return;
}

static void showMotorSpeeds()
{
    menu_state = STATE_INFOSCREEN;
    ssd1306_Clear();
    char buff[64];

    setCursorLargeView(LARGEVIEW_TOP);
    snprintf(buff, sizeof(buff), "Vel0:%05d", timerif_getCounter(TIMER_ENC_M0));
    drawText(buff, true);

    setCursorLargeView(LARGEVIEW_CENTER);
    snprintf(buff, sizeof(buff), "Vel1:%05d", timerif_getCounter(TIMER_ENC_M1));
    drawText(buff, true);
    
    setCursorLargeView(LARGEVIEW_BOTTOM);
    snprintf(buff, sizeof(buff), "Vel2:%05d", timerif_getCounter(TIMER_ENC_M2));
    drawText(buff, true);

}

static void showPowerInfo()
{
    menu_state = STATE_INFOSCREEN;
    ssd1306_Clear();
    char buff[32];

    setCursorCompactView(COMPACTVIEW_TOP);
    snprintf(buff, sizeof(buff), "Battery: %.2f V", BatVoltage);
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_ABOVECENTER);
    snprintf(buff, sizeof(buff), "Wall: %.2f V", WallVoltage);
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_CENTER);
    snprintf(buff, sizeof(buff), "Motors: %.2f A", MtrCurrent);
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_BELOWCENTER);
    snprintf(buff, sizeof(buff), "NUC: %.2f A", NucCurrent);
    drawText(buff, true);
}

static void showFirmwareInfo()
{
    menu_state = STATE_INFOSCREEN;
    ssd1306_Clear();
    char buff[64];

    setCursorCompactView(COMPACTVIEW_TOP);
    snprintf(buff, sizeof(buff), "Firmware ver 3.0.0");
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_ABOVECENTER);
    snprintf(buff, sizeof(buff), "Hardware ver 3");
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_CENTER);
    snprintf(buff, sizeof(buff), "Last update 2024-04-04");
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_BELOWCENTER);
    snprintf(buff, sizeof(buff), "1123456789012345678912345678123456");
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_BOTTOM);
    snprintf(buff, sizeof(buff), "blah");
    drawText(buff, true);
}

// This implies that Send Commands submenu order corresponds to switch statement
static void sendCommand()
{
    switch (menu_item_index)
    {
    case 1:
        printf("CMD:shutdown now\r\n");
        break;
    
    case 2:
        printf("CMD:reboot now\r\n");
        break;

    case 3:
        printf("CMD:echo test\r\n");
        break;
    
    default:
        break;
    }
}
// ================ END MENU ITEM CALLBACKS ================

// ================ BEGIN DRAWING FUNCTIONS ================
static void drawDashboard() 
{
    ssd1306_Clear();
    char buff[64];

    setCursorCompactView(COMPACTVIEW_TOP);
    snprintf(buff, sizeof(buff), "Bat volt: %.1f V", BatVoltage);
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_ABOVECENTER);
    snprintf(buff, sizeof(buff), "Motor duty: %d%%", dummy);
    drawText(buff, true);    

    static IoPinType estop;
    estop.ptr_port = PIN_ESTOP_GPIO_Port;
    estop.pin_number = PIN_ESTOP_Pin;

    setCursorCompactView(COMPACTVIEW_CENTER);
    if (ioif_isActive(&estop))
    {
        snprintf(buff, sizeof(buff), "ESTOP: ON");
    }

    else
    {
        snprintf(buff, sizeof(buff), "ESTOP: OFF");
    }
    drawText(buff, true);

    setCursorCompactView(COMPACTVIEW_BELOWCENTER);
    // drawText("IP:123.123.123.123", true);

    setCursorCompactView(COMPACTVIEW_BOTTOM);
    drawText("LED mode: blink", true);
}

static void drawBorder(int border_position)
{
    ssd1306_DrawRect(BORDER_BEGIN_X, BORDER_BEGIN_Y + FIELD_HEIGHT * border_position, BORDER_WIDTH, FIELD_HEIGHT);
}

static void drawScrollbar()
{
    // divide vertical space between items
    float pixelsPerItem = SSD1306_HEIGHT / (float) getCurrentMenuSize();
    ssd1306_FillRect(SCROLLBAR_BEGIN_X, (menu_item_index - border_position) * pixelsPerItem, SCROLLBAR_WIDTH, (int) (pixelsPerItem * 3));
}

static void drawText(char *text, bool is_scrolling)
{
    size_t text_length = strlen(text);

    // If label fits
    if (text_length <= MAX_TEXT_LENGTH)
    {
        ssd1306_WriteString(text, *ptr_current_font);
    }

    else
    {
        char buffer_label[MAX_TEXT_LENGTH + 1];

        // If scrolling is activated, do scrolling
        if (is_scrolling)
        {
            // If can scroll 
            if (text_length > MAX_TEXT_LENGTH + scrolling_text_index)
            {
                strncpy(buffer_label, text + scrolling_text_index, MAX_TEXT_LENGTH);
            }

            // Else scrolled to end
            else
            {
                // Only show end
                strncpy(buffer_label, text + text_length - MAX_TEXT_LENGTH, MAX_TEXT_LENGTH);
            }
        }

        // Just show beginning
        else
        {
            strncpy(buffer_label, text, MAX_TEXT_LENGTH);
        }

        buffer_label[MAX_TEXT_LENGTH] = '\0';

        ssd1306_WriteString(buffer_label, *ptr_current_font);
    }
}


static void drawMenuItems() 
{
    ssd1306_Clear();
    ptr_current_font = &Font_7x10;
    // Draw 3 items
    for (uint8_t item_pos = 0; item_pos < 3; item_pos++)
    {
        ssd1306_SetCursor(MENU_ITEM_LABEL_BEGIN_X, MENU_ITEM_LABEL_OFFSET_Y + item_pos * FIELD_HEIGHT);

        if (item_pos == border_position)
        {
            drawText(menu[current_menu][menu_item_index + item_pos - border_position].label, true);
        }
        
        else
        {
            drawText(menu[current_menu][menu_item_index + item_pos - border_position].label, false);
        }
    }

    drawBorder(border_position);
    drawScrollbar();
}

static void drawInputScreen() {
    ssd1306_Clear();
    // Draw menu item label
    ssd1306_SetCursor(5, 15);
    drawText(menu[current_menu][menu_item_index].label, true);

    // Draw variable value
    char buff[64];
    snprintf(buff, sizeof(buff), "%d", *(ptr_user_input_value));
    ssd1306_SetCursor(48, 30);
    ssd1306_WriteString(buff, Font_16x26);
}

static void drawInfoScreen()
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
            input_clockwise_counter++;
        }

        else 
        {
            is_input_counterclockwise = true;
            input_counterclockwise_counter++;
        }
    }    
}

static void dashboardInputHandler()
{
    if (is_input_select)
    {
        enterMainMenu();
    }

    else if (input_clockwise_counter > 0)
    {
        // No functionality
        ;
    }

    else if (input_counterclockwise_counter > 0)
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

    while (input_clockwise_counter > 0)
    {   
        if (menu_item_index < getCurrentMenuSize() - 1)
        {
            menu_item_index++;

            if (border_position != ITEM_BOTTOM)
            {
                border_position++;
            }

            input_clockwise_counter--;
        }

        else
        {
            input_clockwise_counter = 0;
        }
    }

    while (input_counterclockwise_counter > 0)
    {
        if (menu_item_index > 0)
        {
            menu_item_index--;

            if (border_position != ITEM_TOP)
            {
                border_position--;
            }
            
            input_counterclockwise_counter--;
        }

        else
        {
            input_counterclockwise_counter = 0;
        }
    }
}

static void infoScreenInputHandler()
{
    if (is_input_select)
    {
        menu_state = STATE_MENU;
    }

    else if (input_clockwise_counter > 0)
    {
        // No functionality
        ;
    }

    else if (input_counterclockwise_counter > 0)
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

    while (input_clockwise_counter > 0)
    {   
        if (*ptr_user_input_value < 100)
        {
            (*ptr_user_input_value)++;
        }

        input_clockwise_counter--;
    }

    while (input_counterclockwise_counter > 0)
    {
        if (*ptr_user_input_value > 0)
        {
            (*ptr_user_input_value)--;
        }

        input_counterclockwise_counter--;
    }
}

static void clearInputs()
{   
    if (is_input_select || is_input_clockwise || is_input_counterclockwise)
    {
        scrolling_main_loop_counter = 0;
        scrolling_text_index = 0;
        is_scrolling_activated = false;
    }

    is_input_select = false;
    is_input_clockwise = false;
    is_input_counterclockwise = false;
    input_clockwise_counter = 0;
    input_counterclockwise_counter = 0;
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

static void setCursorCompactView(CompactViewPosition position)
{
    ptr_current_font = &Font_7x10;

    switch (position)
    {
        case COMPACTVIEW_TOP:
            ssd1306_SetCursor(2, 0);
            break;

        case COMPACTVIEW_ABOVECENTER:
            ssd1306_SetCursor(2, 12);
            break;

        case COMPACTVIEW_CENTER:
            ssd1306_SetCursor(2, 24);
            break;

        case COMPACTVIEW_BELOWCENTER:
            ssd1306_SetCursor(2, 36);
            break;

        case COMPACTVIEW_BOTTOM:
            ssd1306_SetCursor(2, 48);
            break;
        
        default:
            break;
    }

    return;
}

static void setCursorLargeView(LargeViewPosition position)
{
    ptr_current_font = &Font_11x18;

    switch (position)
    {
        case LARGEVIEW_TOP:
            ssd1306_SetCursor(2, 0);
            break;

        case LARGEVIEW_CENTER:
            ssd1306_SetCursor(2, 20);
            break;

        case LARGEVIEW_BOTTOM:
            ssd1306_SetCursor(2, 40);
            break;
        
        default:
            break;
    }

    return;
}
