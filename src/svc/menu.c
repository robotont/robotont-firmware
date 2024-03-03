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

#include "menu.h"
#include "ioif.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define BORDER_BEGIN_X 0
#define BORDER_BEGIN_Y 0
#define BORDER_END_X 100
#define FIELD_HEIGHT 20 // in pixels
#define BORDER_END_Y MENUITEM_HEIGHT
#define NUMBER_OF_MENUITEMS 3
#define VERT_PADDING 2
#define MENUITEM_HEIGHT SSD1306_HEIGHT - 2 * VERT_PADDING / NUMBER_OF_MENUITEMS // i.e 20 pixels for each item w borders
#define MAX_MENUITEMS 8

ItemPosition border_position = ITEM_TOP;
CurrentMenu current_menu = MENU_ROOT;
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
        {"Submenu 1", &enterSubmenu1},
        {"Change value", &inputValue1, &dummy},
        {"Dummy 1.2", &doNothing},
        {"Dummy 1.3", &doNothing},
        {"Dummy 1.4", &doNothing},
        {"Dummy 1.5", &doNothing}
    },
    // SUBMENU 1
    {
        {"^-- Main menu", &enterMainmenu},
        {"Submenu 2", &enterSubmenu2},
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
};


// ITEM label MAX LENGTH?
// use const to save memory?

static void enterMainmenu() {
    menu_state = STATE_MENU;
    current_menu = MENU_ROOT;
    item_index = 0;
    border_position = ITEM_TOP;
    drawMenuItems();
}

static void inputValue1() {
    menu_state = STATE_INPUT;
    
    drawInputScreen();


    
}


static void showDashboard() {
    menu_state = STATE_DASHBOARD;

    drawDashboard();
}

void doNothing() {
    return;
}

static void enterSubmenu1() {
    current_menu = MENU_SUBMENU1;
    item_index = 0;
    border_position = ITEM_TOP;
    drawMenuItems();
}

static void enterSubmenu2() {
    current_menu = MENU_SUBMENU2;
    item_index = 0;
    border_position = ITEM_TOP;
    drawMenuItems();
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

static void drawBorder(int border_position)
{
    ssd1306_DrawRectangle(BORDER_BEGIN_X, BORDER_BEGIN_Y + FIELD_HEIGHT * border_position,
                          BORDER_END_X, VERT_PADDING + (border_position+1) * FIELD_HEIGHT , White);
}

static size_t getCurrentMenuSize() {
    int size_counter = 0;
    for ( ; size_counter < MAX_MENUITEMS; size_counter++)
    {
        if (menu[current_menu][size_counter].label == NULL) 
        {
            break;
        }
    }
    return size_counter;
}

static void drawScrollbar()
{
    // divide space between no of items
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
    ssd1306_WriteString("Bat volt: 14.6 V", Font_7x10, White);

    ssd1306_SetCursor(2, 15);
    char buff[64];
    snprintf(buff, sizeof(buff), "Dummy value: %d", dummy);
    ssd1306_WriteString(buff, Font_7x10, White);

    ssd1306_SetCursor(2, 27);
    ssd1306_WriteString("IP:123.123.123.123", Font_7x10, White);

    ssd1306_SetCursor(2, 39);
    ssd1306_WriteString("press knob 4 menu", Font_7x10, White);

    ssd1306_SetCursor(2, 51);
    ssd1306_WriteString("LED mode: blink", Font_7x10, White);
}

static void drawMenuItems() 
{
    ssd1306_Fill(Black);
    
    ssd1306_SetCursor(2, 3);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_TOP - border_position].label, Font_7x10, White);

    ssd1306_SetCursor(2, 23);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_CENTER - border_position].label, Font_7x10, White);

    ssd1306_SetCursor(2, 43);
    ssd1306_WriteString(menu[current_menu][item_index + ITEM_BOTTOM - border_position].label, Font_7x10, White);

    drawBorder(border_position);
    drawScrollbar();
}

static void drawInputScreen() {
    ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 3);
    ssd1306_WriteString(menu[current_menu][item_index].label, Font_7x10, White);


    char buff[64];
    snprintf(buff, sizeof(buff), "%d", *(menu[current_menu][item_index].ptr_valuetochange));
    ssd1306_SetCursor(2, 35);

    ssd1306_WriteString(buff, Font_16x24, White);
}


static void updateHandlerDashboard()
{
    drawDashboard();

    if (is_input_select)
    {
        enterMainmenu();
        is_input_select = false;
    }

}

static void updateHandlerMenu() {
    if (is_input_select)
    {
        // run current menu item callback 
        menu[current_menu][item_index].select_callback();
        is_input_select = false;
    }

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

        drawMenuItems();

        is_input_counterclockwise = false;
    }

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

        drawMenuItems();

        is_input_clockwise = false;
    }
}

static void updateHandlerInput()
{
    if (is_input_select)
    {
        // set value, return to prev menu

        menu_state = STATE_MENU;
        // leave other menu specific variables unchanged
        drawMenuItems();
        is_input_select = false;
    }

    else if (is_input_counterclockwise)
    {

        // run current menu item left callback
        (*(menu[current_menu][item_index].ptr_valuetochange))--;

        drawInputScreen();

        is_input_counterclockwise = false;
    }

    else if (is_input_clockwise)
    {   
        // run current menu item right callback
        (*(menu[current_menu][item_index].ptr_valuetochange))++;
        drawInputScreen();

        is_input_clockwise = false;
    }
}


void menu_init(void)
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
        updateHandlerDashboard();   
    }

    else if (menu_state == STATE_MENU) 
    {
        updateHandlerMenu();
    }

    else if (menu_state == STATE_INPUT) {
        updateHandlerInput();
    }

    ssd1306_UpdateScreen();
}
