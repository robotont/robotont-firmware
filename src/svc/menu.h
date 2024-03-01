#ifndef MENU_H
#define MENU_H

/**
 * Includes
 * Defines
 * Type definitions
 * Global Variables (extern)
 * 
 * Public function prototypes
 * */

#include "ioif.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

#define PIN_ENC_SW GPIO_PIN_14
#define PIN_ENC_B GPIO_PIN_15
#define PIN_ENC_A GPIO_PIN_6

typedef struct
{
    char* label;
    // bool is_submenu ??? doesnt matter, can use pressed_callback
    void (*pressed_callback)(void);
    // ptr left_callback --- this is for numeric input selection. not used for dashboard or menu navigation
    // ptr right_callback
} MenuItem;

typedef enum 
{
    MENU_ROOT,
    MENU_SUBMENU1,
    MENU_SUBMENU2,
} CurrentMenu;

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
    STATE_DEBUG // rohke info kuvamiseks, nt 6 v2lja arvmuutujatega
} MenuState;


void menu_init(void);
static void processInput(uint16_t pin_number);
void menu_update();
static void drawDashboard();
static void drawScrollbar();
static void drawBorder(int borderIndex);
static void drawMenuItems();
static MenuItem createMenuItem(char* title, uint8_t index);
static initializeMenuItems();

#endif