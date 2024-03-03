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
    // bool is_submenu ??? doesnt matter, can use select_callback
    void (*select_callback)(void);
    int * ptr_valuetochange;
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

// BEGIN MENU CALLBACKS
static void enterMainmenu();
static void showDashboard();
static void doNothing();
static void enterSubmenu1();
static void enterSubmenu2();
static void inputValue1();
// END MENU CALLBACKS

void menu_init(void);
void menu_update();

static void inputHandler(uint16_t pin_number);
static void drawDashboard();
static void drawScrollbar();
static void drawBorder(int borderIndex);
static void drawMenuItems();
static void drawInputScreen();

#endif