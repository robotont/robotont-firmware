/**
 * Includes
 * Defines
 * Type definitions
 * Global Variables (extern)
 * 
 * Public function prototypes
 * */

#ifndef MENU_H
#define MENU_H

#include "ioif.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

#define PIN_ENC_SW GPIO_PIN_14
#define PIN_ENC_B GPIO_PIN_15
#define PIN_ENC_A GPIO_PIN_6

typedef enum 
{
    MENU_ROOT,
    MENU_SUBMENU1,
    MENU_SUBMENU2,
    MENU_LED_SETTINGS
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
    STATE_DEBUG // rohke info kuvamiseks, nt 6 v2lja arvmuutujatega
} MenuState;

typedef struct
{
    char* label;
    void (*select_callback)(void);
    int * ptr_valuetochange;
    MenuType submenu_index;
} MenuItem;

// BEGIN MENU CALLBACKS
static void enterMainmenu();
static void showDashboard();
static void doNothing();
static void enterSubmenu();
static void setValue();
static void setLEDMode();
// END MENU CALLBACKS

void menu_init(void);
void menu_update();

static void inputHandler(uint16_t pin_number);
static void inputHandlerDashboard();
static void inputHandlerMenu();
static void inputHandlerInput();

static int getCurrentMenuSize();
static void drawDashboard();
static void drawScrollbar();
static void drawBorder(int borderIndex);
static void drawMenuItems();
static void drawInputScreen();

#endif