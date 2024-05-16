# Adding menu functionality

This guide covers how to add various functionality for Robotont's OLED screen and encoder knob menu.

## Changing dashboard info

1. Edit the `drawDashboard()` function.
   
```c
static void drawDashboard() 
{
    ssd1306_Clear();
    char buff[64];

    setCursorCompactView(COMPACTVIEW_TOP);
    snprintf(buff, sizeof(buff), "Important value: %d", variable);
    drawText(buff, true);  
}
```

## Adding new items to the menu

1. Create a new callback function for your menu item.
2. Define a new `MenuItem` in `menu[][]` with a pointer your callback function using the `MENUITEM` macro.

```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    {
        MAINMENU,
        MENUITEM("New menu item", &callbackFunction),
    },

    ...
```

## Adding input screens to the menu

1. Make sure the variable you wish to change is in scope.
2. Define a new `MenuItem` in `menu[][]` with a pointer to the variable using the `USERINPUT` macro.

```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    {
        MAINMENU,
        USERINPUT("Change a value", &value),
    },

    ...
```


## Adding info screens to the menu
1. Create a new callback function for your info screen.
> [!NOTE]  
> A info screen callback needs to include changing `menu_state` to `STATE_INFOSCREEN`. 

```c
static void displayInfoCallback()
{
    menu_state = STATE_INFOSCREEN;
    ssd1306_Clear();

    setCursorCompactView(COMPACTVIEW_TOP);
    drawText("a static string", true);
}
```

2. Define a new `MenuItem` in `menu[][]` with a pointer your callback function using the `INFOSCREEN` macro.

```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    {
        MAINMENU,
        INFOSCREEN("Show something", &displayInfoCallback),
    },

    ...
```


## Adding submenus


1. Add a new menu type into the `MenuType` enum

```c
typedef enum 
{
    MENU_NONE = -1,
    MENU_ROOT,
    MENU_NEW_SUBMENU,
} MenuType;
```

2. Add a new array in `menu[][]` containing the items in the new submenu that changes `menu_state` to the new state.

> [!NOTE]  
> The ordering of `MenuType` and `menu[][]` needs to be the same. Meaning that if your submenu is in the last position in the `MenuType` enum, it also needs to be last in  `menu[][]` array. 

```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    ...

    {
        MAINMENU,
        MENUITEM("New item", &doSomething),
    },

    ...
```


3. Add an entrypoint for your submenu somewhere in the menu structure.
   
```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    {
        DASHBOARD,
        SUBMENU("New submenu", MENU_NEW_SUBMENU),
    },

    ...
```


## Creating new states

1. Add a new state into the `MenuState` enum

```c
typedef enum 
{
    STATE_DASHBOARD,
    STATE_MENU,
    STATE_USERINPUT,
    STATE_INFOSCREEN,
    STATE_NEW,
} MenuState;
```

2. Define a new `MenuItem` in `menu[][]` with a callback function that changes `menu_state` to the new state.

```c
static MenuItem menu[][MAX_MENUITEMS] = 
{
    {
        MAINMENU,
        MENUITEM("Switch to new state", &switchToNewState),
    },

    ...
```

```c
static void switchToNewState()
{
    menu_state = STATE_NEW;
    // Additional code
}
```

3. Implement drawing and input handler functions 

```c
static void drawNewState() 
{
    ssd1306_Clear();
    // Drawing code
}
```

```c
static void newStateInputHandler()
{
    if (is_input_select)
    {
        // Button press action
    }

    else if (input_clockwise_counter > 0)
    {
        // Clockwise rotation action
    }

    else if (input_counterclockwise_counter > 0)
    {
        // Counterclockwise rotation action
    }
}
```

4. Add your drawing and input handler functions to `menu_update()`'s `switch` statement.

```c
void menu_update()
{
    if (ssd1306_UpdateScreenCompleted())
    {
        switch (menu_state)
        {
            case STATE_NEW:
                drawNewState();
                newStateInputHandler();
                break;

            ...
```