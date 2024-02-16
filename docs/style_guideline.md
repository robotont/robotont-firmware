# Formatting guideline

With a uniform source code layout and style we will speed up the development and simplify understanding of existing code. Most of the rules already maintained by the `clang-tidy` and `clang-format`.<br>

> [!NOTE]  
> Rules should apply only for project related files. No need to apply given rules on third party library files (i.e. Cube HAL, stm32xxx, C library etc.)

## Rules 

**Generic:**

1. Use fixed width integer types defined in `stdint.h` [header](https://en.cppreference.com/w/c/types/integer)
2. Each `.c` file should be associated with `.h` file
3. Use following syntax and order for inclusions:
```cpp
#include "related_header.h" // 1-st, ""
#include <c_library.h>      // 2-nd, <>
#include "third_party.h"    // 3-rd, ""
#include "project_file.h"`  // 4-th, ""
```

**Statements and expressions** 

1. Avoid global variables if possible
2. Initialize variable at the beginning of the scope
3. Avoid using numbers directly, use `#define` if possible
4. Unsigned literals have suffix `u`, floating points literals have suffix `f`
```cpp
uint32_t some_value = 1u;
float another_value = 2.0f;
int8_t signed_value = 1;
```
5. There should be only one statement and one assignment per line.
```cpp
// BAD
uint8_t a, b, c;
// GOOD
uint8_t a;
uint8_t b;
uint8_t c;
```
6. All, starting and closing, braces should start on a new line. Braces must be placed always
```cpp
// BAD
if (true)
    return 0;
// GOOD
if (true)
{
    return 0;
}
```

**File format**

1. Files must encoded in UTF-8, line endings must be in Windows format (CR-LF)
2. Docstring format:
```cpp
/**
 * @brief
 * 
 * @param a
 * @param b
 * @param c
 * @return <type>
 */
```
> [!NOTE]  
> `@param` and `@return` fields can be ignored, if information in the `@brief` is sufficient.

3. Component placement order in `header.h`:
```cpp
/**
 * Includes
 * Defines
 * Type definitions
 * Global Variables (extern)
 * 
 * Public function prototypes
 * */
```
4. Component placement order in `source.c`:
```cpp
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
```
5. Keep filenames short, lowercase and avoid using dashes (and underscores, if possible). 
```cpp
// GOOD
spidrv.c
usb.c
// ACCEPTABLE
sw_enc.c
// BAD
USB_driver.c
my-lib.c
someModule.c
```

**Style**

1. Name global functions like `filename_functionNameInCamelCase(...)`; name static functions like `functionNameInCamelCase(...)`. Add prefix `is`, if function returns boolean
```cpp
// GOOD
bool robot_isRunning(void);
uint8_t calc_calculateSum(uint8_t a, uint8_t b);
static void enableInterrupt(void);
// BAD
bool robot_is_running(void);                    // Can be mistaken with variable
uint8_t calcCalculateSum(uint8_t a, uint8_t b)  // Can be mistaken with static function
static void enableinterrupts(void)              // Hard to read
```
2. Name variable in `snake_case`. Add prefix `is_` to the boolean type
```cpp
bool is_enabled;
char uart_buffer[UART_BUFFER_SIZE];
```
3. Name struct elements in `snake_case`
```cpp
typedef struct
{
    uint32_t pos_x;
    uint32_t pos_y;
} RobotPositionType;
```
4. Name typedef's in `PascalCase` with suffix `Type`
```cpp
typedef uint32_t RobotSpeedType;
```
5. Name enums in `PascalCase`; elements in `UPPER_CASE`
```cpp
enum PinState
{
    PIN_OFF,
    PIN_ON
};
``` 
6. Name macros and defines in `UPPER_CASE`
7. Pointer variables should have prefix `ptr_`. Exception is made for postfix `handler`, because it is common name for `STM32F4` interface handlers.
```cpp
uint32_t *ptr_name;
I2C_HandleTypeDef *i2c_handler;
```
8. If variable represents physical value, unit suffix should present. SI units are exception
```cpp
#define VOLTAGE_OFFSET_MV 12000u
uint32_t timeout_ms = 100u;
int8_t voltage = 12;
float time_minutes = 32.3f // time_min would be bad example, since "min" could be confused with word "minimum"
```

## Examples

There's examples of how code should be formatted:
<details>
  <summary>sample.h</summary>
  
```cpp
/**
 * @file sample.h
 * @brief Sample code for header file
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2024 Tartu Ülikool
 */

#ifndef SAMPLE_H
#define SAMPLE_H

#include "other_module.h"

#define SAMPLE_VALUE_BAR 1u
#define SAMPLE_VALUE_FOO 2u

typedef uint16_t SampleType;

extern SampleType global_variable;

void sample_function(void);

#endif
```
</details>

<details>
  <summary>sample.c</summary>
  
```cpp
/**
 * @file sample.c
 * @brief Sample code for source file
 *
 * @author Leonid Tšigrinski (leonid.tsigrinski@gmail.com)
 * @copyright Copyright (c) 2024 Tartu Ülikool
 */

#include "sample.h"

#define STATIC_DEFINE_1 3u
#define STATIC_DEFINE_2 4u

typedef uint8_t StaticType;

StaticType static_variable;
SampleType also_static_variable;
SampleType global_variable;

static someFunction(void);

void sample_function(void)
{
    someFunction();
}

static someFunction(void)
{
    static_variable = 0u;
    global_variable = STATIC_DEFINE_2;
}
```
</details>
