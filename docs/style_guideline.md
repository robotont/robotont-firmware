### Formatting guideline

With a uniform source code layout and style we will speed up the development and simplify understanding of existing code. Most of the rules already maintained by the `clang-tidy` and `clang-format`.<br>

> [!NOTE]  
> Rules should apply only for project related files. No need to apply given rules on third party library files (i.e. Cube HAL, stm32xxx, C library etc.)

**Generic:**

1. ~~The code shall adhere to MISRA-C 2012~~ Rule deprecated for several reasons: 1) Document is not freely avaiable 2) Static analyzer tools, that control this, also not free (PC-Lint)
2. Use fixed width integer types defined in `stdint.h` [header](https://en.cppreference.com/w/c/types/integer)
3. Each `.c` file should be associated with `.h` file
4. Use following syntax and order for inclusions:
```cpp
#include "related_header.h" // 1-st, ""
#include <c_library.h>      // 2-nd, <>
#include "third_party.h"    // 3-rd, ""
#include "project_file.h"`  // 4-th, ""
```

**Statements and expressions** 

5. Avoid global variables if possible
6. Initialize variable at the beginning of the scope
7. Avoid using numbers directly, use `#define` if possible
8. Unsigned literals have suffix `u`, floating points literals have suffix `f`
```cpp
uint32_t some_value = 1u;
float another_value = 2.0f;
int8_t signed_value = 1;
```
10. There should be only one statement and one assignment per line.
```cpp
// BAD
uint8_t a, b, c;
// GOOD
uint8_t a;
uint8_t b;
uint8_t c;
```
11. All, starting and closing, braces should start on a new line. Braces must be placed always
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

11. Files must encoded in UTF-8, line endings must be in Windows format (CR-LF)
12. Docstring format:
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
13. Component placement order in `header.h`:
```cpp
/**
 * Includes
 * Defines
 * Type definitions
 * Constants
 * Global Variables
 * 
 * Public function prototypes
 * */
```
14. Component placement order in `source.c`:
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
15. Keep filenames short, lowercase and avoid using dashes (and underscores, if possible). 
```cpp
// GOOD
usbdrv.c
usb.c
// OK
sw_enc.c
// BAD
USB_driver.c
my-lib.c
someModule.c
```

**Style**

16. Name global functions like `filename_functionNameInCamelCase(...)`; name static functions like `functionNameInCamelCase(...)`. Add prefix `is`, if function returns boolean
```cpp
bool robot_isRunning(void);
uint8_t calc_calculateSum(uint8_t a, uint8_t b);
void priv_enableInterrupt(void);
```
17. Name variable in `snake_case`. Add prefix `is_` to the boolean type
```cpp
bool is_enabled;
char uart_buffer[UART_BUFFER_SIZE];
```
18. Name struct elements in `snake_case`
```cpp
typedef struct
{
    uint32_t pos_x;
    uint32_t pos_y;
} RobotPositionType;
```
19. Name typedefs in `PascalCase` with suffix `Type`
```cpp
typedef uint32_t RobotSpeedType;
```
20. Name enums in `PascalCase`; elements in `UPPER_CASE`
```cpp
enum PinState
{
    PIN_OFF,
    PIN_ON
};
``` 
21. Name macros and defines in `UPPER_CASE`
22. Pointer variables should have prefix `ptr_` 
```cpp
uint32_t *ptr_name;
```
23. If variable represents physical value, unit suffix should present. SI units are exception
```cpp
#define VOLTAGE_OFFSET_MV 12000u
uint32_t timeout_ms = 100u;
int8 voltage = 12;
float time_min = 32.3f
```

**Examples**

There's examples of how code should be formatted:
<details>
  <summary>sample.h</summary>
  
```cpp
// To be filled
```
</details>

<details>
  <summary>sample.c</summary>
  
```cpp
// To be filled
```
</details>
