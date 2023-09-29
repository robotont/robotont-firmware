# Robotont mbed mainboard firmware

Simplistic firmware for the Robotex platform mainboard.

## Serial protocol

The virtual serial port on the mbed USB bus is used for bidirectional communication. Approximately 100 times a second, motor speeds are given (from -100 to 100 in arbitrary units, sorry...). The format is: "motor1:motor2:motor3\n".

In the same format You can also send data to the mainboard to set the motor speeds (from -100 to 100).

## Development

Repository contains PlatformIO project. PlatformIO is a development ecosystem for embedded systems and eases working on the code a lot.

To work on this code just clone the repo and open it in the PlatformIO IDE (or build and upload it with the PlatformIO core tools from the command line). All the dependencies and build tools are installed automatically on the first build.

## Coding Style and formatting

With a uniform source code layout and style we will speed up the development and simplify understanding of existing code. 
Some of the rules can be maintained automaticly using tools called `clang-format` and `clang-tidy`. </br>
    
  - `clang-format`: Separate executable, that can reformat given file according to style rules. Rules can be configured.
  - `clang-tidy`: Linter tool, that highliht syntax and style issues (e.g. `snake_case` vs `camelCase`). Rules can be configured.

There's examples of how code should be formatted:
<details>
  <summary>sample.h</summary>
  
  `<details>` afterward.
```cpp
/**
\file     sample.h
\brief    Sample module to demonstrate code format and naming
\detail   Demonstrates format and naming

\author   Leonid Tšigrinski
\copyright To be defined
*/

#ifndef SAMPLE_H
#define SAMPLE_H

#include <stdint.h>

// Macro and constant naming
#define SAMPLE_GLOBAL_DEFINE 0

// Global variable naming
extern uint8_t g_global_variable;
uint8_t g_global_variable = 0U;

// Suppression for clang-tidy
typedef float float32_t; // NOLINT

// typedef and pointer
typedef struct OpaqueSampleType const *SampleIdType;

// Public function naming (NAMING-2)
void sample_init(const uint8_t * p_ptr_argument);
SampleIdType sample_createInstance(void);
// Use types from stdint (TYPES-1)
uint32_t sample_runDiagnostics(SampleIdType p_id);

#endif /* SAMPLE_H */
```
</details>

**Integration with VSCode** 

`clang-tidy` and `clang-format` are both included with the C/C++ extension. But you need additionly configure your envoirenment in order to use them:

 1. Add thoose paremeters to your `settings.json` file
```json
    {
        "C_Cpp.clang_format_style": "file",             // so it will parse file named ".clang-format"
        "C_Cpp.codeAnalysis.clangTidy.enabled": true,   // so you don't need invoke "Run Code Analysis" command manually
    }
```

 2. Make sure, you have files `.clang-format and` `.clang-tidy` is in your root folder. They should be in repository.
 3. Open desired file in the repository and press `Format Document`.  Dialog window should appear with avaiable formatting options. Choose `C/C++ (default)` option
 4. *Optional: You can test, if formatting works, by changing `ColumnLimit` or any other option in `.clang-format` file* 

 *Extra reading 1: [VSCode update: Clang-tidy](https://devblogs.microsoft.com/cppblog/visual-studio-code-c-december-2021-update-clang-tidy/)* </br>
 *Extra reading 2: [VSCode IDE, Code formatting, Clang](https://code.visualstudio.com/docs/cpp/cpp-ide#_code-formatting)* </br>
 *Extra reading 3: [Manual configuration tool](https://zed0.co.uk/clang-format-configurator/)* </br>

### CLI

1. Install PlatformIO [CLI](https://docs.platformio.org/en/latest/core/installation.html).
2. Make sure to add [the standard udev rules](https://docs.platformio.org/en/latest/faq.html#faq-udev-rules) of platformIO and also ST-link 2.1 specific ones:
```curl -fsSL https://raw.githubusercontent.com/stlink-org/stlink/develop/config/udev/rules.d/49-stlinkv2-1.rules | sudo tee /etc/udev/rules.d/49-stlinkv2-1.rules```

To build and upload, in project root, do
`pio run -t upload`


### Error: libusb_open() failed

Do:
`sudo apt -y install stlink-tools`
`sudo systemctl restart udev`
