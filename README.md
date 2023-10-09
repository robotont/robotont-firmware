# Robotont mbed mainboard firmware

Simplistic firmware for the Robotex platform mainboard.

## Serial protocol

The virtual serial port on the mbed USB bus is used for bidirectional communication. Approximately 100 times a second, motor speeds are given (from -100 to 100 in arbitrary units, sorry...). The format is: `motor1:motor2:motor3\n`. In the same format You can also send data to the mainboard to set the motor speeds (from -100 to 100).

## Development

### Envoirenment setup using VSCode

Repository contains PlatformIO project. In order to setup envoirenment using VSCode:

 1. Install `C/C++` extension.
 2. Install `PlatformIO` extension and wait until core setup is done. Restart IDE.
 3. Go to `paltformio.ini` file and make sure that `upload_protocol = stlink`. If you are using Segger J-Link, read this [guide](https://docs.platformio.org/en/stable/plus/debug-tools/jlink.html)
 4. Connect debugger via SWD port and upload your code
 5. Add thoose paremeters to your `settings.json` file
```json
    {
        "C_Cpp.clang_format_style": "file",             // so it will parse file named ".clang-format"
        "C_Cpp.codeAnalysis.clangTidy.enabled": true,   // so you don't need invoke "Run Code Analysis" command manually
    }
```
 6. Make sure, you have files `.clang-format and` `.clang-tidy` is in your root folder. They should be in repository.
 7. Open desired file in the repository and press `Format Document`.  Dialog window should appear with avaiable formatting options. Choose `C/C++ (default)` option
 8. *Optional: You can test, if formatting works, by changing `ColumnLimit` or any other option in `.clang-format` file* 

  Explanation:
  - `clang-format`: Separate executable, that can reformat given file according to style rules. Rules can be configured.
  - `clang-tidy`: Linter tool, that highliht syntax and style issues (e.g. `snake_case` vs `camelCase`). Rules can be configured.

`clang-tidy` and `clang-format` are both included with the C/C++ extension. </br>
 *Extra reading 1: [VSCode update: Clang-tidy](https://devblogs.microsoft.com/cppblog/visual-studio-code-c-december-2021-update-clang-tidy/)* </br>
 *Extra reading 2: [VSCode IDE, Code formatting, Clang](https://code.visualstudio.com/docs/cpp/cpp-ide#_code-formatting)* </br>
 *Extra reading 3: [Manual configuration tool](https://zed0.co.uk/clang-format-configurator/)* </br>

### CLI envoirenment setup

 1. Install PlatformIO [CLI](https://docs.platformio.org/en/latest/core/installation.html).
 2. Make sure to add [the standard udev rules](https://docs.platformio.org/en/latest/faq.html#faq-udev-rules) of platformIO and also ST-link 2.1 specific ones:
```sh
curl -fsSL https://raw.githubusercontent.com/stlink-org/stlink/develop/config/udev/rules.d/49-stlinkv2-1.rules
sudo tee /etc/udev/rules.d/49-stlinkv2-1.rules
```
 3. To build and upload, in project root, do `pio run -t upload`

### Known issues

**Error: libusb_open() failed**
```sh
sudo apt -y install stlink-tools
sudo systemctl restart udev
```
