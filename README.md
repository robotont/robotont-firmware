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
Some of the rules can be maintained automaticly using tool called `clang-format`. It is separate executable, that can reformat given file according to style configuration. You can link your configuration file to the `clang-format` executable and it will import all rules and reformat file accordingly. </br>

In `VSCode` you can install extension to in order to link `clang-format` program to the built-in auto-formatting. </br>
 </br>
**Step-by-step integration with VSCode:**

 1. Get `clang-format` ([LLVM Snapshot Builds](https://llvm.org/builds/) / [LLVM Debian/Ubuntu packages])(https://apt.llvm.org/)
 2. Make sure, that you have `clang-format` in your PATH or you know executable location
 3. Install `Clang-Format`  extension in VSCode
 4. Copy this to the `.vscode/settings.json` file:
 ```json
 {
"clang-format.executable": "C:/LLVM/bin/clang-format.exe", // Or 'clang-format' (from PATH)
"clang-format.style": "file",
"clang-format.assumeFilename": "${workspaceFolder}/.clang-format", // Should be present already
}
 ```
 5. Open desired file in the repository and press `Format Document`.  Dialog window should appear with avaiable formatting options. Choose `clang-format`.

 *P.S.* In addition, you can manually generate format config file, using tools such [Clang-Format Editor](https://clangpowertools.com/clang-format-editor.html) or [online version](https://zed0.co.uk/clang-format-configurator/)

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
